#include "recording.hxx"

#include "settings.hxx"

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace rec {

namespace {

// The original binary format's magic. Still recognised on load, never
// written - see loadBinaryV1.
const Char MAGIC_V1[8] = { 'T', 'T', '0', '2', 'R', 'E', 'C', '1' };

// A revolution with more points than this is not a revolution, it is a corrupt
// length field. The C1 produces ~500; ten thousand is far past any plausible
// device and stops a bad file from asking for gigabytes.
constexpr UInt32 MAX_POINTS_PER_REV = 10000u;
constexpr UInt32 MAX_REVS           = 2000000u;

template <typename T>
Bool readPod(FILE* f, T& out)
{
    return std::fread(&out, sizeof(T), 1, f) == 1;
}

} // namespace

Void Recording::clear()
{
    revs.clear();
}

Void Recording::append(const LidarFrame& f, Float64 tS)
{
    Rev r;
    r.tS     = tS;
    r.hz     = f.hz;
    r.points = f.points;
    revs.push_back(std::move(r));
}

const Rev& Recording::at(Size i) const
{
    static const Rev none;
    return (i < revs.size()) ? revs[i] : none;
}

Float64 Recording::durationS() const
{
    if(revs.size() < 2u)
        return 0.0;
    return revs.back().tS - revs.front().tS;
}

Size Recording::pointCount() const
{
    Size n = 0;
    for(const Rev& r : revs)
        n += r.points.size();
    return n;
}

Size Recording::indexAt(Float64 tS) const
{
    if(revs.empty())
        return 0;

    // Linear from the front would be fine at these sizes, but a scrub drags
    // across the whole recording every frame, so this is the one place it is
    // worth being a binary search.
    Size lo = 0, hi = revs.size() - 1u;
    if(tS <= revs[0].tS)
        return 0;
    if(tS >= revs[hi].tS)
        return hi;

    while(lo + 1u < hi)
    {
        const Size mid = (lo + hi) / 2u;
        if(revs[mid].tS <= tS) lo = mid;
        else                   hi = mid;
    }
    return lo;
}

Bool Recording::save(const Str& path, Str& err) const
{
    err.clear();

    if(revs.empty())
    {
        err = "nothing recorded";
        return false;
    }

    // "wb" and explicit newlines, not "w": text mode on Windows turns every
    // newline into CRLF, which grows the file by its line count for nothing and
    // makes it differ byte for byte from the same recording written by the Pico.
    FILE* f = std::fopen(path.c_str(), "wb");
    if(f == nullptr)
    {
        err = "could not open " + path + " for writing";
        return false;
    }

    Bool ok = std::fprintf(f, "# tt02rec 2\n") > 0;
    ok = ok && std::fprintf(f, "# RPLIDAR C1 scan recording\n") > 0;
    ok = ok && std::fprintf(f,
        "# angle = centidegree delta from the previous point (first from 0)\n") > 0;
    ok = ok && std::fprintf(f,
        "# dist  = whole millimetres, 0 = no return on that bearing\n") > 0;
    ok = ok && std::fprintf(f, "# t     = milliseconds from the start\n") > 0;
    ok = ok && std::fprintf(f, "# revolutions %zu\n", revs.size()) > 0;

    for(Size i = 0; ok && i < revs.size(); ++i)
    {
        const Rev& r = revs[i];

        ok = ok && std::fprintf(f, "R %lld %d %zu\n",
                                static_cast<long long>(r.tS * 1000.0 + 0.5),
                                static_cast<Int32>(r.hz * 100.0f + 0.5f),
                                r.points.size()) > 0;

        Float32 prevDeg = 0.0f;
        for(Size k = 0; ok && k < r.points.size(); ++k)
        {
            const LidarPoint& p = r.points[k];

            // The delta wraps once per revolution, where the device's angle
            // rolls through 360. Folding it here keeps every written value
            // small and positive instead of emitting one -35900.
            Float32 d = p.angleDeg - prevDeg;
            while(d <   0.0f) d += 360.0f;
            while(d >= 360.0f) d -= 360.0f;
            prevDeg = p.angleDeg;

            ok = std::fprintf(f, "%d %d",
                              static_cast<Int32>(d * 100.0f + 0.5f),
                              static_cast<Int32>(p.distMm + 0.5f)) > 0;

            // Wrapped for a person, not for the parser - which reads `count`
            // pairs wherever they fall.
            ok = ok && (std::fputc(((k % 12u) == 11u) ? '\n' : ' ', f) != EOF);
        }
        ok = ok && (std::fputc('\n', f) != EOF);
    }

    // fclose can fail, and on a buffered write that is where a full disk shows
    // up. Ignoring it is how a truncated file gets reported as saved.
    const Bool closed = (std::fclose(f) == 0);
    if(!ok || !closed)
    {
        err = "write failed - the file may be incomplete";
        return false;
    }
    return true;
}

Bool Recording::load(const Str& path, Str& err)
{
    err.clear();
    revs.clear();

    FILE* f = std::fopen(path.c_str(), "rb");
    if(f == nullptr)
    {
        err = "could not open " + path;
        return false;
    }

    Char head[8] = {};
    const Size got = std::fread(head, 1, sizeof(head), f);
    std::fclose(f);

    if(got == sizeof(head) && std::memcmp(head, MAGIC_V1, sizeof(MAGIC_V1)) == 0)
        return loadBinaryV1(path, err);

    return loadTextV2(path, err);
}

Bool Recording::loadTextV2(const Str& path, Str& err)
{
    FILE* f = std::fopen(path.c_str(), "rb");
    if(f == nullptr)
    {
        err = "could not open " + path;
        return false;
    }

    Char line[64] = {};
    if(std::fgets(line, sizeof(line), f) == nullptr
       || std::strncmp(line, "# tt02rec", 9) != 0)
    {
        std::fclose(f);
        err = "not a tt02rec recording";
        return false;
    }

    // Token by token rather than line by line: the pair stream is wrapped for
    // readability and the parser must not care where.
    Rev     cur;
    Bool    inRev  = false;
    Size    want   = 0;
    Float32 runDeg = 0.0f;

    Char tok[64];
    while(std::fscanf(f, "%63s", tok) == 1)
    {
        if(tok[0] == '#')
        {
            Int32 c = 0;
            while((c = std::fgetc(f)) != EOF && c != '\n') { }
            continue;
        }

        if(tok[0] == 'R' && tok[1] == 0)
        {
            if(inRev)
                revs.push_back(std::move(cur));

            long long tms = 0, n = 0;
            Int32     hzc = 0;
            if(std::fscanf(f, "%lld %d %lld", &tms, &hzc, &n) != 3
               || n < 0 || n > static_cast<long long>(MAX_POINTS_PER_REV))
            {
                std::fclose(f);
                err = revs.empty() ? Str("revolution header is not readable")
                                   : Str("file is truncated; kept what was readable");
                return !revs.empty();
            }

            cur = Rev{};
            cur.tS = static_cast<Float64>(tms) / 1000.0;
            cur.hz = static_cast<Float32>(hzc) / 100.0f;
            cur.points.reserve(static_cast<Size>(n));
            want   = static_cast<Size>(n);
            runDeg = 0.0f;
            inRev  = true;
            continue;
        }

        if(!inRev)
            continue;

        // A pair; its first token is already in `tok`.
        const Int32 da = std::atoi(tok);
        Int32 dist = 0;
        if(std::fscanf(f, "%d", &dist) != 1)
            break;

        runDeg += static_cast<Float32>(da) / 100.0f;
        while(runDeg >= 360.0f) runDeg -= 360.0f;

        if(cur.points.size() < want)
        {
            LidarPoint p;
            p.angleDeg = runDeg;
            p.distMm   = static_cast<Float32>(dist);
            cur.points.push_back(p);
        }
    }

    if(inRev)
        revs.push_back(std::move(cur));

    std::fclose(f);

    if(revs.empty())
    {
        err = "no revolutions in the file";
        return false;
    }
    return true;
}

Bool Recording::loadBinaryV1(const Str& path, Str& err)
{
    // The original format. Read-only, and kept so that changing the format
    // never cost anybody a recording.
    FILE* f = std::fopen(path.c_str(), "rb");
    if(f == nullptr)
    {
        err = "could not open " + path;
        return false;
    }

    Char magic[8] = {};
    if(std::fread(magic, 1, sizeof(magic), f) != sizeof(magic))
    {
        std::fclose(f);
        err = "file ends in its header";
        return false;
    }

    UInt32 n = 0u, reserved = 0u;
    if(!readPod(f, n) || !readPod(f, reserved) || n > MAX_REVS)
    {
        std::fclose(f);
        err = "header is not readable";
        return false;
    }

    revs.reserve(n);
    for(UInt32 i = 0; i < n; ++i)
    {
        Rev    r;
        UInt32 pc = 0u;
        if(!readPod(f, r.tS) || !readPod(f, r.hz) || !readPod(f, pc)
           || pc > MAX_POINTS_PER_REV)
        {
            std::fclose(f);
            err = revs.empty() ? Str("file ends before any revolution")
                               : Str("file is truncated; kept what was readable");
            return !revs.empty();
        }

        r.points.resize(pc);
        Bool ok = true;
        for(UInt32 k = 0; ok && k < pc; ++k)
            ok = readPod(f, r.points[k].angleDeg) && readPod(f, r.points[k].distMm);

        if(!ok)
        {
            std::fclose(f);
            err = "file is truncated; kept what was readable";
            return !revs.empty();
        }
        revs.push_back(std::move(r));
    }

    std::fclose(f);
    err = "read as a v1 binary recording; save it again to convert";
    return true;
}

Str dir()
{
    const Str base = settings::dir();
    if(base.empty())
        return Str();

    const Str d = base + "\\recordings";
    if(!::CreateDirectoryA(d.c_str(), nullptr)
       && ::GetLastError() != ERROR_ALREADY_EXISTS)
        return Str();

    return d;
}

Vec<Str> list()
{
    Vec<Str> out;

    const Str d = dir();
    if(d.empty())
        return out;

    WIN32_FIND_DATAA fd = {};
    const Str pattern = d + "\\*.tt02rec";
    HANDLE h = ::FindFirstFileA(pattern.c_str(), &fd);
    if(h == INVALID_HANDLE_VALUE)
        return out;

    do
    {
        if((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
            out.push_back(fd.cFileName);
    }
    while(::FindNextFileA(h, &fd) != 0);
    ::FindClose(h);

    // Newest first. The names are timestamped and fixed-width, so sorting them
    // as text sorts them by time - which is why the name is built that way.
    std::sort(out.begin(), out.end(), [](const Str& a, const Str& b) { return a > b; });
    return out;
}

Str makeName()
{
    SYSTEMTIME t = {};
    ::GetLocalTime(&t);

    Char buf[64];
    std::snprintf(buf, sizeof(buf), "scan-%04d%02d%02d-%02d%02d%02d.tt02rec",
                  t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond);
    return Str(buf);
}

} // namespace rec
