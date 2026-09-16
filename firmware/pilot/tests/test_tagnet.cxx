// tagnet: the NPU detector's CPU half over a tag drawn by hand, with the
// heatmaps a perfect network would give, on every platform.
#include "shared.hxx"

#include "tag36h11_codes.hxx"
#include "tagnet.hxx"

#include <cmath>
#include <cstdio>

static Int32 failures = 0;
static Int32 checks = 0;

static Void check(Bool ok, const Char* what)
{
    ++checks;
    if(ok)
    {
        std::printf("  ok    %s\n", what);
    }
    else
    {
        std::printf("  FAIL  %s\n", what);
        ++failures;
    }
}

// A picture: grey 128 everywhere, then a tag whose black border runs from
// (left, top) for `side` pixels, with the white margin cell outside it, turned
// `turns` quarter turns clockwise. Cells are side / 8 pixels.
static Vec<UInt8> picture(UInt16 id, Int32 left, Int32 top, Int32 side, Int32 turns)
{
    Vec<UInt8> grey(static_cast<Size>(tagnet::IN_W * tagnet::IN_H), 128);
    const Float32 cell = static_cast<Float32>(side) / 8.0f;
    const UInt64 code = tagnet::TAG36H11_CODES[id];
    for(Int32 y = top - static_cast<Int32>(cell); y < top + side + static_cast<Int32>(cell); ++y)
    {
        for(Int32 x = left - static_cast<Int32>(cell); x < left + side + static_cast<Int32>(cell); ++x)
        {
            if(x < 0 || y < 0 || x >= tagnet::IN_W || y >= tagnet::IN_H)
            {
                continue;
            }
            // Cell coordinates in the 10x10 frame, 0..9, with the border at 1 and 8.
            Int32 cx = static_cast<Int32>(std::floor((x - left) / cell)) + 1;
            Int32 cy = static_cast<Int32>(std::floor((y - top) / cell)) + 1;
            for(Int32 t = 0; t < turns; ++t)
            {
                const Int32 nx = 9 - cy;
                const Int32 ny = cx;
                cx = nx;
                cy = ny;
            }
            UInt8 v = 235;
            if(cx >= 1 && cx <= 8 && cy >= 1 && cy <= 8)
            {
                v = 18;
                if(cx >= 2 && cx <= 7 && cy >= 2 && cy <= 7)
                {
                    const Int32 bit = (cy - 2) * 6 + (cx - 2);
                    v = ((code >> (35 - bit)) & 1u) != 0u ? 235 : 18;
                }
            }
            grey[static_cast<Size>(y * tagnet::IN_W + x)] = v;
        }
    }
    return grey;
}

static Void splat(Vec<Float32>& heat, Int32 channel, Float32 x, Float32 y, Float32 sigma)
{
    const Int32 cx = static_cast<Int32>(std::lround(x));
    const Int32 cy = static_cast<Int32>(std::lround(y));
    for(Int32 yy = cy - 4; yy <= cy + 4; ++yy)
    {
        for(Int32 xx = cx - 4; xx <= cx + 4; ++xx)
        {
            if(xx < 0 || yy < 0 || xx >= tagnet::OUT_W || yy >= tagnet::OUT_H)
            {
                continue;
            }
            const Float32 d2 = (xx - x) * (xx - x) + (yy - y) * (yy - y);
            Float32& at = heat[static_cast<Size>(channel * tagnet::OUT_W * tagnet::OUT_H + yy * tagnet::OUT_W + xx)];
            at = std::max(at, std::exp(-d2 / (2.0f * sigma * sigma)));
        }
    }
    heat[static_cast<Size>(channel * tagnet::OUT_W * tagnet::OUT_H + cy * tagnet::OUT_W + cx)] = 1.0f;
}

// The heatmaps for a tag whose black border is the square (left, top, side).
static Vec<Float32> heatmaps(Int32 left, Int32 top, Int32 side)
{
    Vec<Float32> heat(tagnet::HEAT_VALUES, 0.0f);
    const Float32 s = static_cast<Float32>(tagnet::STRIDE);
    splat(heat, 0, (left + side / 2.0f) / s, (top + side / 2.0f) / s, 1.5f);
    splat(heat, 1, left / s, top / s, 1.2f);
    splat(heat, 1, (left + side) / s, top / s, 1.2f);
    splat(heat, 1, (left + side) / s, (top + side) / s, 1.2f);
    splat(heat, 1, left / s, (top + side) / s, 1.2f);
    return heat;
}

static Bool near(const tagnet::Corner& c, Float32 x, Float32 y, Float32 tol)
{
    return std::fabs(c.x - x) <= tol && std::fabs(c.y - y) <= tol;
}

int main()
{
    std::printf("tagnet: heatmaps to tags\n");
    std::printf("\n-- the family table --\n");
    {
        check(tagnet::TAG36H11_COUNT == 587, "587 tags in tag36h11");
        UInt16 id = 0;
        UInt8 ham = 0;
        check(
            tagnet::matchCode(tagnet::TAG36H11_CODES[5], &id, &ham) && id == 5 && ham == 0,
            "a code names its tag exactly"
        );
        check(
            tagnet::matchCode(tagnet::TAG36H11_CODES[5] ^ 0x5u, &id, &ham) && id == 5 && ham == 2,
            "two flipped bits still name it, hamming 2"
        );
        check(!tagnet::matchCode(tagnet::TAG36H11_CODES[5] ^ 0x7u, &id, &ham), "three do not");
        // The table's own rotation: a code turned a quarter is still its tag.
        UInt64 turned = 0;
        for(Int32 r = 0; r < 6; ++r)
        {
            for(Int32 c = 0; c < 6; ++c)
            {
                const Int32 sr = 5 - c;
                const Int32 sc = r;
                turned = (turned << 1) | ((tagnet::TAG36H11_CODES[77] >> (35 - (sr * 6 + sc))) & 1u);
            }
        }
        check(
            tagnet::matchCode(turned, &id, &ham) && id == 77 && ham == 0,
            "a code turned a quarter names the same tag"
        );
    }
    std::printf("\n-- peaks --\n");
    {
        Vec<Float32> heat(tagnet::HEAT_VALUES, 0.0f);
        splat(heat, 0, 20.0f, 10.0f, 2.0f);
        splat(heat, 0, 60.0f, 40.0f, 1.0f);
        const Vec<tagnet::Peak> p = tagnet::peaks(heat.data(), tagnet::CENTRE_THRESHOLD);
        check(p.size() == 2, "two peaks, not the shoulders of the wide one");
        check(
            p.size() == 2 && near(tagnet::Corner{ p[0].x, p[0].y }, 20.0f, 10.0f, 0.1f),
            "at their centres"
        );
    }
    std::printf("\n-- a tag drawn by hand --\n");
    {
        const Vec<UInt8> grey = picture(5, 100, 60, 120, 0);
        const Vec<Float32> heat = heatmaps(100, 60, 120);
        const Vec<tagnet::Found> found = tagnet::detect(grey.data(), heat.data());
        check(found.size() == 1, "one tag found");
        if(found.size() == 1)
        {
            check(found[0].id == 5 && found[0].hamming == 0, "id 5, read clean");
            check(found[0].margin > 80.0f, "with a wide margin on a clean print");
            Bool onSquare = true;
            for(const tagnet::Corner& c : found[0].corners)
            {
                const Bool xOk = std::fabs(c.x - 100.0f) < 2.5f || std::fabs(c.x - 220.0f) < 2.5f;
                const Bool yOk = std::fabs(c.y - 60.0f) < 2.5f || std::fabs(c.y - 180.0f) < 2.5f;
                onSquare = onSquare && xOk && yOk;
            }
            check(onSquare, "corners on the black border within 2.5 px");
        }
        // Turned: the same tag.
        for(Int32 turns = 1; turns < 4; ++turns)
        {
            const Vec<UInt8> turnedPic = picture(5, 100, 60, 120, turns);
            const Vec<tagnet::Found> f = tagnet::detect(turnedPic.data(), heat.data());
            check(f.size() == 1 && f[0].id == 5, "turned a quarter, still tag 5");
        }
        // Extra corner peaks around: a quantized network's habit.
        Vec<Float32> noisy = heat;
        splat(noisy, 1, 100.0f / 4 + 3.0f, 60.0f / 4 + 2.0f, 1.0f);
        splat(noisy, 1, 220.0f / 4 - 2.0f, 180.0f / 4 + 3.0f, 1.0f);
        splat(noisy, 1, 160.0f / 4, 60.0f / 4, 1.0f);
        const Vec<tagnet::Found> f = tagnet::detect(grey.data(), noisy.data());
        check(
            f.size() == 1 && f[0].id == 5 && f[0].hamming == 0,
            "three stray corners nearby, and the code still picks the right four"
        );
        // No tag under the peaks: a centre in blank grey decodes nothing.
        const Vec<UInt8> blank(static_cast<Size>(tagnet::IN_W * tagnet::IN_H), 128);
        check(
            tagnet::detect(blank.data(), heat.data()).empty(),
            "peaks over blank grey: nothing, since the border and the ring look alike"
        );
        // A tag half off the picture: refused, not read.
        const Vec<UInt8> edge = picture(9, 260, 60, 120, 0);
        const Vec<Float32> edgeHeat = heatmaps(260, 60, 120);
        check(
            tagnet::detect(edge.data(), edgeHeat.data()).empty(),
            "a tag past the edge is not decoded"
        );
    }
    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
