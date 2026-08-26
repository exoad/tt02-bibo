"""Audits hub/ against Jack's C++ Style Guide as recorded in docs/conventions.md.

    python tools/style_audit.py

Exits 0 when clean, 1 otherwise, so it can gate a commit.

Comment- and string-aware: a rule about code must not fire on prose, and this
file is full of prose describing the rules it enforces.

Written after the 2026-08-25 audit found 151 violations - all but a handful of
them in tests/, which the original rename pass had skipped entirely and left
UNCOMPILABLE for weeks. A style rule nobody can check is a style rule that
decays silently; this is the check.
"""
import io, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
TESTS = os.path.join(HERE, '..', 'tests')

def strip_noise(text):
    """Blank out // comments, /* */ comments and "..." literals, keeping
    offsets so line numbers stay right."""
    out = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c == '/' and i + 1 < n and text[i+1] == '/':
            while i < n and text[i] != '\n':
                out.append(' '); i += 1
        elif c == '/' and i + 1 < n and text[i+1] == '*':
            while i < n and not (text[i] == '*' and i + 1 < n and text[i+1] == '/'):
                out.append('\n' if text[i] == '\n' else ' '); i += 1
            out.append('  '); i += 2
        elif c == '"':
            out.append(' '); i += 1
            while i < n and text[i] != '"':
                if text[i] == '\\': out.append(' '); i += 1
                if i < n: out.append('\n' if text[i] == '\n' else ' '); i += 1
            out.append(' '); i += 1
        elif c == "'":
            out.append(' '); i += 1
            while i < n and text[i] != "'":
                if text[i] == '\\': out.append(' '); i += 1
                if i < n: out.append(' '); i += 1
            out.append(' '); i += 1
        else:
            out.append(c); i += 1
    return ''.join(out)

RULES = [
    # (name, regex, note)
    ('c-style cast',
     r'\((?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|UInt16|UInt32|UInt64|Float32|Float64|Size|Char|MapMode|ImU32|ImWchar|board::Which|scene3d::SceneMode|LidarState|PicoState|FlashState|PFN_[A-Za-z]+|HWND|HMODULE|DpiAwarenessContext|LPARAM|WPARAM)\)\s*[A-Za-z_(&*]',
     'use a named cast'),

    ('bare builtin type',
     r'(?<![A-Za-z_>:.])(?:unsigned\s+(?:int|char|short|long)|signed\s+char|\bint\b|\bfloat\b|\bdouble\b|\bbool\b|\bchar\b|\bsize_t\b|\bunsigned\b)(?![A-Za-z_0-9])',
     'use the shared.hpp alias'),

    ('if with space',   r'\bif\s+\(',      'if(cond)'),
    ('for with space',  r'\bfor\s+\(',     'for(...)'),
    ('while with space',r'\bwhile\s+\(',   'while(...)'),
    ('switch with space',r'\bswitch\s+\(', 'switch(...)'),

    ('k-prefixed constant', r'\bk[A-Z][A-Za-z0-9]*\b', 'SCREAMING_SNAKE_CASE'),
    ('m_ member',           r'\bm_[A-Za-z0-9_]+',      'camelCase, no m_'),
    ('g_ global',           r'\bg_[A-Za-z0-9_]+',      'camelCase, no g_'),
    ('trailing underscore', r'\b[a-z][A-Za-z0-9]*_\b(?!\s*\()', 'camelCase, no trailing _'),
]

# Lines that are legitimately exempt, with the reason.
EXEMPT = [
    # main.cpp resolves user32 entry points; the typedefs themselves are Win32
    # signatures and `int` there is the OS ABI, not our code's choice.
    (r'typedef .*WINAPI', 'Win32 ABI signature'),
    (r'int APIENTRY|WinMain|int main\(', 'the platform entry point signature'),
    (r'IMGUI_IMPL_API|ImGui_ImplWin32_WndProcHandler', 'third-party signature'),
    (r'static_cast<int>|static_cast<float>|static_cast<unsigned', 'named cast to an ABI type'),
    (r'#\s*(define|include|if|ifdef|ifndef|endif|else|elif|pragma)', 'preprocessor'),

    # `sizeof(Float32)` is not a cast. The cast pattern cannot tell the
    # difference between `(T)x` and `sizeof(T) * x` without a real parser.
    (r'\bsizeof\s*\(', 'sizeof, not a cast'),

    # shared.hpp is where the aliases are DEFINED. `using Float32 = float;` has
    # to name the builtin; that is the entire point of the file.
    (r'\busing\s+\w+\s*=\s*(float|double|bool|char|int|unsigned|std::)', 'the alias definition itself'),
]

def exempt(line):
    for pat, why in EXEMPT:
        if re.search(pat, line):
            return why
    return None

def audit(paths):
    hits = {}
    for path in paths:
        code = strip_noise(rd(path))
        raw  = rd(path).split('\n')
        lines = code.split('\n')
        for i, l in enumerate(lines):
            if not l.strip():
                continue
            why = exempt(raw[i] if i < len(raw) else '')
            for name, pat, note in RULES:
                for m in re.finditer(pat, l):
                    if why:
                        continue
                    hits.setdefault(name, []).append(
                        (os.path.basename(path), i + 1, m.group(0).strip(),
                         (raw[i] if i < len(raw) else '').strip()[:96]))
    return hits

def rd(p):
    return io.open(p, encoding='utf-8', errors='surrogateescape').read()

files = []
for d in (SRC, TESTS):
    if not os.path.isdir(d):
        continue
    for f in sorted(os.listdir(d)):
        if f.endswith(('.cpp', '.hpp', '.h')):
            files.append(os.path.join(d, f))

hits = audit(files)

total = 0
waived = 0
for name in [r[0] for r in RULES]:
    v = [h for h in hits.get(name, []) if exempt(h[3]) is None]
    w = len(hits.get(name, [])) - len(v)
    waived += w
    if not v:
        continue
    total += len(v)
    print('\n== %s (%d) ==' % (name, len(v)))
    for f, ln, tok, src in v[:24]:
        print('  %-22s %5d  %-18s %s' % (f, ln, tok, src))
    if len(v) > 24:
        print('  ... and %d more' % (len(v) - 24))

# resource.h is included by app.rc, which rc.exe compiles - not a C++ compiler,
# and not a C one either. Renaming it would break the resource build to satisfy
# a rule about C++ headers.
HEADER_EXEMPT = {'resource.h'}

print('\n--- header extensions ---')
for d in (SRC, TESTS):
    for f in sorted(os.listdir(d)):
        if f.endswith('.h') and f not in HEADER_EXEMPT:
            print('  .h header (should be .hpp):', f)
            total += 1

print('\n--- includes of .h project headers ---')
for path in files:
    for i, l in enumerate(rd(path).split('\n')):
        m = re.match(r'\s*#include\s+"([^"]+\.h)"', l)
        if not m:
            continue
        inc = m.group(1)
        # Third-party headers keep whatever extension upstream gave them.
        if inc in HEADER_EXEMPT or inc.startswith(('imgui', 'sl_lidar', 'stb_')):
            continue
        print('  %s:%d  %s' % (os.path.basename(path), i + 1, l.strip()))
        total += 1

print('\n%d violation(s), %d waived by EXEMPT' % (total, waived))
sys.exit(0 if total == 0 else 1)
