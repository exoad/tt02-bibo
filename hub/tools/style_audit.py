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
ROOT = os.path.join(HERE, '..', '..')


def at(*parts):
    return os.path.join(ROOT, *parts)


# Everything in this repo that is OURS. vendor/ is upstream and is not audited;
# neither is hub/vendor's imgui copy.
#
# The list is explicit rather than a walk because the interesting mistake is a
# directory nobody remembered - hub/tests/board_preview sat one level below the
# old TESTS root and was never scanned, and firmware/ was never in scope at all.
# A walk would have hidden that by silently including them the day they appeared;
# an explicit list makes adding a directory a decision somebody makes.
DIRS = [
    at('hub', 'src'),
    at('hub', 'tests'),
    at('hub', 'tests', 'board_preview'),
    at('lidar', 'bridge'),
    at('firmware', 'lib'),
    at('firmware', 'lib', 'drivers'),
    at('firmware', 'lib', 'chassis'),
    at('firmware', 'app'),
    at('firmware', 'scratch'),
    at('firmware', 'tests'),
    at('shared'),
]

# C cannot follow two of the C++ rules, so they are not applied to it:
#
#   - named casts. C has no static_cast; `(Int64) x` is the only spelling there
#     is, and banning it would ban casting. NOT silently: the carve-out is
#     counted and reported at the end, because firmware/ is expected to become
#     C++ eventually and every one of those casts is work that move inherits.
#     A waiver nobody can see is a waiver that grows.
#   - the .hpp extension. docs/conventions.md carves this out explicitly - a
#     header that must compile as C is a .h, and that is firmware/ and
#     shared/shared.h.
#   - static inline. In C that is the idiom for a header definition, not
#     redundancy: without `static` each translation unit emits a copy and they
#     collide, and without `inline` the compiler need not inline it. C++ gets
#     internal linkage from `static` alone, which is why the rule applies there.
C_ONLY_WAIVES = {'c-style cast', 'static inline'}

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
     'use the shared.hxx alias'),

    ('if with space',   r'\bif\s+\(',      'if(cond)'),
    ('for with space',  r'\bfor\s+\(',     'for(...)'),
    ('while with space',r'\bwhile\s+\(',   'while(...)'),
    ('switch with space',r'\bswitch\s+\(', 'switch(...)'),

    # Found `static UINT DpiForWindow(HWND)` in main.cxx, three lines from the
    # Win32 GetDpiForWindow it wraps - which is exactly why it read as fine.
    ('PascalCase function',
     r'^\s*(?:static\s+)?(?:const\s+)?(?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|'
     r'UInt16|UInt32|UInt64|Float32|Float64|Size|Str|Char|Utf8|UINT|LRESULT|HRESULT)'
     r'\s+[A-Z][A-Za-z0-9]*\s*\(',
     'functions are camelCase'),

    # Found `static Void sleep_ms(Int32)` in test_pico_link.cxx, where it read
    # as the Pico SDK call it is named after and is not.
    ('snake_case function',
     r'^\s*(?:static\s+)?(?:const\s+)?(?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|'
     r'UInt16|UInt32|UInt64|Float32|Float64|Size|Str|Char|Utf8)'
     r'\s+[a-z][a-z0-9]*_[a-z0-9_]+\s*\(',
     'functions are camelCase'),

    ('k-prefixed constant', r'\bk[A-Z][A-Za-z0-9]*\b', 'SCREAMING_SNAKE_CASE'),
    ('m_ member',           r'\bm_[A-Za-z0-9_]+',      'camelCase, no m_'),
    ('g_ global',           r'\bg_[A-Za-z0-9_]+',      'camelCase, no g_'),
    ('trailing underscore', r'\b[a-z][A-Za-z0-9]*_\b(?!\s*\()', 'camelCase, no trailing _'),

    # The standard library is aliased in shared/shared.hxx for the same reason
    # the builtins are: a file that says `Int32 count` on one line and
    # `std::vector<std::string>` on the next has two naming schemes in it.
    #
    # std::move, std::min, std::sort and the rest of the FUNCTIONS keep their
    # spelling - only the TYPES are aliased, so this names them explicitly
    # rather than banning the namespace.
    ('unaliased std type',
     r'\bstd::(?:vector|deque|array|map|set|unordered_map|unordered_set|pair|'
     r'tuple|string|string_view|optional|variant|function|unique_ptr|'
     r'shared_ptr|weak_ptr|mutex|recursive_mutex|lock_guard|unique_lock|'
     r'thread|atomic)\b',
     'use the shared.hxx alias (Vec, Str, Map, Mutex, ...)'),

    # Allman, everywhere. A body on the same line as its head is the one brace
    # style question this project has already answered, and it is the one that
    # creeps back in every time somebody writes a two-line guard clause.
    #
    # Aggregate rows in a table are NOT this - `{ Icon::ICON_RADAR, "radar" },`
    # is data, and expanding it would quadruple every table in the tree for no
    # gain. The pattern below requires a `)` or a control keyword before the
    # brace, which is what separates a body from a row.
    # A parameter list that does not close on its own line.
    #
    # DEFINITIONS and DECLARATIONS only - a CALL may wrap, and often should,
    # because a call site's arguments are expressions and an expression is
    # allowed to be long. A signature is a contract, and a contract you have to
    # scroll to finish reading is one people stop reading.
    #
    # Anchored on a leading return type, which is what separates a signature
    # from a call: a call starts with an identifier.
    ('wrapped parameter list',
     r'^\s*(?:\[\[nodiscard\]\]\s*)?(?:static\s+|inline\s+|constexpr\s+|const\s+)*'
     r'(?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|UInt16|UInt32|UInt64|Float32|'
     r'Float64|Size|Str|Char|Utf8|CharSeq|Pin)[\w:<>,\s\*&]*?\s[\w:~]+\s*\([^)]*$',
     'put the whole parameter list on one line'),

    # `static` already gives a free function internal linkage, and a static
    # function is only ever emitted where it is used - so `inline` adds nothing
    # a C++ compiler did not already know. CLion says so too.
    #
    # C is a different language here and is waived below: `static inline` in a
    # C header is the standard idiom for a definition that must not collide
    # across translation units, and removing it there would be wrong.
    ('static inline',
     r'\bstatic\s+inline\b',
     'in C++, static already implies it - drop the inline'),

    ('one-lined body',
     r'(?:\)|\b(?:else|do|try)\b)\s*(?:const\s*)?(?:noexcept\s*)?\{[^{}]*[^{}\s][^{}]*\}',
     'expand the braces onto their own lines'),
]

# Lines that are legitimately exempt, with the reason.
EXEMPT = [
    # main.cxx resolves user32 entry points; the typedefs themselves are Win32
    # signatures and `int` there is the OS ABI, not our code's choice.
    (r'typedef .*WINAPI', 'Win32 ABI signature'),
    (r'int APIENTRY|WinMain|int main\(', 'the platform entry point signature'),
    (r'IMGUI_IMPL_API|ImGui_ImplWin32_WndProcHandler', 'third-party signature'),
    (r'static_cast<int>|static_cast<float>|static_cast<unsigned', 'named cast to an ABI type'),
    (r'#\s*(define|include|if|ifdef|ifndef|endif|else|elif|pragma)', 'preprocessor'),

    # `sizeof(Float32)` is not a cast. The cast pattern cannot tell the
    # difference between `(T)x` and `sizeof(T) * x` without a real parser.
    (r'\bsizeof\s*\(', 'sizeof, not a cast'),

    # shared.hxx is where the aliases are DEFINED. `using Float32 = float;` has
    # to name the builtin; that is the entire point of the file.
    (r'\busing\s+\w+\s*=\s*(float|double|bool|char|int|unsigned|std::)', 'the alias definition itself'),

    # shared.h is the same file for C. `typedef char Utf8;` is the definition,
    # not a use.
    (r'^\s*typedef\s+\w+\s+\w+\s*;', 'the alias definition itself'),

    # A lambda body genuinely reads better on one line when it is a single
    # expression - `[](const Str& a) { return a > b; }` as a sort predicate. The
    # rule targets function and control-flow bodies, not these.
    (r'\[[^\]]*\]\s*\([^)]*\)\s*(?:->\s*[A-Za-z_:<>]+\s*)?\{', 'a lambda, not a function body'),
]

def exempt(line):
    for pat, why in EXEMPT:
        if re.search(pat, line):
            return why
    return None

def is_c(path):
    """A .c or a .h.

    The extension IS the language now, with no carve-out list to keep in step:

        .c  .h    C
        .cxx .hxx C++

    That is why the C++ half moved off .cpp/.hpp - .h was already doing double
    duty as "a C header" and "a header somebody did not think about", and an
    unowned .h is assumed to be C++ by most editors, which is how a C header
    ends up flagged for using NULL instead of nullptr.
    """
    return path.endswith('.c') or path.endswith('.h')


def audit(paths):
    hits = {}
    for path in paths:
        code = strip_noise(rd(path))
        raw  = rd(path).split('\n')
        lines = code.split('\n')
        waived_here = C_ONLY_WAIVES if is_c(path) else set()
        for i, l in enumerate(lines):
            if not l.strip():
                continue
            why = exempt(raw[i] if i < len(raw) else '')
            for name, pat, note in RULES:
                if name in waived_here:
                    continue
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
for d in DIRS:
    if not os.path.isdir(d):
        continue
    for f in sorted(os.listdir(d)):
        if f.endswith(('.cxx', '.hxx', '.h', '.c')):
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

# ===========================================================================
# The structural pass: not what the code LOOKS like, but where it may reach.
#
# Formatting rules keep a file readable. These keep the ARCHITECTURE true - and
# the architecture is the thing that decays silently, because every individual
# violation is a reasonable-looking one-line include that solves somebody's
# immediate problem.
# ===========================================================================

# Which layer a firmware file belongs to, and what that layer may include.
#
# Strictly downward. hal knows nothing; drivers and chassis know hal; an app
# knows only the umbrella. A driver that needed another driver would be two
# things wearing one name, and the moment that is allowed the folders stop
# meaning anything.
#
# The SPELLING is part of the rule, not incidental. "../hal.h" rather than
# "hal.h" from lib/drivers/, because a quoted include is searched next to the
# including file first and hal.h is not there - the bare form compiles only
# because -Ifirmware/lib is set, and an editor without the project loaded then
# underlines every include in the library at once. Allowing both spellings here
# would let the unparseable one back in one file at a time.
LAYERS = {
    # hal.h is the floor everything stands on, so lib root may name it. hal.h
    # itself only needs shared.h, and naming itself is not a thing a file does.
    'firmware/lib':          {'types.h', 'hal.h'},
    'firmware/lib/drivers':  {'../hal.h'},
    'firmware/lib/chassis':  {'../hal.h', 'cal.h'},
    'firmware/app':          {'../lib/tt02.h'},
    'firmware/scratch':      {'../lib/tt02.h'},
    # A host test of ONE header includes that header, not the umbrella - the
    # umbrella drags in the SDK and these compile with MSVC.
    'firmware/tests':        {'../lib/text.h'},
}

# gfx draws INTO a Screen, so it is the one file at lib root that legitimately
# reaches sideways into a driver. Written down rather than special-cased in
# silence.
LAYER_EXTRA = {
    'firmware/lib/gfx.h':  {'drivers/display.h'},
    'firmware/lib/status.h': {'hal.h'},
    'firmware/lib/lights.h': {'hal.h'},
    'firmware/lib/tt02.h': {'hal.h', 'text.h', 'gfx.h', 'status.h',
                            'drivers/display.h',
                            'drivers/range.h', 'drivers/storage.h',
                            'chassis/cal.h', 'chassis/chassis.h',
                            'lights.h'},
}

def layer_of(path):
    p = path.replace('\\', '/')
    for key in sorted(LAYERS, key=len, reverse=True):
        if ('/' + key + '/') in ('/' + p):
            return key
    return None

print('\n--- include direction ---')
struct_bad = 0
for path in files:
    p = path.replace('\\', '/')
    key = None
    for k in sorted(LAYERS, key=len, reverse=True):
        if k in p:
            key = k
            break
    if key is None:
        continue

    allowed = set(LAYERS[key])
    for extra_path, extra in LAYER_EXTRA.items():
        if p.endswith(extra_path.split('firmware/')[-1]):
            allowed |= extra

    for i, line in enumerate(rd(path).split('\n')):
        t = line.strip()
        if not t.startswith('#include "'):
            continue
        what = t.split('"')[1]
        # The Pico SDK is not ours and is not a layer. hal.h exists precisely to
        # be the file that reaches into it, and an app naming pico/bootrom.h for
        # a reboot is honest about a dependency it genuinely has. The rule this
        # pass enforces is about the direction OUR headers point.
        if what.startswith(('pico/', 'hardware/', 'boards/')):
            continue
        if what in allowed:
            continue
        struct_bad += 1
        print('  %-28s %5d  includes %s' % (os.path.basename(path), i + 1, what))
        print('  %-28s        %s may include: %s'
              % ('', key, ', '.join(sorted(allowed)) or '(nothing)'))

if struct_bad == 0:
    print('  ok')

# resource.h is included by app.rc, which rc.exe compiles - not a C++ compiler,
# and not a C one either. Renaming it would break the resource build to satisfy
# a rule about C++ headers.
HEADER_EXEMPT = {'resource.h'}

# ---------------------------------------------------------------------------
# The C carve-out, counted.
#
# C has no static_cast, so `(Int64) x` is not a style failure there - it is the
# only spelling C has. But docs/conventions.md says named casts EVERYWHERE that
# the language allows them, and firmware/ is expected to move to C++.
#
# Silently skipping the rule would mean the size of that move is unknown until
# somebody starts it. This prints the bill instead. It is not a violation and
# does not fail the audit.
# ---------------------------------------------------------------------------
print('\n--- C-style casts in C files (legal in C, work if these become C++) ---')

CAST_PAT = [r for name, r, _ in RULES if name == 'c-style cast'][0]
c_casts = {}
for path in files:
    if not is_c(path):
        continue
    code = strip_noise(rd(path))
    n = 0
    for line in code.split('\n'):
        if exempt(line) is not None:
            continue
        n += len(re.findall(CAST_PAT, line))
    if n > 0:
        c_casts[os.path.relpath(path, ROOT).replace('\\', '/')] = n

if not c_casts:
    print('  none')
else:
    for path in sorted(c_casts, key=lambda k: -c_casts[k]):
        print('  %-40s %4d' % (path, c_casts[path]))
    print('  %-40s %4d' % ('TOTAL', sum(c_casts.values())))

# ---------------------------------------------------------------------------
# Application code uses the LIBRARY, not libc.
#
# firmware/lib wraps the C standard library so the project has one vocabulary:
# serialPrintf rather than printf, textEq rather than strcmp, textInt rather
# than atoi. The wrappers cost nothing - they are macros or static inline - and
# the point is not speed, it is that the seam is complete. A console calling
# printf() directly was the one place reaching past it, sixty-two times, and
# the day the transport is not stdio that is sixty-two call sites to find
# instead of one definition to change.
#
# Only app/ and scratch/ are checked. lib/ is WHERE the wrapping happens, so
# text.h naming strtol is the wrapper doing its job.
# ---------------------------------------------------------------------------
print('\n--- application code reaching past the library ---')

LIBC_DIRECT = [
    ('printf',   'serialPrintf'),
    ('puts',     'serialPrintLine'),
    ('fputs',    'serialPrint'),
    ('strcmp',   'textEq'),
    ('strncmp',  'textStarts'),
    ('strlen',   'textLen'),
    ('atoi',     'textInt'),
    ('atof',     'textFloat'),
    ('sscanf',   'textTwoInts'),
    ('toupper',  'textUpper'),
]

libc_bad = 0
for path in files:
    norm = path.replace('\\', '/')
    if '/firmware/app/' not in norm and '/firmware/scratch/' not in norm:
        continue
    code = strip_noise(rd(path))
    for i, line in enumerate(code.split('\n')):
        for name, instead in LIBC_DIRECT:
            # Whole word followed by a paren, so snprintf does not match printf
            # and textLen does not match strlen.
            if re.search(r'(?<![A-Za-z0-9_])' + name + r'\s*\(', line):
                print('  %-22s %5d  %s( -> use %s('
                      % (os.path.basename(path), i + 1, name, instead))
                libc_bad += 1

if libc_bad == 0:
    print('  ok')
total += libc_bad

print('\n--- header extensions ---')
for path in files:
    f = os.path.basename(path)
    norm = path.replace('\\', '/')

    # A C header under firmware/ is correctly a .h. Anywhere else, a .h is a
    # C++ header wearing the wrong extension.
    if f.endswith('.h') and f not in HEADER_EXEMPT and '/firmware/' not in norm:
        print('  .h outside firmware (C++ headers are .hxx):', path)
        total += 1

    # The old spellings, so a file copied in from elsewhere is caught.
    if f.endswith('.hpp'):
        print('  .hpp (C++ headers are .hxx):', path)
        total += 1
    if f.endswith('.cpp'):
        print('  .cpp (C++ sources are .cxx):', path)
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
        # C sources include C headers. shared.h and pico2w.h are .h because they
        # must be, so including them by that name is correct.
        if is_c(path) or inc in ('shared.hxx', 'types.h'):
            continue
        print('  %s:%d  %s' % (os.path.basename(path), i + 1, l.strip()))
        total += 1

total += struct_bad

print('\n%d file(s): %s' % (
    len(files),
    ', '.join(sorted(set(os.path.relpath(os.path.dirname(f), ROOT).replace('\\', '/')
                         for f in files)))))
print('%d violation(s), %d waived by EXEMPT' % (total, waived))
sys.exit(0 if total == 0 else 1)
