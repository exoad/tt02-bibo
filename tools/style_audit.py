"""Audits this repo's C++ against Jack's Style Guide as recorded in docs/conventions.md.

    python tools/style_audit.py

Exits 0 when clean, 1 otherwise. Rules about code ignore comments and literals.
"""
import io, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')

def at(*parts):
    return os.path.join(ROOT, *parts)

# Our code; vendor/ and third_party/ are upstream. Listed rather than walked, so
# adding a directory is a decision. A listed directory that does not exist is a
# violation, below.
DIRS = [
    at('viewer', 'src'),
    at('viewer', 'tests'),
    at('firmware', 'lib'),
    at('firmware', 'lib', 'chassis'),
    at('firmware', 'app'),
    at('firmware', 'tests'),
    at('firmware', 'pilot', 'src'),
    at('firmware', 'pilot', 'tests'),
    at('firmware', 'pilot', 'app'),
    at('firmware', 'pilot', 'tools'),
    at('firmware', 'pilot', 'programs'),
]

# C has no named casts, and `static inline` is its header-definition idiom. The
# cast waiver is counted and printed below rather than skipped silently.
C_ONLY_WAIVES = {'c-style cast', 'static inline'}

# shared.hxx defines the aliases, so it names the types they alias.
VOCAB_FILES  = {'shared.hxx'}
VOCAB_WAIVES = {'unaliased std type', 'bare builtin type'}

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

# A cast is matched by its shape, not by a list of target types: types are
# PascalCase, so a capitalised name in parens is a type while `(width) * 2` is
# arithmetic. Win32 types shout, the stdlib uses _t, Slamtec sl_.
CAST_TYPE = (r'(?:const\s+)?(?:(?:unsigned|signed)\s+)?'
             r'(?:[A-Z][A-Za-z0-9_]*(?:::[A-Za-z_]\w*)*'
             r'|\w+_t'
             r'|sl_\w+'
             r'|(?:unsigned|signed|int|float|double|char|short|long|bool|void)\b'
             r'(?:\s+(?:int|long|char))?'
             r')\s*\**\s*')

# Matched against the raw line: strip_noise blanks comment and literal contents,
# which is where these rules' targets live.
RAW_RULES = {'absolute user path', 'namespace trailer comment'}

RULES = [
    # (name, regex, note)
    # The username must start alphanumeric, or a `file:///C:/Users/...`
    # placeholder matches on the ellipsis.
    ('absolute user path',
     r'[A-Za-z](?::|%3[Aa])[\\/]{1,4}[Uu]sers[\\/]{1,4}'
     r'[A-Za-z0-9_][A-Za-z0-9_.-]*',
     'derive the path - a home directory in the tree names the machine'),
    # The `>` in the lookbehind keeps `static_cast<Size>(SRC) * 4` out: a named
    # cast whose result is multiplied.
    ('c-style cast',
     r'(?<![A-Za-z0-9_)\]>])\(\s*' + CAST_TYPE + r'\)\s*(?!&&|\|\|)[A-Za-z_(&*]',
     'use a named cast'),
    ('bare builtin type',
     r'(?<![A-Za-z_>:.])(?:unsigned\s+(?:int|char|short|long)|signed\s+char|\bint\b|\bfloat\b|\bdouble\b|\bbool\b|\bchar\b|\bsize_t\b|\bunsigned\b)(?![A-Za-z_0-9])',
     'use the shared.hxx alias'),
    # Nothing verifies it, and a rename leaves it wrong.
    ('namespace trailer comment',
     r'^\s*\}\s*//\s*namespace\b',
     'delete it - a closing brace does not need to say what it closes'),
    # `struct Foo f;`. A name must follow, which separates a use from a
    # definition or a forward declaration.
    ('elaborated type specifier',
     # The separator must be whitespace or a star, or `struct ID3D11Device;`
     # splits one identifier in two.
     r'(?<![A-Za-z0-9_])struct\s+[A-Za-z_]\w*(?:\s+|\s*\*+\s*)[A-Za-z_]\w*\s*[,;=)]',
     'drop the struct keyword - in C++ the name alone is the type'),
    ('if with space',   r'\bif\s+\(',      'if(cond)'),
    ('for with space',  r'\bfor\s+\(',     'for(...)'),
    ('while with space',r'\bwhile\s+\(',   'while(...)'),
    ('switch with space',r'\bswitch\s+\(', 'switch(...)'),
    ('PascalCase function',
     r'^\s*(?:static\s+)?(?:const\s+)?(?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|'
     r'UInt16|UInt32|UInt64|Float32|Float64|Size|Str|Char|Utf8|UINT|LRESULT|HRESULT)'
     r'\s+[A-Z][A-Za-z0-9]*\s*\(',
     'functions are camelCase'),
    ('snake_case function',
     r'^\s*(?:static\s+)?(?:const\s+)?(?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|'
     r'UInt16|UInt32|UInt64|Float32|Float64|Size|Str|Char|Utf8)'
     r'\s+[a-z][a-z0-9]*_[a-z0-9_]+\s*\(',
     'functions are camelCase'),
    ('k-prefixed constant', r'\bk[A-Z][A-Za-z0-9]*\b', 'SCREAMING_SNAKE_CASE'),
    ('m_ member',           r'\bm_[A-Za-z0-9_]+',      'camelCase, no m_'),
    ('g_ global',           r'\bg_[A-Za-z0-9_]+',      'camelCase, no g_'),
    ('trailing underscore', r'\b[a-z][A-Za-z0-9]*_\b(?!\s*\()', 'camelCase, no trailing _'),
    # Only the types are aliased, so they are named rather than banning std::.
    # The \b after `duration` spares duration_cast.
    ('unaliased std type',
     r'\bstd::(?:vector|deque|array|map|set|unordered_map|unordered_set|pair|'
     r'tuple|string|string_view|optional|variant|function|unique_ptr|'
     r'shared_ptr|weak_ptr|mutex|recursive_mutex|lock_guard|unique_lock|'
     r'thread|atomic|condition_variable|ifstream|ofstream|fstream'
     r'|chrono::(?:steady_clock|system_clock|high_resolution_clock|time_point'
     r'|milliseconds|microseconds|nanoseconds|seconds|duration)'
     r'|this_thread::sleep_for)\b',
     'use the shared.hxx alias (Vec, Str, Clock, TimePoint, sleepMs, ...)'),
    ('unaliased fixed-width integer',
     r'(?<![A-Za-z0-9_:.])(?:std::)?(?:u?int(?:8|16|32|64)_t|uintptr_t'
     r'|ptrdiff_t)(?![A-Za-z0-9_])',
     'use the vocabulary alias - Int32, UInt8, UPtr, ISize'),
    # Definitions and declarations only, told apart from a call (whose arguments
    # may wrap) by the leading return type.
    ('wrapped parameter list',
     r'^\s*(?:\[\[nodiscard\]\]\s*)?(?:static\s+|inline\s+|constexpr\s+|const\s+)*'
     r'(?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|UInt16|UInt32|UInt64|Float32|'
     r'Float64|Size|Str|Char|Utf8|CharSeq|Pin)[\w:<>,\s\*&]*?\s[\w:~]+\s*\([^)]*$',
     'put the whole parameter list on one line'),
    ('static inline',
     r'\bstatic\s+inline\b',
     'in C++, static already implies it - drop the inline'),
    # The `)` or keyword before the brace separates a body from an aggregate row.
    ('one-lined body',
     r'(?:\)|\b(?:else|do|try)\b)\s*(?:const\s*)?(?:noexcept\s*)?\{[^{}]*[^{}\s][^{}]*\}',
     'expand the braces onto their own lines'),
    # The brace on the head's line with the body below, which the rule above
    # cannot see.
    ('cuddled brace',
     r'(?:\)|\b(?:else|do|try)\b)\s*(?:const\s*)?(?:noexcept\s*)?\{\s*$',
     'Allman - put the brace on its own line'),
    # `if(x) return;`; a body on the next line is not this. The head's parens are
    # matched three levels deep, since `[^;]*` misses for heads and `[^)]*` stops
    # inside static_cast. The body may start with any non-space, so
    # `while(n) --n;` counts.
    ('braceless one-lined body',
     r'^\s*(?:if|while|for)\s*'
     r'\((?:[^()]|\((?:[^()]|\([^()]*\))*\))*\)'
     r'\s*[^\s{/;][^;]*;',
     'give the body its own braces on their own lines'),
    ('pointer bound to the name',
     r'\b[A-Z][A-Za-z0-9_]*\s+\*[a-z][A-Za-z0-9_]*',
     'bind the * to the type: Type* name'),
    ('reference bound to the name',
     r'\b[A-Z][A-Za-z0-9_]*\s+&[a-z][A-Za-z0-9_]*',
     'bind the & to the type: Type& name'),
]

EXEMPT = [
    (r'typedef .*WINAPI', 'Win32 ABI signature'),
    # `int main(` stays: on arm-none-eabi Int32 is `long int`, and main must
    # return int. MSVC, where Int32 is int, cannot see that.
    (r'int APIENTRY|WinMain|int main\(', 'the platform entry point signature'),
    (r'IMGUI_IMPL_API|ImGui_ImplWin32_WndProcHandler', 'third-party signature'),
    (r'static_cast<int>|static_cast<float>|static_cast<unsigned', 'named cast to an ABI type'),
    (r'#\s*(define|include|if|ifdef|ifndef|endif|else|elif|pragma)', 'preprocessor'),
    # The cast pattern cannot tell `(T)x` from `sizeof(T) * x`.
    (r'\bsizeof\s*\(', 'sizeof, not a cast'),
    (r'\busing\s+\w+\s*=\s*(float|double|bool|char|int|unsigned|std::)', 'the alias definition itself'),
    (r'^\s*typedef\s+\w+\s+\w+\s*;', 'the alias definition itself'),
    # Lambdas may keep a one-expression body on one line and cuddle their brace.
    (r'\[[^\]]*\]\s*\([^)]*\)\s*(?:->\s*[A-Za-z_:<>]+\s*)?\{', 'a lambda, not a function body'),
    (r'\[[&=]?[^\]]*\]\s*(?:\([^)]*\)\s*)?(?:mutable\s*)?(?:->[^{]*)?\{\s*$',
     'a lambda, not a function body'),
]

def exempt(line):
    for pat, why in EXEMPT:
        if re.search(pat, line):
            return why
    return None

def is_c(path):
    """.c and .h are C; C++ is .cxx and .hxx."""
    return path.endswith('.c') or path.endswith('.h')

def audit(paths):
    hits = {}
    for path in paths:
        code = strip_noise(rd(path))
        raw  = rd(path).split('\n')
        lines = code.split('\n')
        waived_here = C_ONLY_WAIVES if is_c(path) else set()
        if os.path.basename(path) in VOCAB_FILES:
            waived_here = waived_here | VOCAB_WAIVES
        for i, l in enumerate(lines):
            r = raw[i] if i < len(raw) else ''
            # Both: a comment-only line is blank once stripped, and raw rules
            # still apply to it.
            if not l.strip() and not r.strip():
                continue
            why = exempt(r)
            for name, pat, note in RULES:
                if name in waived_here:
                    continue
                subject = r if name in RAW_RULES else l
                if not subject.strip():
                    continue
                for m in re.finditer(pat, subject):
                    if why:
                        continue
                    hits.setdefault(name, []).append(
                        (os.path.basename(path), i + 1, m.group(0).strip(),
                         (raw[i] if i < len(raw) else '').strip()[:96]))
    return hits

def rd(p):
    return io.open(p, encoding='utf-8', errors='surrogateescape').read()

files = []
missing = []
for d in DIRS:
    if not os.path.isdir(d):
        # A violation, not a skip: a skipped directory drops out of the audit unseen.
        missing.append(os.path.relpath(d, ROOT))
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

# The layer each firmware file belongs to and what it may include, strictly
# downward: hal knows nothing, chassis knows hal, an app knows only the umbrella.
# The spelling is part of the rule: "../hal.hxx" from lib/chassis/, since the
# bare form needs -Ifirmware/lib and an editor without the project cannot find it.
LAYERS = {
    # pins.hxx declares facts, so naming pins::SERVO is reading downward.
    'firmware/lib':          {'shared.hxx', 'hal.hxx', 'pins.hxx'},
    'firmware/lib/chassis':  {'../hal.hxx', 'cal.hxx', '../pins.hxx'},
    'firmware/app':          {'../lib/bibo.hxx'},
    # A host test includes its one header, not the umbrella, which drags in the
    # SDK. chassis.hxx is tested through tests/fakes/hal.hxx.
    'firmware/tests':        {'../lib/text.hxx',
                              '../lib/pins.hxx',
                              '../lib/chassis/chassis.hxx'},
}

# Lib-root files that reach sideways, with the reason.
LAYER_EXTRA = {
    # pins.hxx formats its own conflict message; text.hxx is a leaf, so no cycle.
    'firmware/lib/pins.hxx': {'shared.hxx', 'text.hxx'},
    # The host-test fake, behind #ifdef BIBO_FAKE_HAL, off in every flashed image.
    'firmware/lib/hal.hxx': {'shared.hxx', '../tests/fakes/hal.hxx'},
    'firmware/lib/status.hxx': {'hal.hxx'},
    'firmware/lib/bibo.hxx': {'hal.hxx', 'text.hxx', 'pins.hxx', 'status.hxx',
                            'chassis/cal.hxx', 'chassis/chassis.hxx',
                            'shared.hxx'},
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
        # The Pico SDK is not a layer; hal.hxx is the file that reaches into it.
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

# rc.exe compiles resource.h; it is not a C++ header.
HEADER_EXEMPT = {'resource.h'}

# Counted, not failed: the casts C files would need rewritten as C++.
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
    # Say whether there are no casts or no C files: a check that cannot fire
    # must not read as clean.
    cFiles = [f for f in files if is_c(f)]
    if not cFiles:
        print('  no C files in scope - the C++ conversion is complete, and '
              'this pass has nothing left to count')
    else:
        print('  none')
else:
    for path in sorted(c_casts, key=lambda k: -c_casts[k]):
        print('  %-40s %4d' % (path, c_casts[path]))
    print('  %-40s %4d' % ('TOTAL', sum(c_casts.values())))

# firmware/app calls the library's wrappers, not libc, so the seam to the
# transport stays complete. lib/ is where the wrapping happens, so only app/ is
# checked.
print('\n--- application code reaching past the library ---')

LIBC_DIRECT = [
    ('printf',   'serial::printf'),
    # Listed on its own: the lookbehind below keeps `printf` from matching it.
    ('snprintf', 'text::format'),
    ('puts',     'serial::printLine'),
    ('fputs',    'serial::print'),
    ('strcmp',   'text::eq'),
    ('strncmp',  'text::starts'),
    ('strlen',   'text::len'),
    ('atoi',     'text::toInt'),
    ('atof',     'text::toFloat'),
    ('sscanf',   'text::twoInts'),
    ('toupper',  'text::upper'),
]

libc_bad = 0
for path in files:
    norm = path.replace('\\', '/')
    if '/firmware/app/' not in norm:
        continue
    code = strip_noise(rd(path))
    for i, line in enumerate(code.split('\n')):
        for name, instead in LIBC_DIRECT:
            # A whole word before a paren. `:` in the lookbehind spares
            # serial::printf; `.` and `>` spare a method called printf.
            if re.search(r'(?<![A-Za-z0-9_:.>])' + name + r'\s*\(', line):
                print('  %-22s %5d  %s( -> use %s('
                      % (os.path.basename(path), i + 1, name, instead))
                libc_bad += 1

if libc_bad == 0:
    print('  ok')
total += libc_bad

print('\n--- raw C arrays ---')
# `T name[N]` where T is ours: Array<T, N> keeps its length, a raw array decays
# to a pointer. A third-party element type (`BYTE data[512]` for RegEnumValueA)
# keeps its API's shape, and firmware/ is exempt: the Pico build has no Array.
ARRAY_ELEM_OURS = re.compile(
    r'^\s*(?:static\s+|const\s+|constexpr\s+|inline\s+|mutable\s+)*'
    r'(?:const\s+)?'
    r'(Char|Utf8|Bool|Size|Str|StrView|Int8|Int16|Int32|Int64'
    r'|UInt8|UInt16|UInt32|UInt64|Float32|Float64)'
    r'\s*\**\s+[A-Za-z_]\w*\s*\[[^\]]*\]\s*(?:=|;|,)')
NOT_A_DECL = re.compile(r'^\s*(?:return|if|for|while|switch|else|case|delete)\b')

raw_arrays = 0
for path in files:
    if '/firmware/' in path.replace('\\', '/'):
        continue
    code = strip_noise(rd(path)).split('\n')
    raw = rd(path).split('\n')
    for i, l in enumerate(code):
        if 'Array<' in l or NOT_A_DECL.match(l):
            continue
        if ARRAY_ELEM_OURS.match(l):
            print('  %-24s %5d  %s'
                  % (os.path.basename(path), i + 1,
                     (raw[i] if i < len(raw) else '').strip()[:60]))
            raw_arrays += 1

if raw_arrays == 0:
    print('  ok')
total += raw_arrays

print('\n--- signatures over 100 columns ---')
# A ratchet: the count may not rise above SIG_BUDGET, and SIG_BUDGET only falls.
SIG_BUDGET = 6

SIGNATURE = re.compile(
    r'^\s*(?:\[\[nodiscard\]\]\s*)?'
    r'(?:static\s+|inline\s+|constexpr\s+|const\s+|virtual\s+|explicit\s+)*'
    r'[A-Za-z_][\w:<>,\s\*&]*?\s[\w:~]+\s*\([^;]*\)\s*'
    r'(?:const\s*)?(?:noexcept\s*)?(?:override\s*)?[;{]?\s*$')
NOT_A_SIG = re.compile(r'^\s*(?:if|for|while|switch|return|else|case)\b')

long_sigs = []
for path in files:
    code = strip_noise(rd(path)).split('\n')
    raw = rd(path).split('\n')
    for i, l in enumerate(code):
        text = raw[i].rstrip() if i < len(raw) else ''
        if len(text) <= 100 or NOT_A_SIG.match(l) or not SIGNATURE.match(l):
            continue
        long_sigs.append((len(text), os.path.basename(path), i + 1))

long_sigs.sort(reverse=True)
print('  %d signature(s) over 100 columns, budget %d'
      % (len(long_sigs), SIG_BUDGET))
for cols, f, ln in long_sigs[:5]:
    print('    %4d  %-24s %5d' % (cols, f, ln))
if len(long_sigs) > SIG_BUDGET:
    print('  ^ that is MORE than the budget. Shorten a signature, or say why '
          'the budget moved.')
    total += 1
elif len(long_sigs) < SIG_BUDGET:
    print('  under budget - lower SIG_BUDGET to %d to keep the ratchet tight'
          % len(long_sigs))

print('\n--- enum member prefixes ---')
# Members are prefixed with their enum's name (MapMode::MAP_MODE_POINTS), so an
# unscoped enum is safe to `using` and a grep for MAP_MODE finds the family.

# Enums whose members may skip the prefix, name -> reason; each is printed.
ENUM_WAIVED = {}

def screamingOf(name):
    """MapMode -> MAP_MODE. Loss -> LOSS."""
    s = re.sub(r'(?<=[a-z0-9])(?=[A-Z])', '_', name)
    s = re.sub(r'(?<=[A-Z])(?=[A-Z][a-z])', '_', s)
    return s.upper()

ENUM_NAMED = re.compile(r'\benum\s+(?:class\s+|struct\s+)?([A-Z]\w*)\s*(?::[^{]*)?\{?')
ENUM_TYPEDEF = re.compile(r'^\s*typedef\s+enum\b')

enum_bad = 0
enum_waived = 0
for path in files:
    lines = strip_noise(rd(path)).split('\n')
    i = 0
    while i < len(lines):
        isTypedef = ENUM_TYPEDEF.match(lines[i])
        m = None if isTypedef else ENUM_NAMED.search(lines[i])
        if not isTypedef and not m:
            i += 1
            continue
        j, depth, started, body = i, 0, False, []
        while j < len(lines):
            depth += lines[j].count('{') - lines[j].count('}')
            if '{' in lines[j]:
                started = True
            body.append(lines[j])
            if started and depth == 0:
                break
            j += 1
        block = '\n'.join(body)
        if isTypedef:
            tail = re.search(r'\}\s*(\w+)\s*;', lines[j] if j < len(lines) else '')
            name = tail.group(1) if tail else None
        else:
            name = m.group(1)
        i = j + 1
        if not name or '{' not in block:
            continue
        want = screamingOf(name)
        inner = block[block.find('{') + 1:block.rfind('}')]
        for tok in inner.split(','):
            mem = tok.strip().split('=')[0].strip()
            if not re.fullmatch(r'[A-Za-z_]\w*', mem or ''):
                continue
            if mem.startswith(want + '_') or mem == want:
                continue
            if name in ENUM_WAIVED:
                enum_waived += 1
                continue
            print('  %-24s %-14s %-20s want %s_*'
                  % (os.path.basename(path), name, mem, want))
            enum_bad += 1

if enum_bad == 0:
    print('  ok')
for nm, why in sorted(ENUM_WAIVED.items()):
    print('  WAIVED  %-10s %d member(s) - %s' % (nm, enum_waived, why))
total += enum_bad

print('\n--- namespace layout ---')
# Allman brace, body indented two spaces per namespace level: most of the
# firmware sits two deep, and four would push every line eight columns right.
NS_SAME_LINE = re.compile(r'^\s*namespace(\s+[A-Za-z_][\w:]*)?\s*\{')
NS_OPEN_LINE = re.compile(r'^(?P<ind>\s*)namespace(\s+[A-Za-z_][\w:]*)?\s*$')

ns_bad_layout = 0
for path in files:
    code = strip_noise(rd(path))          # braces in prose are not structure
    lines = rd(path).split('\n')
    clean = code.split('\n')
    depth = 0
    ns_stack = []      # brace depths at which a namespace opened
    pending = False    # saw `namespace X`, its `{` is next
    for i, line in enumerate(lines):
        c = clean[i] if i < len(clean) else ''
        if NS_SAME_LINE.match(c):
            print('  %-26s %5d  brace on the namespace line'
                  % (os.path.basename(path), i + 1))
            ns_bad_layout += 1
        else:
            m = NS_OPEN_LINE.match(c)
            if m:
                want = ' ' * (2 * len(ns_stack))
                if m.group('ind') != want:
                    print('  %-26s %5d  namespace indent %d, want %d'
                          % (os.path.basename(path), i + 1,
                             len(m.group('ind')), len(want)))
                    ns_bad_layout += 1
                pending = True
        # Every line inside N namespaces starts at column 2N or deeper. A line
        # starting with `}` closes its namespace only if the depth falls to it:
        # `struct P { Float32 x, y; };` does not.
        stripped = line.strip()
        level = len(ns_stack)
        closes, opens = c.count('}'), c.count('{')
        if closes and ns_stack and c.strip().startswith('}') \
                and (depth - closes + opens) <= ns_stack[-1]:
            level -= 1
        if stripped and not stripped.startswith('#') and level > 0:
            indent = len(line) - len(line.lstrip())
            if indent < 2 * level:
                print('  %-26s %5d  indent %d, want at least %d'
                      % (os.path.basename(path), i + 1, indent, 2 * level))
                ns_bad_layout += 1
        for ch in c:
            if ch == '{':
                if pending:
                    ns_stack.append(depth)
                    pending = False
                depth += 1
            elif ch == '}':
                depth -= 1
                if ns_stack and depth == ns_stack[-1]:
                    ns_stack.pop()

if ns_bad_layout == 0:
    print('  ok')
total += ns_bad_layout

print('\n--- header guards ---')
guard_bad = 0
for path in files:
    if not path.endswith(('.hxx', '.h')):
        continue
    if os.path.basename(path) in HEADER_EXEMPT:
        continue
    if '#pragma once' not in rd(path):
        print('  %-30s no #pragma once' % os.path.basename(path))
        guard_bad += 1
if guard_bad == 0:
    print('  ok')
total += guard_bad

print('\n--- header extensions ---')
for path in files:
    f = os.path.basename(path)
    norm = path.replace('\\', '/')
    # A C header is a .h only under firmware/; elsewhere it is C++ misnamed.
    if f.endswith('.h') and f not in HEADER_EXEMPT and '/firmware/' not in norm:
        print('  .h outside firmware (C++ headers are .hxx):', path)
        total += 1
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
        # Upstream headers keep upstream's extension.
        if inc in HEADER_EXEMPT or inc.startswith(
                ('imgui', 'sl_lidar', 'stb_',
                 'pico/', 'hardware/', 'boards/')):
            continue
        # C sources include C headers.
        if is_c(path) or inc in ('shared.hxx', 'types.h'):
            continue
        print('  %s:%d  %s' % (os.path.basename(path), i + 1, l.strip()))
        total += 1

# Each module declares its expected namespace; a header that stops declaring
# one still compiles, its symbols moving to the global namespace. Not listed:
# hal.hxx (a set of namespaces, checked below), shared.hxx, cal.hxx (macros) and
# bibo.hxx (the umbrella).
MODULE_NAMESPACE = {
    'status.hxx':   'status',
    'text.hxx':     'text',
    'chassis.hxx':  'drive',
}

HAL_NAMESPACES = {'timing', 'serial', 'board', 'pwm', 'servo', 'led'}

print('\n--- namespaces ---')
ns_bad = 0
for path in files:
    # By path: a header outside firmware/lib may share a module's file name.
    if '/firmware/lib' not in path.replace('\\', '/'):
        continue
    base = os.path.basename(path)
    if base == 'hal.hxx':
        # Indented, and `bibo::pwm` counts as declaring `pwm`.
        have = set(re.findall(r'^\s*namespace (?:\w+::)*(\w+)\s*$',
                              rd(path), re.M))
        for want in sorted(HAL_NAMESPACES - have):
            print('  %-14s declares no namespace %s' % (base, want))
            ns_bad += 1
        continue
    want = MODULE_NAMESPACE.get(base)
    if want is None:
        continue
    if not re.search(r'^\s*namespace (?:\w+::)*%s\s*$' % re.escape(want),
                     rd(path), re.M):
        print('  %-14s declares no namespace %s' % (base, want))
        ns_bad += 1

total += ns_bad
total += struct_bad

# Nothing untracked belongs in the root. Unquoted source text in a shell command
# redirects: `grep -rn in->headOn .` writes a file called headOn, and a C++ line
# with `(a >> 11)` appends to one called `11)`, which can grow without bound.
# Catch them at zero bytes.
print('\n--- stray files in the repository root ---')
stray_bad = 0
try:
    import subprocess
    out = subprocess.check_output(
        ['git', 'ls-files', '--others', '--exclude-standard'],
        cwd=ROOT, stderr=subprocess.DEVNULL).decode('utf-8', 'replace')
except Exception as e:
    # A missing git is not a clean tree.
    print('  SKIPPED - could not ask git (%s)' % e.__class__.__name__)
    out = None

if out is not None:
    for name in sorted(l.strip() for l in out.split('\n') if l.strip()):
        if '/' in name:
            continue
        full = os.path.join(ROOT, name)
        if not os.path.isfile(full):
            continue
        size = os.path.getsize(full)
        print('  untracked file in root: %-24s %d byte(s)%s'
              % (name, size, '   <-- and it is GROWING' if size > (1 << 30) else ''))
        stray_bad += 1
    if stray_bad == 0:
        print('  ok')

total += stray_bad

print('\n%d file(s): %s' % (
    len(files),
    ', '.join(sorted(set(os.path.relpath(os.path.dirname(f), ROOT).replace('\\', '/')
                         for f in files)))))
print('%d violation(s), %d waived by EXEMPT' % (total, waived))
for m in missing:
    print('DIRS names a directory that does not exist: %s' % m)
total += len(missing)

sys.exit(0 if total == 0 else 1)
