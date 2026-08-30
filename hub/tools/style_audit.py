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
    at('firmware', 'sketches'),
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

# shared.hxx is WHERE the aliasing happens, so it is exempt from the rule that
# everything else use the aliases - the same carve-out, and for the same
# reason, that lets firmware/lib/text.hxx name strtol. `using Str = std::string`
# and sleepMs's one-line body naming std::this_thread are the file doing its
# job, not a file that forgot the vocabulary.
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

# A cast is SYNTAX, not a list of type names.
#
# This rule used to enumerate the types a cast could be TO - which meant it
# only ever found casts to types somebody had remembered to add. It missed
# `(MINMAXINFO*)lparam` and `(const RECT*)lparam` in main.cxx and `(sl_u32)baud`
# in lidar_source.cxx for as long as they existed, because those three names
# were not on the list, and no list is ever finished.
#
# What makes a cast recognisable instead is the SHAPE: an opening paren that
# does not follow an identifier (which would make it a call), a TYPE, a closing
# paren, and an operand. The type is the hard part, and the project's own
# naming convention settles it - types are PascalCase here, so a capitalised
# name in parens is a type while `(width) * 2` is arithmetic on a camelCase
# local. Win32 shouts (HWND, RECT, LPARAM), the standard library uses _t, and
# Slamtec uses sl_, so those three get named explicitly.
CAST_TYPE = (r'(?:const\s+)?(?:(?:unsigned|signed)\s+)?'
             r'(?:[A-Z][A-Za-z0-9_]*(?:::[A-Za-z_]\w*)*'
             r'|\w+_t'
             r'|sl_\w+'
             r'|(?:unsigned|signed|int|float|double|char|short|long|bool|void)\b'
             r'(?:\s+(?:int|long|char))?'
             r')\s*\**\s*')

RULES = [
    # (name, regex, note)
    #
    # The `>` in the lookbehind keeps `static_cast<Size>(SRC) * 4` out: that is
    # a named cast whose RESULT is multiplied, not a cast of `(SRC)`. Without
    # it every correctly-written cast followed by a `*` reported itself.
    ('c-style cast',
     r'(?<![A-Za-z0-9_)\]>])\(\s*' + CAST_TYPE + r'\)\s*(?!&&|\|\|)[A-Za-z_(&*]',
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
    # chrono and the file streams joined the list when they got aliases. They
    # were the biggest hole in it: 31 raw std::chrono uses across the two
    # files that own threaded I/O, invisible because nobody had added the
    # name here and there was no alias to point at.
    #
    # `duration` but not `duration_cast`, and `this_thread::sleep_for` but not
    # the namespace: only the TYPES are aliased, and a cast and a sleep are
    # functions, which keep their spelling like std::move and std::sort do.
    # The \b after `duration` is what separates the two - `duration_cast`
    # continues with a word character, so it never matches.
    ('unaliased std type',
     r'\bstd::(?:vector|deque|array|map|set|unordered_map|unordered_set|pair|'
     r'tuple|string|string_view|optional|variant|function|unique_ptr|'
     r'shared_ptr|weak_ptr|mutex|recursive_mutex|lock_guard|unique_lock|'
     r'thread|atomic|ifstream|ofstream|fstream'
     r'|chrono::(?:steady_clock|system_clock|high_resolution_clock|time_point'
     r'|milliseconds|microseconds|nanoseconds|seconds|duration)'
     r'|this_thread::sleep_for)\b',
     'use the shared.hxx alias (Vec, Str, Clock, TimePoint, sleepMs, ...)'),

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

    # The brace on the HEAD's line, body closing somewhere far below.
    #
    # The rule above cannot see this: it needs the open and close brace on one
    # physical line, and a cuddled `{` whose body closes twenty lines later
    # never appears on one line at all. That blind spot hid 40 of these in
    # lidar_source.cxx and its test - the Slamtec boundary files, which were
    # carrying the SDK's brace style rather than this project's.
    #
    # A lambda is the written-down exception and is filtered in EXEMPT.
    ('cuddled brace',
     r'(?:\)|\b(?:else|do|try)\b)\s*(?:const\s*)?(?:noexcept\s*)?\{\s*$',
     'Allman - put the brace on its own line'),

    # `if(x) return;` - a body with no braces, sharing its head's line.
    #
    # docs/conventions.md bans a body sharing a line with its head "however
    # short", and a braceless statement is still a body. Nothing caught these
    # because every other brace rule looks for a brace, and this is the shape
    # that has none: 250 of them across the tree.
    #
    # A body on the NEXT line without braces is NOT this and is left alone -
    # it does not share the head's line, which is what the rule is about.
    ('braceless one-lined body',
     r'^\s*(?:if|for|while)\s*\([^;]*\)\s*(?!$)[A-Za-z_][\w:.>()\[\]-]*\s*(?:\(|=|\+\+|--|;)',
     'give the body its own braces on their own lines'),

    # `Type *name` / `Type &name`.
    #
    # The declaration binds the * to the TYPE here - `Char* p`, not `Char *p` -
    # because in this project a pointer to Char is a type in its own right and
    # is spelled as one. The tree was already 604:0 and 747:0 this way when the
    # rule was written; it is here so it stays that way, not to fix a mess.
    ('pointer bound to the name',
     r'\b[A-Z][A-Za-z0-9_]*\s+\*[a-z][A-Za-z0-9_]*',
     'bind the * to the type: Type* name'),

    ('reference bound to the name',
     r'\b[A-Z][A-Za-z0-9_]*\s+&[a-z][A-Za-z0-9_]*',
     'bind the & to the type: Type& name'),
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

    # A lambda's brace cuddles by convention everywhere, including here - it is
    # an expression being passed to something, not a function body standing on
    # its own. `const auto flush = [&]() {` is the shape.
    (r'\[[&=]?[^\]]*\]\s*(?:\([^)]*\)\s*)?(?:mutable\s*)?(?:->[^{]*)?\{\s*$',
     'a lambda, not a function body'),
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
        if os.path.basename(path) in VOCAB_FILES:
            waived_here = waived_here | VOCAB_WAIVES
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
    # pins.hxx sits beside types.hxx in the layering and for the same reason:
    # it declares facts and includes nothing but types. A subsystem naming
    # pins::SERVO instead of 0 is reading downward, not sideways.
    'firmware/lib':          {'types.hxx', 'hal.hxx', 'pins.hxx'},
    'firmware/lib/drivers':  {'../hal.hxx'},
    'firmware/lib/chassis':  {'../hal.hxx', 'cal.hxx', '../pins.hxx'},
    'firmware/app':          {'../lib/bibo.hxx'},
    # Renamed from scratch/ when a sketch became a file rather than a slot. The
    # key is matched by substring against the path, so the stale name matched
    # nothing and took BOTH this check and the libc one below off sketches
    # entirely - silently, for as long as the rename went unnoticed here.
    'firmware/sketches':     {'../lib/bibo.hxx'},
    # A host test of ONE header includes that header, not the umbrella - the
    # umbrella drags in the SDK and these compile with MSVC.
    'firmware/tests':        {'../lib/text.hxx'},
}

# gfx draws INTO a Screen, so it is the one file at lib root that legitimately
# reaches sideways into a driver. Written down rather than special-cased in
# silence.
LAYER_EXTRA = {
    'firmware/lib/gfx.hxx':  {'drivers/display.hxx'},
    'firmware/lib/status.hxx': {'hal.hxx'},
    'firmware/lib/lights.hxx': {'hal.hxx'},
    'firmware/lib/net.hxx': {'hal.hxx'},
    # cue.hxx DECIDES what the car expresses and lights.hxx emits it, so this is the
    # same legitimate sideways reach gfx.hxx makes into a driver: one layer above
    # naming the one below it, written down rather than special-cased in silence.
    'firmware/lib/cue.hxx': {'hal.hxx', 'lights.hxx'},
    'firmware/lib/bibo.hxx': {'hal.hxx', 'text.hxx', 'gfx.hxx', 'status.hxx',
                            'pins.hxx',
                            'drivers/dfplayer.hxx', 'drivers/display.hxx',
                            'drivers/range.hxx', 'drivers/storage.hxx',
                            'chassis/cal.hxx', 'chassis/chassis.hxx',
                            'lights.hxx', 'cue.hxx', 'net.hxx'},
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
        # lwIP joins the list for the same reason the SDK is on it: it is
        # somebody else's stack, and net.h exists precisely to be the one file
        # that reaches into it - the same job hal.h does for the SDK. The rule
        # this pass enforces is about the direction OUR headers point.
        if what.startswith(('pico/', 'hardware/', 'boards/', 'lwip/')):
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
    ('printf',   'serial::printf'),
    # snprintf was missing for as long as this check existed, and app/main.cxx
    # called it twice. The negative lookbehind below is defeated by the leading
    # `s`, so `printf` never matched it - the one libc call the rule was most
    # sure it had covered was the one it could not see.
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
    if '/firmware/app/' not in norm and '/firmware/sketches/' not in norm:
        continue
    code = strip_noise(rd(path))
    for i, line in enumerate(code.split('\n')):
        for name, instead in LIBC_DIRECT:
            # Whole word followed by a paren, so snprintf does not match
            # printf and text::len does not match strlen.
            #
            # The ':' in that lookbehind is what the namespaces cost this rule.
            # serial::printf CONTAINS printf, so without it every corrected
            # call site reported itself as the thing it had just been corrected
            # from - the rule accusing its own fix.
            if re.search(r'(?<![A-Za-z0-9_:])' + name + r'\s*\(', line):
                print('  %-22s %5d  %s( -> use %s('
                      % (os.path.basename(path), i + 1, name, instead))
                libc_bad += 1

if libc_bad == 0:
    print('  ok')
total += libc_bad

print('\n--- signatures over 100 columns ---')
# A RATCHET, not a rule.
#
# docs/conventions.md is honest that these are a design problem and not a
# formatting one: a function taking eleven parameters was hard to read wrapped
# as well, and "a parameter list never wraps" only made that visible. It
# recorded 47 at the time, worst 167, and said the work was not done.
#
# It still is not. What this does instead is stop the number GROWING: the
# budget below is what the tree measured when the check went in, and the audit
# fails if it rises. Shortening a signature lowers the budget with it; adding a
# twelfth parameter to something already over the line does not pass.
#
# 2 columns of the current figure are mine: indenting namespace bodies on
# 2026-08-30 moved every line inside a namespace two to the right, which pushed
# borderline signatures over. That is a real cost of that change and is
# recorded here rather than absorbed quietly.
SIG_BUDGET = 41

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
# "Enum members: SCREAMING_SNAKE_CASE, PREFIXED WITH THE ENUM NAME" -
# MapMode::MAP_MODE_POINTS. docs/conventions.md has said so since 2026-08-25
# and nothing had ever checked it. 224 members already comply; they comply by
# habit, which is the state a rule is in right before it stops being true.
#
# The prefix is what makes an unscoped enum safe to `using`, and what makes a
# grep for MAP_MODE find the whole family.

# Lamp is the one enum that does not comply, and it is WAIVED rather than
# silently skipped, for the reason the C-cast carve-out was: a waiver nobody
# can see is a waiver that grows. Renaming HEAD_L to LAMP_HEAD_L is 64
# references across six files, two of which (pins.hxx, sketches/speaker.cxx)
# are being written right now. It is a rename to do when that lands, not
# during.
ENUM_WAIVED = {'Lamp': 'speaker work in flight - 64 refs across 6 files'}


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
        # Collect the enum body by brace depth.
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
# Allman brace, and a body indented one level inside it.
#
#     namespace ui
#     {
#       Void draw()
#       {
#       }
#     }
#
# Two spaces per namespace level. Most of the firmware sits two namespaces
# deep (bibo::lights), and four would push every real line eight columns right
# before it had said anything.
#
# The tree was split almost exactly in half before this rule: 45 files wrote
# `namespace ui {` and 46 wrote the brace on its own line, and NOTHING
# indented a body. The half that was already Allman only stayed that way
# because the namespace-presence check above happened to require it.
NS_SAME_LINE = re.compile(r'^\s*namespace(\s+[A-Za-z_][\w:]*)?\s*\{')
NS_OPEN_LINE = re.compile(r'^(?P<ind>\s*)namespace(\s+[A-Za-z_][\w:]*)?\s*$')

ns_bad_layout = 0
for path in files:
    code = strip_noise(rd(path))          # braces in prose are not structure
    lines = rd(path).split('\n')
    clean = code.split('\n')
    depth = 0          # brace depth
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
# `#pragma once`, not an #ifndef guard.
#
# Nothing checked this, and six firmware headers were still carrying C-era
# guards named `_H` - hal.hxx saying `#ifndef BIBO_HAL_H` - left behind when
# the C++ migration renamed them .h -> .hxx. A rename changed the extension
# and not the contents, which is exactly the kind of drift a rename pass
# leaves and nobody sees again.
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
        # The Pico SDK, lwIP and the rest keep whatever extension upstream gave
        # them. hal.hxx and net.hxx exist precisely to be the files that reach
        # into somebody else's code, and a rule about OUR extensions that
        # flagged them would be a rule nobody could satisfy.
        if inc in HEADER_EXEMPT or inc.startswith(
                ('imgui', 'sl_lidar', 'stb_',
                 'pico/', 'hardware/', 'boards/', 'lwip/')):
            continue
        # C sources include C headers. shared.h and pico2w.h are .h because they
        # must be, so including them by that name is correct.
        if is_c(path) or inc in ('shared.hxx', 'types.h'):
            continue
        print('  %s:%d  %s' % (os.path.basename(path), i + 1, l.strip()))
        total += 1


# ===========================================================================
#  namespaces
# ===========================================================================
#
# This used to check module PREFIXES, because the library was C and C has no
# namespaces - so every symbol carried its module in its name and a rule could
# only look at spelling. The library is C++ now and the boundary is real: the
# compiler knows what is in namespace gpio, and gpio::write cannot quietly
# become something else's write.
#
# So what is left to check is the thing the compiler cannot: that each module
# HAS its namespace, and that it is the one everybody else expects. A header
# that quietly stops declaring one still compiles - its symbols simply move to
# the global namespace, one file at a time, which is exactly how the prefixes
# decayed before anything checked them.
#
# WHAT IS NOT LISTED, and why:
#
#   hal.hxx   deliberately MANY namespaces - gpio, pwm, spi, i2c, serial, led,
#             radio, adc, watchdog, timing, board - because it is THE BOARD and
#             one namespace for the lot would say nothing. They are checked
#             below as a set.
#   types.hxx the vocabulary itself. Int32 is not in a module, and putting it in
#             one would mean writing the namespace on every declaration in the
#             project.
#   cal.hxx   generated by the hub, and macros besides - the preprocessor has
#             finished before C++ has heard of a namespace.
#   bibo.hxx  the umbrella. Declares nothing.
MODULE_NAMESPACE = {
    'lights.hxx':   'lights',
    'cue.hxx':      'cue',
    'net.hxx':      'net',
    'status.hxx':   'status',
    'gfx.hxx':      'gfx',
    'text.hxx':     'text',
    'chassis.hxx':  'drive',
    'display.hxx':  'tft',
    'range.hxx':    'tof',
    'storage.hxx':  'sd',
}

# hal is the board, and these are the modules in it.
HAL_NAMESPACES = {'gpio', 'timing', 'serial', 'board', 'pwm', 'servo', 'led',
                  'radio', 'adc', 'watchdog', 'spi', 'i2c'}

print('\n--- namespaces ---')
ns_bad = 0
for path in files:
    # By PATH, not just by name. hub/src/lights.hxx is the hub's own model of
    # the same lamps and is not in namespace lights - it reported itself missing
    # a namespace it was never supposed to have.
    if '/firmware/lib' not in path.replace('\\', '/'):
        continue

    base = os.path.basename(path)

    if base == 'hal.hxx':
        # `\s*` in front: a namespace inside another namespace is indented now,
        # so `namespace gpio` sits two columns in. Anchoring at column 0 made
        # every module in hal.hxx report itself missing the moment the bodies
        # were indented.
        have = set(re.findall(r'^\s*namespace (\w+)\s*$', rd(path), re.M))
        for want in sorted(HAL_NAMESPACES - have):
            print('  %-14s declares no namespace %s' % (base, want))
            ns_bad += 1
        continue

    want = MODULE_NAMESPACE.get(base)
    if want is None:
        continue
    if not re.search(r'^\s*namespace %s\s*$' % re.escape(want), rd(path), re.M):
        print('  %-14s declares no namespace %s' % (base, want))
        ns_bad += 1

total += ns_bad

total += struct_bad

print('\n%d file(s): %s' % (
    len(files),
    ', '.join(sorted(set(os.path.relpath(os.path.dirname(f), ROOT).replace('\\', '/')
                         for f in files)))))
print('%d violation(s), %d waived by EXEMPT' % (total, waived))
sys.exit(0 if total == 0 else 1)
