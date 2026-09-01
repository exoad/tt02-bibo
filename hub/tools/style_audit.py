"""Audits hub/ against Jack's C++ Style Guide as recorded in docs/conventions.md.

    python tools/style_audit.py

Exits 0 when clean, 1 otherwise, so it can gate a commit.

Comment- and string-aware: a rule about code must not fire on prose, and this
file is full of prose describing the rules it enforces.
"""
import io, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')


def at(*parts):
    return os.path.join(ROOT, *parts)


# Everything in this repo that is OURS. vendor/ (and hub/vendor's imgui copy)
# is upstream and is not audited. The list is explicit rather than a walk so
# that adding a directory is a decision somebody makes - a walk hides the
# interesting mistake, a directory nobody remembered.
DIRS = [
    at('hub', 'src'),
    at('hub', 'tests'),
    # board_preview holds only a build/ directory now - no sources. Kept so
    # whatever lands there tomorrow is scanned.
    at('hub', 'tests', 'board_preview'),
    at('lidar', 'bridge'),
    at('firmware', 'lib'),
    at('firmware', 'lib', 'drivers'),
    at('firmware', 'lib', 'chassis'),
    at('firmware', 'app'),
    at('firmware', 'sketches'),
    at('firmware', 'tests'),
    # The companion board's program. Stubs today - listed from its first commit
    # so nobody has to remember to add it later.
    at('pilot', 'src'),
    at('pilot', 'tests'),
    at('shared'),
]

# Rules C cannot follow, so they are not applied to it:
#   - named casts. C has no static_cast; `(Int64) x` is the only spelling there
#     is. NOT silently: the carve-out is counted and reported at the end, since
#     firmware/ is expected to become C++ and a waiver nobody can see grows.
#   - static inline. In C that is the idiom for a header definition, not
#     redundancy; C++ gets internal linkage from `static` alone.
#   - .hpp: a header that must compile as C is a .h (docs/conventions.md).
C_ONLY_WAIVES = {'c-style cast', 'static inline'}

# shared.hxx and shared.hxx are WHERE the aliasing happens, so they are exempt
# from the rule that everything else use the aliases - the same carve-out that
# lets firmware/lib/text.hxx name strtol.
VOCAB_FILES  = {'shared.hxx', 'shared.hxx'}
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

# A cast is SYNTAX, not a list of type names: enumerating the types a cast could
# be TO missed `(MINMAXINFO*)lparam` and `(sl_u32)baud` for years. The SHAPE is
# an open paren not following an identifier, a TYPE, a close paren, an operand.
# Types are PascalCase here, so a capitalised name in parens is a type while
# `(width) * 2` is arithmetic; Win32 shouts, the stdlib uses _t, Slamtec sl_.
CAST_TYPE = (r'(?:const\s+)?(?:(?:unsigned|signed)\s+)?'
             r'(?:[A-Z][A-Za-z0-9_]*(?:::[A-Za-z_]\w*)*'
             r'|\w+_t'
             r'|sl_\w+'
             r'|(?:unsigned|signed|int|float|double|char|short|long|bool|void)\b'
             r'(?:\s+(?:int|long|char))?'
             r')\s*\**\s*')

# THE TRAP OF THIS FILE: these rules match the RAW line, not the stripped one.
# strip_noise blanks comment and string-literal CONTENTS - correct for every
# rule about code, fatal for a rule about TEXT. A hardcoded home path lives only
# inside a literal or a comment, so run against the stripped copy it finds 0 of
# its targets, forever, while reporting a clean tree.
RAW_RULES = {'absolute user path', 'namespace trailer comment'}

RULES = [
    # (name, regex, note)

    # A home directory in the tree names the machine it was written on. Spelled
    # without a name in it so it keeps working for the next contributor. The
    # username must START alphanumeric, or `file:///C:/Users/...` in lsp.cxx
    # matches on the ellipsis and reports a comment that names nobody.
    ('absolute user path',
     r'[A-Za-z](?::|%3[Aa])[\\/]{1,4}[Uu]sers[\\/]{1,4}'
     r'[A-Za-z0-9_][A-Za-z0-9_.-]*',
     'derive the path - a home directory in the tree names the machine'),
    # The `>` in the lookbehind keeps `static_cast<Size>(SRC) * 4` out - a named
    # cast whose RESULT is multiplied, not a cast of `(SRC)`. Without it every
    # correctly-written cast followed by a `*` reported itself.
    ('c-style cast',
     r'(?<![A-Za-z0-9_)\]>])\(\s*' + CAST_TYPE + r'\)\s*(?!&&|\|\|)[A-Za-z_(&*]',
     'use a named cast'),

    ('bare builtin type',
     r'(?<![A-Za-z_>:.])(?:unsigned\s+(?:int|char|short|long)|signed\s+char|\bint\b|\bfloat\b|\bdouble\b|\bbool\b|\bchar\b|\bsize_t\b|\bunsigned\b)(?![A-Za-z_0-9])',
     'use the shared.hxx alias'),

    # `} // namespace foo`. Nothing verifies it, it is written once, and it
    # survives a rename - at which point it is confidently wrong.
    ('namespace trailer comment',
     r'^\s*\}\s*//\s*namespace\b',
     'delete it - a closing brace does not need to say what it closes'),

    # `struct Foo f;` - the C89 elaborated type specifier. C++ injects a struct's
    # name as a type name, so the keyword adds nothing. A NAME must follow, which
    # separates a use from a definition or a forward declaration.
    ('elaborated type specifier',
     # The separator must be real - whitespace or a star. Without that,
     # `\w*\s*\w*` splits ONE identifier in two and the forward declaration
     # `struct ID3D11Device;` matched as `Devic` `e`.
     r'(?<![A-Za-z0-9_])struct\s+[A-Za-z_]\w*(?:\s+|\s*\*+\s*)[A-Za-z_]\w*\s*[,;=)]',
     'drop the struct keyword - in C++ the name alone is the type'),

    ('if with space',   r'\bif\s+\(',      'if(cond)'),
    ('for with space',  r'\bfor\s+\(',     'for(...)'),
    ('while with space',r'\bwhile\s+\(',   'while(...)'),
    ('switch with space',r'\bswitch\s+\(', 'switch(...)'),

    # Found `static UINT DpiForWindow(HWND)` in main.cxx, three lines from the
    # Win32 GetDpiForWindow it wraps - which is why it read as fine.
    ('PascalCase function',
     r'^\s*(?:static\s+)?(?:const\s+)?(?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|'
     r'UInt16|UInt32|UInt64|Float32|Float64|Size|Str|Char|Utf8|UINT|LRESULT|HRESULT)'
     r'\s+[A-Z][A-Za-z0-9]*\s*\(',
     'functions are camelCase'),

    # Found `static Void sleep_ms(Int32)` in test_pico_link.cxx, reading as the
    # Pico SDK call it is named after and is not.
    ('snake_case function',
     r'^\s*(?:static\s+)?(?:const\s+)?(?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|'
     r'UInt16|UInt32|UInt64|Float32|Float64|Size|Str|Char|Utf8)'
     r'\s+[a-z][a-z0-9]*_[a-z0-9_]+\s*\(',
     'functions are camelCase'),

    ('k-prefixed constant', r'\bk[A-Z][A-Za-z0-9]*\b', 'SCREAMING_SNAKE_CASE'),
    ('m_ member',           r'\bm_[A-Za-z0-9_]+',      'camelCase, no m_'),
    ('g_ global',           r'\bg_[A-Za-z0-9_]+',      'camelCase, no g_'),
    ('trailing underscore', r'\b[a-z][A-Za-z0-9]*_\b(?!\s*\()', 'camelCase, no trailing _'),

    # Only the TYPES are aliased in shared/shared.hxx, so they are named
    # explicitly rather than banning the namespace: std::move, std::sort and
    # duration_cast are functions and keep their spelling. The \b after
    # `duration` is what separates it from `duration_cast`. chrono and the file
    # streams joined late and were the biggest hole here - 31 raw std::chrono
    # uses, invisible because there was no alias to point at.
    ('unaliased std type',
     r'\bstd::(?:vector|deque|array|map|set|unordered_map|unordered_set|pair|'
     r'tuple|string|string_view|optional|variant|function|unique_ptr|'
     r'shared_ptr|weak_ptr|mutex|recursive_mutex|lock_guard|unique_lock|'
     r'thread|atomic|condition_variable|ifstream|ofstream|fstream'
     r'|chrono::(?:steady_clock|system_clock|high_resolution_clock|time_point'
     r'|milliseconds|microseconds|nanoseconds|seconds|duration)'
     r'|this_thread::sleep_for)\b',
     'use the shared.hxx alias (Vec, Str, Clock, TimePoint, sleepMs, ...)'),

    # The fixed-width integers, bare or std:: qualified. `bare builtin type`
    # above never caught uint32_t - a hole the size of the whole stdint family.
    # Zero real uses when it went in; the alternative is finding out at 31.
    ('unaliased fixed-width integer',
     r'(?<![A-Za-z0-9_:.])(?:std::)?(?:u?int(?:8|16|32|64)_t|uintptr_t'
     r'|ptrdiff_t)(?![A-Za-z0-9_])',
     'use the vocabulary alias - Int32, UInt8, UPtr, ISize'),

    # Allman, everywhere. Aggregate rows in a table are NOT this - the pattern
    # requires a `)` or a control keyword before the brace, which separates a
    # body from a row. A parameter list that does not close on its own line:
    # DEFINITIONS and DECLARATIONS only, since a call's arguments are
    # expressions and may wrap. Anchored on a leading return type, since a call
    # starts with an identifier.
    ('wrapped parameter list',
     r'^\s*(?:\[\[nodiscard\]\]\s*)?(?:static\s+|inline\s+|constexpr\s+|const\s+)*'
     r'(?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|UInt16|UInt32|UInt64|Float32|'
     r'Float64|Size|Str|Char|Utf8|CharSeq|Pin)[\w:<>,\s\*&]*?\s[\w:~]+\s*\([^)]*$',
     'put the whole parameter list on one line'),

    # `static` already gives internal linkage and a static function is emitted
    # only where it is used, so `inline` adds nothing. C is waived above - there
    # `static inline` is the header-definition idiom and removing it is wrong.
    ('static inline',
     r'\bstatic\s+inline\b',
     'in C++, static already implies it - drop the inline'),

    ('one-lined body',
     r'(?:\)|\b(?:else|do|try)\b)\s*(?:const\s*)?(?:noexcept\s*)?\{[^{}]*[^{}\s][^{}]*\}',
     'expand the braces onto their own lines'),

    # The brace on the HEAD's line, body closing far below. The rule above needs
    # both braces on one physical line and so cannot see this - a blind spot
    # that hid 40 of them in the Slamtec boundary files, which were carrying the
    # SDK's brace style. A lambda is the written-down exception, see EXEMPT.
    ('cuddled brace',
     r'(?:\)|\b(?:else|do|try)\b)\s*(?:const\s*)?(?:noexcept\s*)?\{\s*$',
     'Allman - put the brace on its own line'),

    # `if(x) return;` - a body with no braces, sharing its head's line. Every
    # other brace rule looks for a brace and this is the shape that has none:
    # 250 across the tree. A body on the NEXT line is NOT this.
    #
    # Three traps, all fixed here. A `[^;]*` condition can never match a for
    # head, which has two semicolons by definition. A body required to start
    # with a LETTER lets `while(n) --n;` walk past. And the head's parens are
    # matched to THREE levels because a regex cannot balance them - a `[^)]*`
    # head stops inside static_cast and reported six correct app_ui.cxx loops.
    ('braceless one-lined body',
     r'^\s*(?:if|while|for)\s*'
     r'\((?:[^()]|\((?:[^()]|\([^()]*\))*\))*\)'
     r'\s*[^\s{/;][^;]*;',
     'give the body its own braces on their own lines'),

    # `Char* p`, not `Char *p` - a pointer to Char is a type in its own right
    # here. The tree was already 604:0 and 747:0 this way when the rule was
    # written; it is here so it stays that way, not to fix a mess.
    ('pointer bound to the name',
     r'\b[A-Z][A-Za-z0-9_]*\s+\*[a-z][A-Za-z0-9_]*',
     'bind the * to the type: Type* name'),

    ('reference bound to the name',
     r'\b[A-Z][A-Za-z0-9_]*\s+&[a-z][A-Za-z0-9_]*',
     'bind the & to the type: Type& name'),
]

# Lines that are legitimately exempt, with the reason.
EXEMPT = [
    # main.cxx's user32 typedefs are Win32 signatures; `int` there is the OS ABI.
    (r'typedef .*WINAPI', 'Win32 ABI signature'),
    # `int main(` STAYS WAIVED, measured rather than assumed. Converting all 24
    # entry points to `Int32 main(...)` passed the host suites and went clean
    # (MSVC's int32_t IS int), then failed on both boards with
    # "'::main' must return 'int'" - on arm-none-eabi int32_t is `long int`,
    # same 32 bits, different type. The LANGUAGE fixes the type here, as for
    # WinMain; writing Int32 is a portability bug MSVC cannot see.
    (r'int APIENTRY|WinMain|int main\(', 'the platform entry point signature'),
    (r'IMGUI_IMPL_API|ImGui_ImplWin32_WndProcHandler', 'third-party signature'),
    (r'static_cast<int>|static_cast<float>|static_cast<unsigned', 'named cast to an ABI type'),
    (r'#\s*(define|include|if|ifdef|ifndef|endif|else|elif|pragma)', 'preprocessor'),

    # `sizeof(Float32)` is not a cast, and the pattern cannot tell `(T)x` from
    # `sizeof(T) * x` without a real parser.
    (r'\bsizeof\s*\(', 'sizeof, not a cast'),

    # shared.hxx is where the aliases are DEFINED; `using Float32 = float;` has
    # to name the builtin.
    (r'\busing\s+\w+\s*=\s*(float|double|bool|char|int|unsigned|std::)', 'the alias definition itself'),

    # shared.h is the same file for C: `typedef char Utf8;` is the definition.
    (r'^\s*typedef\s+\w+\s+\w+\s*;', 'the alias definition itself'),

    # A single-expression lambda body reads better on one line - the rule
    # targets function and control-flow bodies, not these.
    (r'\[[^\]]*\]\s*\([^)]*\)\s*(?:->\s*[A-Za-z_:<>]+\s*)?\{', 'a lambda, not a function body'),

    # A lambda's brace cuddles by convention: `const auto flush = [&]() {` is an
    # expression passed to something, not a function body standing on its own.
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
    .c/.h are C, .cxx/.hxx are C++. The C++ half moved off .cpp/.hpp because .h
    was doing double duty as "a C header" and "a header nobody thought about",
    and an unowned .h is assumed C++ by most editors.
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
            r = raw[i] if i < len(raw) else ''
            # Both, not just the stripped one: a comment-only line is blank
            # AFTER stripping, so testing `l` alone hides every raw-rule hit
            # that lives in a comment.
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

# The structural pass: not what the code LOOKS like, but where it may reach.
# Formatting keeps a file readable; this keeps the ARCHITECTURE true, and the
# architecture decays one reasonable-looking include at a time.

# Which layer a firmware file belongs to, and what that layer may include.
# Strictly downward: hal knows nothing; drivers and chassis know hal; an app
# knows only the umbrella.
#
# The SPELLING is part of the rule: "../hal.h" rather than "hal.h" from
# lib/drivers/. The bare form compiles only because -Ifirmware/lib is set, and
# an editor without the project loaded underlines every include in the library.
LAYERS = {
    # hal.h is the floor everything stands on, so lib root may name it. pins.hxx
    # sits beside shared.hxx: it declares facts and includes nothing but types,
    # so naming pins::SERVO instead of 0 is reading downward, not sideways.
    'firmware/lib':          {'shared.hxx', 'hal.hxx', 'pins.hxx'},
    # ../shared.hxx so a driver's PROTOCOL half can reach the vocabulary without
    # the SDK - that is what makes dfplayer_proto.hxx testable off the bench.
    # ../pins.hxx: a driver reads the pin map it was WIRED with rather than
    # holding pad numbers, a downward read and not a reach at another driver.
    'firmware/lib/drivers':  {'../hal.hxx', '../shared.hxx', '../pins.hxx',
                              'dfplayer_proto.hxx'},
    'firmware/lib/chassis':  {'../hal.hxx', 'cal.hxx', '../pins.hxx'},
    'firmware/app':          {'../lib/bibo.hxx'},
    # Renamed from scratch/. The key is matched by substring against the path,
    # so the stale name matched nothing and silently took BOTH this check and
    # the libc one below off sketches entirely.
    'firmware/sketches':     {'../lib/bibo.hxx'},
    # A host test of ONE header includes that header, not the umbrella - the
    # umbrella drags in the SDK and these compile with MSVC.
    'firmware/tests':        {'../lib/text.hxx',
                              '../lib/pins.hxx',
                              '../lib/sfx.hxx',
                              '../lib/control.hxx',
                              '../lib/geom.hxx',
                              '../lib/kinematics.hxx',
                              '../lib/pursuit.hxx',
                              '../lib/chassis/odom.hxx',
                              # The safety property, tested on the host through
                              # tests/fakes/hal.hxx - the first test of a module
                              # that includes hal.hxx at all.
                              '../lib/chassis/chassis.hxx',
                              '../lib/drivers/dfplayer_proto.hxx'},
}

# gfx draws INTO a Screen, so it is the one file at lib root that legitimately
# reaches sideways into a driver. Written down rather than special-cased in
# silence - which is what every entry below is.
LAYER_EXTRA = {
    'firmware/lib/gfx.hxx':  {'drivers/display.hxx'},
    # pins.hxx formats its own conflict message, so it names text.hxx - a leaf,
    # so this is a sideways reach that cannot cycle.
    'firmware/lib/pins.hxx': {'shared.hxx', 'text.hxx'},
    # hal.hxx names the host-test fake behind #ifdef BIBO_FAKE_HAL, off in every
    # image this project flashes - the one place the library reaches into tests/.
    'firmware/lib/hal.hxx': {'shared.hxx', '../tests/fakes/hal.hxx'},
    # boot.hxx is serial + the pin map + a visible refusal: the one lib-root
    # file that legitimately needs pins.
    'firmware/lib/boot.hxx': {'hal.hxx', 'pins.hxx'},
    # sfx.hxx is names and numbers - what the clips on the card MEAN. No SDK, so
    # its table can be tested without a board.
    'firmware/lib/sfx.hxx': {'shared.hxx'},
    # control.hxx is arithmetic - PID and feedforward - and odom.hxx turns ticks
    # into meters. Neither touches hardware, which is what lets both be tested
    # on the host against invented inputs.
    'firmware/lib/control.hxx': {'shared.hxx'},
    # The autonomy maths, pure and portable, a strict stack each naming only the
    # one below: geom, kinematics, pursuit, then plan.
    'firmware/lib/geom.hxx': {'shared.hxx'},
    'firmware/lib/kinematics.hxx': {'geom.hxx'},
    'firmware/lib/pursuit.hxx': {'geom.hxx', 'kinematics.hxx'},
    'firmware/lib/plan.hxx': {'geom.hxx', 'pursuit.hxx'},
    'firmware/lib/chassis/odom.hxx': {'../shared.hxx'},
    # sound.hxx owns the speaker, so it reaches down to the driver and sideways
    # to the clip table and the pin map.
    'firmware/lib/sound.hxx': {'hal.hxx', 'pins.hxx', 'sfx.hxx',
                               'drivers/dfplayer.hxx'},
    'firmware/lib/status.hxx': {'hal.hxx'},
    'firmware/lib/lights.hxx': {'hal.hxx'},
    'firmware/lib/net.hxx': {'hal.hxx'},
    # cue.hxx DECIDES what the car expresses and lights.hxx emits it - one layer
    # above naming the one below.
    'firmware/lib/cue.hxx': {'hal.hxx', 'lights.hxx'},
    'firmware/lib/bibo.hxx': {'hal.hxx', 'text.hxx', 'gfx.hxx', 'status.hxx',
                            'pins.hxx', 'sfx.hxx', 'sound.hxx', 'boot.hxx',
                            'control.hxx', 'chassis/odom.hxx',
                            'geom.hxx', 'kinematics.hxx', 'pursuit.hxx',
                            'plan.hxx',
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
        # The Pico SDK and lwIP are not ours and are not layers. hal.h and net.h
        # exist precisely to be the files that reach into somebody else's code.
        # This pass is about the direction OUR headers point.
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

# resource.h is compiled by rc.exe, not a C or C++ compiler. Renaming it would
# break the resource build to satisfy a rule about C++ headers.
HEADER_EXEMPT = {'resource.h'}

# The C carve-out, counted. C has no static_cast, but firmware/ is expected to
# move to C++ and skipping the rule silently would leave the size of that move
# unknown. This prints the bill. Not a violation; does not fail the audit.
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
    # "none" reads as "no casts left to fix"; what it means now is "there are no
    # C files to look in". Say which it is - a check that cannot fire should
    # announce that, not report success.
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

# Application code uses the LIBRARY, not libc. firmware/lib wraps the C standard
# library so the project has one vocabulary. The point is not speed, it is that
# the seam is complete: a console calling printf() directly was sixty-two call
# sites to find the day the transport is not stdio. Only app/ and sketches/ are
# checked - lib/ is WHERE the wrapping happens.
print('\n--- application code reaching past the library ---')

LIBC_DIRECT = [
    ('printf',   'serial::printf'),
    # snprintf was missing for as long as this check existed: the lookbehind
    # below is defeated by the leading `s`, so `printf` never matched it - the
    # one libc call the rule was surest of was the one it could not see.
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
            # Whole word followed by a paren, so snprintf does not match printf.
            # The ':' in the lookbehind is what the namespaces cost this rule:
            # serial::printf CONTAINS printf, so without it every corrected call
            # site reported itself - the rule accusing its own fix. `.` and `>`
            # are there for gfx::Canvas's printf METHOD.
            if re.search(r'(?<![A-Za-z0-9_:.>])' + name + r'\s*\(', line):
                print('  %-22s %5d  %s( -> use %s('
                      % (os.path.basename(path), i + 1, name, instead))
                libc_bad += 1

if libc_bad == 0:
    print('  ok')
total += libc_bad

print('\n--- raw C arrays ---')
# `T name[N]` where T is OURS. Array<T, N> carries its length; a raw array
# decays to a pointer at the first call and the length becomes something the
# caller has to know by other means.
#
# THE CARVE-OUT IS THE ELEMENT TYPE. `BYTE data[512]` handed to RegEnumValueA is
# somebody else's buffer in somebody else's shape, and we do not dress a
# third-party API up to look like ours. firmware/ is exempt entirely -
# freestanding, no <array>. The first thing this found was three `Char cmd[48]`
# buffers added AFTER the conversion pass: a sweep fixes a tree once.
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
# A RATCHET, not a rule. docs/conventions.md is honest that long signatures are
# a design problem and not a formatting one, and that the work is not done. What
# this does instead is stop the number GROWING: the budget below is what the
# tree measured when the check went in, and the audit fails if it rises.
# Shortening a signature lowers the budget with it.
#
# Where the figure came from: 2 columns are from indenting namespace bodies on
# 2026-08-30, and 30 -> 51 on 2026-08-31 was a const-correctness pass across
# firmware/lib - the same parameters spelled longer, 21 crossing the line.
SIG_BUDGET = 51

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
# MapMode::MAP_MODE_POINTS. Nothing had ever checked it; 224 members complied by
# habit, which is the state a rule is in right before it stops being true. The
# prefix is what makes an unscoped enum safe to `using` and a grep for MAP_MODE
# find the whole family.

# Empty, and that is the point of having printed it. Lamp was the one enum whose
# members did not carry their enum's name, waived on 2026-08-31 while the
# speaker work was mid-flight across the same files. It landed, the rename
# happened, and the waiver came out with it.
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
# Allman brace, and a body indented one level inside it. Two spaces per
# namespace level - most of the firmware sits two deep (bibo::lights), and four
# would push every real line eight columns right before it said anything.
#
# The tree was split 45/46 on the brace before this rule, and NOTHING indented
# a body.
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

        # And the BODY, which the check above cannot see: every line inside N
        # namespaces starts at column 2N or deeper. The reindent that introduced
        # the rule got 30 lines wrong unnoticed - it decided a line closed its
        # namespace by counting `}`, true for `} // namespace ui` and false for
        # `struct WorldPt { Float32 x, y; };`, left sitting at column 0.
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
# `#pragma once`, not an #ifndef guard. Six firmware headers still carried C-era
# guards named `_H` - hal.hxx saying `#ifndef BIBO_HAL_H` - left behind when the
# C++ migration renamed .h -> .hxx and changed nothing else. That is the kind of
# drift a rename pass leaves and nobody sees again.
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

    # A C header under firmware/ is correctly a .h; anywhere else it is a C++
    # header wearing the wrong extension.
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
        # The Pico SDK, lwIP and the rest keep whatever extension upstream gave
        # them; hal.hxx and net.hxx exist to be the files that reach into them.
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


# Namespaces. This used to check module PREFIXES, because the library was C and
# every symbol carried its module in its name. The library is C++ now and the
# compiler enforces the boundary, so what is left to check is what it cannot:
# that each module HAS its namespace and that it is the one everybody expects. A
# header that quietly stops declaring one still compiles - its symbols move to
# the global namespace, one file at a time, which is how the prefixes decayed.
#
# NOT LISTED: hal.hxx (deliberately many namespaces - it is THE BOARD - checked
# as a set below), shared.hxx (the vocabulary itself, not a module), cal.hxx
# (hub-generated, and macros besides), bibo.hxx (the umbrella, declares nothing).
MODULE_NAMESPACE = {
    'boot.hxx':     'boot',
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
    # By PATH, not just by name: hub/src/lights.hxx is the hub's own model of
    # the same lamps and reported itself missing a namespace it never had.
    if '/firmware/lib' not in path.replace('\\', '/'):
        continue

    base = os.path.basename(path)

    if base == 'hal.hxx':
        # `\s*` in front, and `(?:\w+::)*` so `bibo::gpio` counts as declaring
        # `gpio`: namespace bodies are indented now and the names concatenated
        # on 2026-08-31. Anchoring at column 0 on a bare inner name made every
        # module in hal.hxx report itself missing.
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

# Stray files in the repository root. Not a style rule - a DAMAGE rule, and the
# only one here about the shell rather than about C++.
#
# On 2026-08-31 a 75 GB file called `headOn` was found in the root. Nothing in
# this repository wrote it: it came from a shell command that printed source
# text WITHOUT QUOTING it, and the text contained an arrow -
#
#     grep -rn in->headOn .      the shell reads as
#     grep -rn in- .  > headOn   pattern `in-`, output redirected
#
# `>` truncates, so one of those costs nothing; `>>` appends, and a C++ line
# full of shifts is full of `>>` - `(a >> 11) & 0x1F` appends to a file called
# `11)`. Fifteen turned up in one day; fourteen were zero bytes, and the
# fifteenth landed on a name that was also being read and grew.
#
# So the check is blunt: NOTHING untracked belongs in the root. Catching it at
# zero bytes is the point - the 75 GB one was invisible for hours because
# nothing looked, and `git status` buries it in the same `??` list as a new
# directory somebody meant to add.
print('\n--- stray files in the repository root ---')
stray_bad = 0
try:
    import subprocess
    out = subprocess.check_output(
        ['git', 'ls-files', '--others', '--exclude-standard'],
        cwd=ROOT, stderr=subprocess.DEVNULL).decode('utf-8', 'replace')
except Exception as e:
    # A missing git is not a clean tree. Say which one this is.
    print('  SKIPPED - could not ask git (%s)' % e.__class__.__name__)
    out = None

if out is not None:
    for name in sorted(l.strip() for l in out.split('\n') if l.strip()):
        if '/' in name:
            continue                      # not in the root
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
sys.exit(0 if total == 0 else 1)
