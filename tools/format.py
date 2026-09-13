"""Layout rules for .cxx and .hxx files, checked or fixed.

    python tools/format.py [--apply] [--markdown OUT] [path ...]

With no paths every tracked and untracked source outside vendor/ is checked; a
path limits the report and --apply to that file, or to the sources under that
directory. --apply rewrites, --markdown writes the report as tables. Exit 1 when
anything is found, or with --apply when a file was refused; exit 2 on bad usage.

CALL WRAPPING. A call's arguments stay on one line when the call fits in LIMIT
columns and has at most MAX_ARGS arguments. Otherwise the `(` ends its line,
every argument gets its own line one indent in, and the `)` returns to the
column the call's own line starts at:

      bibo::serial::printf(
          "INFO slew throttle %d us/tick = %d us/s, idle to full %d ms\\n",
          d.throttleSlewUs,
          escPerSec,
          fullMs
      );

Not paren-alignment: that pushes the longest calls furthest right, and every
argument moves when the function is renamed.

PADDED `=`. One space before `=`, never a run of them to line up a column: a
padded column re-pads its whole block on every rename.

BLANK LINES.
  B1  no two blank lines in a row;
  B2  none directly after a line whose code ends with `{`;
  B3  none directly before a line whose code starts with `}`;
  B4  none between a comment-only line and the next non-blank line.
A line's code is the line without its comments, so `{  // why` opens a block
and `x = 1;  // why` is not a comment line. A blank line inside a block
comment, a raw string or a continued string literal, or right after a line
ending in a backslash, is never removed.

SAFETY. Every rule only moves whitespace, so --apply compares a comment- and
literal-aware token stream before and after and refuses to write a file where
they differ. A call with an argument that spans lines, or with a nested call
that does, is left for a person: re-indenting it would strand the inner lines.
"""
import io
import os
import re
import subprocess
import sys

LIMIT = 100
INDENT = 4
MAX_ARGS = 6
EXTS = ('.cxx', '.hxx')
RAW_PREFIXES = ('R', 'u8R', 'uR', 'UR', 'LR')

BLANK_RULES = {
    'B1': 'two blank lines in a row',
    'B2': 'blank line after a line ending in {',
    'B3': 'blank line before a line starting with }',
    'B4': 'blank line after a comment',
}

# Parenthesised, but not argument lists.
KEYWORDS = {'if', 'while', 'for', 'switch', 'return', 'catch', 'sizeof',
            'static_cast', 'reinterpret_cast', 'const_cast', 'dynamic_cast',
            'defined', 'decltype', 'noexcept', 'alignof', 'and', 'or', 'not'}

# A declaration head. Parameter lists belong to style_audit.py's 'wrapped
# parameter list' rule, and two rules must not disagree about one line.
DECL = re.compile(
    r'^\s*(?:\[\[nodiscard\]\]\s*)?(?:static\s+|inline\s+|constexpr\s+|const\s+|virtual\s+)*'
    r'(?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|UInt16|UInt32|UInt64|Float32|'
    r'Float64|Size|Str|Char|Utf8|CharSeq|Pin|auto|void|bool|int)\b')

# Words that can stand right before a call without making it a declaration:
# `return foo(` is a call, `Commands foo(` is not.
STATEMENT_WORDS = {'return', 'co_return', 'co_yield', 'co_await', 'new', 'delete',
                   'throw', 'case', 'goto', 'else', 'do', 'operator'}


def is_decl(code, before, line, name):
    """Is the parenthesis after `before` opening a parameter list, not a call?

    DECL knows only the shared.hxx aliases, so a declaration returning a project
    type (`Commands commandsFor(...)`) is recognised by the type before its
    name. A call is preceded by an operator, a statement word, or nothing.
    """
    if DECL.match(line):
        return True
    j = before
    while j >= 0 and code[j] in ' \t':
        j -= 1
    if j < 0 or code[j] == '\n':
        # A constructor is named like a type (`Wide(`, `Wide::Wide(`); a call
        # statement is a camelCase function and a macro is ALL CAPS.
        seg = name.split(':')[-1]
        return seg[:1].isupper() and not seg.isupper()
    c = code[j]
    if c in '>&*~':
        return True                                  # Vec<X>& f(  Str* f(  ~T(
    if c == ']' and j > 0 and code[j - 1] == ']':
        return True                                  # [[nodiscard]] f(
    if c.isalnum() or c == '_':
        e = j
        while j >= 0 and (code[j].isalnum() or code[j] == '_'):
            j -= 1
        return code[j + 1:e + 1] not in STATEMENT_WORDS
    return False


_noise = [None, None]


def blank_noise(text):
    """Comment and string bodies become spaces, offsets preserved.

    A `(` or `,` inside a literal or a comment is not code. The last result is
    cached: every call in a file asks again about the same text.
    """
    if _noise[0] != text:
        _noise[0], _noise[1] = text, _blank_noise(text)
    return _noise[1]


def _blank_noise(text):
    out = list(text)
    i, n, st = 0, len(text), 'code'
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ''
        if st == 'code':
            if c == '/' and nxt == '/':
                st = 'line'
                out[i] = out[i + 1] = ' '
                i += 2
                continue
            if c == '/' and nxt == '*':
                st = 'block'
                out[i] = out[i + 1] = ' '
                i += 2
                continue
            if c == '"':
                st = 'str'
            elif c == "'":
                st = 'chr'
            i += 1
            continue
        if st == 'line':
            if c == '\n':
                st = 'code'
            else:
                out[i] = ' '
            i += 1
            continue
        if st == 'block':
            if c == '*' and nxt == '/':
                st = 'code'
                out[i] = out[i + 1] = ' '
                i += 2
                continue
            if c != '\n':
                out[i] = ' '
            i += 1
            continue
        if c == '\\':
            out[i] = ' '
            if i + 1 < n and text[i + 1] != '\n':
                out[i + 1] = ' '
            i += 2
            continue
        if (st == 'str' and c == '"') or (st == 'chr' and c == "'"):
            st = 'code'
        elif c != '\n':
            out[i] = ' '
        i += 1
    return ''.join(out)


def collapse(text):
    """Whitespace runs to one space, outside string and char literals only.

    Collapsing inside a literal changes printed text that tests assert on
    exactly: "  FAIL  %s" must keep both of its double spaces.
    """
    out = []
    i = 0
    n = len(text)
    st = 'code'
    pending = False
    while i < n:
        c = text[i]
        if st == 'code':
            if c == '"' or c == "'":
                if pending:
                    out.append(' ')
                    pending = False
                st = 'str' if c == '"' else 'chr'
                out.append(c)
                i += 1
                continue
            if c.isspace():
                pending = bool(out)
                i += 1
                continue
            if pending:
                out.append(' ')
                pending = False
            out.append(c)
            i += 1
            continue
        out.append(c)
        if c == '\\':
            if i + 1 < n:
                out.append(text[i + 1])
            i += 2
            continue
        if (st == 'str' and c == '"') or (st == 'chr' and c == "'"):
            st = 'code'
        i += 1
    return ''.join(out)


def raw_end(text, quote):
    """End offset of the raw string whose `"` is at `quote`, or None if it is not one."""
    j = quote
    while j > 0 and (text[j - 1].isalnum() or text[j - 1] == '_'):
        j -= 1
    if text[j:quote] not in RAW_PREFIXES:
        return None
    k = text.find('(', quote + 1)
    delim = text[quote + 1:k]
    if k < 0 or len(delim) > 16 or any(ch in delim for ch in ' \t\n\\)"'):
        return None
    end = text.find(')' + delim + '"', k + 1)
    return len(text) if end < 0 else end + len(delim) + 2


def tokens(text):
    """A normal form that ignores layout and keeps every literal byte for byte.

    It shares no code with the lexer the blank-line rules use, so a mistake in
    one is caught by the other.
    """
    out = []
    i = 0
    n = len(text)
    st = 'code'
    while i < n:
        c = text[i]
        nx = text[i + 1] if i + 1 < n else ''
        if st == 'code':
            if c == '/' and nx == '/':
                st = 'line'
                i += 2
                continue
            if c == '/' and nx == '*':
                st = 'block'
                i += 2
                continue
            if c == '"':
                end = raw_end(text, i)
                if end is not None:
                    out.append(text[i:end])
                    i = end
                    continue
            if c == "'":
                # A digit separator (1'000) is part of a number, not a char literal.
                j = i
                while j > 0 and (text[j - 1].isalnum() or text[j - 1] in "_.'"):
                    j -= 1
                if j < i and text[j].isdigit():
                    out.append(c)
                    i += 1
                    continue
            if c == '"' or c == "'":
                st = 'str' if c == '"' else 'chr'
                out.append(c)
                i += 1
                continue
            if not c.isspace():
                out.append(c)
            i += 1
            continue
        if st == 'line':
            if c == '\n':
                # A backslash carries the comment onto the next line.
                j = i - 1
                while j >= 0 and text[j] in ' \t':
                    j -= 1
                if j < 0 or text[j] != '\\':
                    st = 'code'
            i += 1
            continue
        if st == 'block':
            if c == '*' and nx == '/':
                st = 'code'
                i += 2
                continue
            i += 1
            continue
        out.append(c)
        if c == '\\':
            if i + 1 < n:
                out.append(text[i + 1])
            i += 2
            continue
        if (st == 'str' and c == '"') or (st == 'chr' and c == "'"):
            st = 'code'
        i += 1
    return ''.join(out)


def scan(text):
    """Every call in `text`, as (open, close, argStarts, line, indent)."""
    code = blank_noise(text)
    lines = text.split('\n')
    starts, at = [], 0
    for l in lines:
        starts.append(at)
        at += len(l) + 1

    def line_of(off):
        lo, hi = 0, len(starts) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if starts[mid] <= off:
                lo = mid
            else:
                hi = mid - 1
        return lo

    found = []
    i, n = 0, len(code)
    while i < n:
        if code[i] != '(':
            i += 1
            continue
        j = i - 1
        while j >= 0 and code[j] in ' \t':
            j -= 1
        e = j
        while j >= 0 and (code[j].isalnum() or code[j] in '_:'):
            j -= 1
        name = code[j + 1:e + 1]
        if not name or name.split(':')[-1] in KEYWORDS or name[0].isdigit():
            i += 1
            continue
        ln = line_of(i)
        if is_decl(code, j, lines[ln], name) or lines[ln].lstrip().startswith('#'):
            i += 1
            continue
        depth, k = 0, i
        seps = [i]
        while k < n:
            ch = code[k]
            if ch in '([{':
                depth += 1
            elif ch in ')]}':
                depth -= 1
                if depth == 0 and ch == ')':
                    break
            elif ch == ',' and depth == 1:
                seps.append(k)
            k += 1
        if k >= n:
            i += 1
            continue
        found.append((i, k, seps, ln,
                      len(lines[ln]) - len(lines[ln].lstrip())))
        i += 1
    return found, line_of, starts


def shape(text, call, line_of, starts):
    """What this call should look like, or None when it is already right.

    Returns (kind, replacement, lineNo, cols, args).
    """
    op, cl, seps, ln, ind = call
    lines = text.split('\n')
    multi = line_of(cl) != ln
    args = len(seps)
    head = lines[ln][:op - starts[ln]]
    body = collapse(text[op + 1:cl]).strip()
    flat = head + '(' + body + ')'
    cols = len(flat) + 1                 # the ; that usually follows
    wants = args > MAX_ARGS or cols > LIMIT
    if not multi and not wants:
        return None
    if multi and wants:
        pad = ' ' * (ind + INDENT)
        want_lines = []
        for a in range(len(seps)):
            s = seps[a] + 1
            e2 = seps[a + 1] if a + 1 < len(seps) else cl
            want_lines.append(collapse(text[s:e2]).strip())
        if any(p == '' for p in want_lines):
            return None
        new = head + '(\n' + ',\n'.join(pad + p for p in want_lines) \
              + '\n' + ' ' * ind + ')'
        old = text[starts[ln]:cl + 1]
        if old == new:
            return None
        return ('reshape', new, ln + 1, cols, args)
    if multi and not wants:
        return ('unwrap', flat, ln + 1, cols, args)
    pad = ' ' * (ind + INDENT)
    parts = []
    for a in range(len(seps)):
        s = seps[a] + 1
        e2 = seps[a + 1] if a + 1 < len(seps) else cl
        parts.append(collapse(text[s:e2]).strip())
    if any(p == '' for p in parts):
        return None
    new = head + '(\n' + ',\n'.join(pad + p for p in parts) \
          + '\n' + ' ' * ind + ')'
    return ('reshape', new, ln + 1, cols, args)


def nested_multiline(call, calls, line_of):
    op, cl = call[0], call[1]
    for (o2, c2, _s, _l, _i) in calls:
        if o2 > op and c2 < cl and line_of(c2) != line_of(o2):
            return True
    return False


def multiline_argument(text, call, line_of):
    """Does an argument already span lines, or a // comment sit inside the call?

    Either way the call is left alone. Collapsing an argument list lets a line
    comment swallow the rest of the call, and joins a run of adjacent string
    literals into one very long line.
    """
    op, cl, seps, _ln, _ind = call
    code = blank_noise(text)
    for a in range(len(seps)):
        s = seps[a] + 1
        e = seps[a + 1] if a + 1 < len(seps) else cl
        while s < e and text[s] in ' \t\r\n':
            s += 1
        while e > s and text[e - 1] in ' \t\r\n':
            e -= 1
        if s >= e:
            continue
        if line_of(s) != line_of(e - 1):
            return True
    # A `//` that blank_noise blanked is a comment, not two characters of a string.
    for i in range(op, min(cl, len(code) - 1)):
        if text[i] == '/' and text[i + 1] == '/' \
           and code[i] == ' ' and code[i + 1] == ' ':
            return True
    return False


# Only the run of spaces before `=` counts; `==`, `<=`, `+=` and the like are
# operators, not padding.
EQ_PAD = re.compile(r'^(.*?[^\s=<>!+\-*/%&|^~])(\s{2,})=(?!=)')


def unpad_equals(text):
    """Collapse alignment padding before `=`. Returns (text, count)."""
    code = blank_noise(text)
    lines = text.split('\n')
    clean = code.split('\n')
    hits = 0
    for i, line in enumerate(lines):
        m = EQ_PAD.match(line)
        if not m:
            continue
        # The `=` must be code, not inside a string or comment.
        eq = len(m.group(1)) + len(m.group(2))
        if eq >= len(clean[i]) or clean[i][eq] != '=':
            continue
        # A macro continuation line keeps its padding; it is harmless there.
        if line.rstrip().endswith('\\'):
            continue
        lines[i] = m.group(1) + ' =' + line[eq + 1:]
        hits += 1
    return '\n'.join(lines), hits


CODE, COMMENT, LITERAL = ord('c'), ord('m'), ord('l')


def spliced(text, i):
    """Does a backslash end the line whose newline is at `i`?"""
    j = i - 1
    while j >= 0 and text[j] in ' \t':
        j -= 1
    return j >= 0 and text[j] == '\\'


def char_classes(text):
    """CODE, COMMENT or LITERAL for every character of `text`.

    A newline takes the class of what it lies inside, so the newline before a
    line says whether that line starts in code. A string or char literal left
    open at a newline ends there, so a stray apostrophe in `#if 0` text cannot
    swallow the rest of the file.
    """
    n = len(text)
    cls = bytearray([CODE]) * n
    i = 0
    st = 'code'
    while i < n:
        c = text[i]
        nx = text[i + 1] if i + 1 < n else ''
        if st == 'code':
            if c == '/' and nx in '/*' and nx:
                st = 'line' if nx == '/' else 'block'
                cls[i] = cls[i + 1] = COMMENT
                i += 2
            elif c.isalpha() or c == '_':
                j = i + 1
                while j < n and (text[j].isalnum() or text[j] == '_'):
                    j += 1
                if j < n and text[j] == '"' and text[i:j] in RAW_PREFIXES:
                    k = text.find('(', j + 1)
                    delim = text[j + 1:k]
                    if k >= 0 and len(delim) <= 16 \
                       and not any(ch in delim for ch in ' \t\n\\)"'):
                        end = text.find(')' + delim + '"', k + 1)
                        end = n if end < 0 else end + len(delim) + 2
                        cls[j:end] = bytearray([LITERAL]) * (end - j)
                        j = end
                i = j
            elif c.isdigit() or (c == '.' and nx.isdigit()):
                # A pp-number, so the ' in 1'000 does not open a char literal.
                j = i + 1
                while j < n:
                    d = text[j]
                    if d.isalnum() or d in '_.':
                        j += 1
                    elif d in '+-' and text[j - 1] in 'eEpP':
                        j += 1
                    elif d == "'" and j + 1 < n and (text[j + 1].isalnum() or text[j + 1] == '_'):
                        j += 2
                    else:
                        break
                i = j
            else:
                if c == '"' or c == "'":
                    st = 'str' if c == '"' else 'chr'
                    cls[i] = LITERAL
                i += 1
        elif st == 'line':
            if c == '\n' and not spliced(text, i):
                st = 'code'
            else:
                cls[i] = COMMENT
            i += 1
        elif st == 'block':
            cls[i] = COMMENT
            if c == '*' and nx == '/':
                cls[i + 1] = COMMENT
                st = 'code'
                i += 2
            else:
                i += 1
        else:
            cls[i] = LITERAL
            if c == '\\':
                if i + 1 < n:
                    cls[i + 1] = LITERAL
                i += 2
                continue
            if c == '\n':
                cls[i] = CODE
                st = 'code'
            elif (st == 'str' and c == '"') or (st == 'chr' and c == "'"):
                st = 'code'
            i += 1
    return cls


def blank_lines(text):
    """Drop the blank lines B1-B4 forbid. Returns (text, [(lineNo, rule)])."""
    trail = text.endswith('\n')
    lines = (text[:-1] if trail else text).split('\n')
    cls = char_classes(text)
    starts, at = [], 0
    for l in lines:
        starts.append(at)
        at += len(l) + 1

    def kinds(k):
        """The classes of line k's non-space characters."""
        s = starts[k]
        return {cls[s + x] for x, ch in enumerate(lines[k]) if not ch.isspace()}

    def ends_in_code(k):
        e = starts[k] + len(lines[k])
        return e >= len(text) or cls[e] == CODE

    def code(k):
        s = starts[k]
        return ''.join(ch for x, ch in enumerate(lines[k]) if cls[s + x] == CODE).strip()

    def protected(k):
        inside = k > 0 and cls[starts[k] - 1] != CODE
        return inside or (k > 0 and lines[k - 1].rstrip().endswith('\\'))

    def opens(k):
        return ends_in_code(k) and code(k).endswith('{')

    def closes(k):
        return code(k).startswith('}')

    def comment_only(k):
        return ends_in_code(k) and kinds(k) == {COMMENT}

    drop = {}
    k, count = 0, len(lines)
    while k < count:
        if lines[k].strip():
            k += 1
            continue
        j = k
        while j < count and not lines[j].strip():
            j += 1
        run = range(k, j)
        if any(protected(r) for r in run):
            # Only a first blank line can be protected by what precedes it; it
            # stays, so the rest are just extra.
            for r in run[1:]:
                if not protected(r):
                    drop[r] = 'B1'
        else:
            rule = None
            if k > 0 and opens(k - 1):
                rule = 'B2'
            elif j < count and closes(j):
                rule = 'B3'
            elif k > 0 and comment_only(k - 1):
                rule = 'B4'
            if rule:
                drop[k] = rule
            for r in run[1:]:
                drop[r] = 'B1'
        k = j
    if not drop:
        return text, []
    kept = [l for x, l in enumerate(lines) if x not in drop]
    return '\n'.join(kept) + ('\n' if trail else ''), \
        [(x + 1, drop[x]) for x in sorted(drop)]


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')


def sources():
    # Untracked files too, so a new file is checked before the commit that adds
    # it. --exclude-standard keeps build trees out.
    listed = subprocess.check_output(['git', 'ls-files'], cwd=ROOT).decode().split()
    listed += subprocess.check_output(
                  ['git', 'ls-files', '--others', '--exclude-standard'], cwd=ROOT).decode().split()
    return [f for f in listed
            if f.endswith(EXTS)
            and not f.startswith('vendor/') and 'third_party' not in f]


def parse(argv):
    """((apply, markdown, paths), None), or (None, what is wrong)."""
    apply, markdown, paths = False, None, []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == '--apply':
            apply = True
        elif a == '--markdown':
            if i + 1 >= len(argv):
                return None, '--markdown needs a file name'
            i += 1
            markdown = argv[i]
        elif a.startswith('--'):
            return None, 'unknown option ' + a
        else:
            paths.append(a)
        i += 1
    return (apply, markdown, paths), None


def named(paths):
    """The sources the paths cover, as (shown, path) pairs, or (None, what is wrong)."""
    root = os.path.abspath(ROOT)
    tree = None
    out, seen = [], set()
    for p in paths:
        full = os.path.abspath(p)
        try:
            rel = os.path.relpath(full, root).replace(os.sep, '/')
        except ValueError:
            rel = '..'                   # another drive
        inside = rel != '..' and not rel.startswith('../')
        if os.path.isdir(full):
            if inside:
                if tree is None:
                    tree = sources()
                prefix = '' if rel == '.' else rel + '/'
                found = [os.path.join(root, f) for f in tree if f.startswith(prefix)]
            else:
                found = sorted(os.path.join(d, f) for d, _s, fs in os.walk(full)
                               for f in fs if f.endswith(EXTS))
            if not found:
                print('  %s: no .cxx or .hxx file here' % p)
        elif os.path.isfile(full):
            if not full.endswith(EXTS):
                print('  %s: not a .cxx or .hxx file, left alone' % p)
                continue
            found = [full]
        else:
            return None, 'no such file or directory: ' + p
        for f in found:
            f = os.path.normpath(f)
            if f in seen:
                continue
            seen.add(f)
            try:
                r = os.path.relpath(f, root).replace(os.sep, '/')
            except ValueError:
                r = '..'
            out.append((f.replace(os.sep, '/') if r.startswith('..') else r, f))
    return out, None


def run(argv):
    args, err = parse(argv)
    if args is None:
        print('format.py: ' + err, file=sys.stderr)
        return 2
    apply, markdown, paths = args
    if paths:
        files, err = named(paths)
        if files is None:
            print('format.py: ' + err, file=sys.stderr)
            return 2
    else:
        files = [(f, os.path.join(ROOT, f)) for f in sources()]

    violations = []
    blanks = []
    skipped = 0
    rewritten = 0
    refused = 0
    eqfixed = 0

    for rel, path in files:
        if not os.path.isfile(path):
            continue
        raw = io.open(path, encoding='utf-8', errors='surrogateescape',
                      newline='').read()
        nl = '\r\n' if '\r\n' in raw else '\n'
        text = raw.replace('\r\n', '\n')
        original = text

        if apply:
            # One scan per round, edits applied back to front so every earlier
            # offset stays valid; rescanning after each edit is far too slow on
            # a large file. Overlapping edits (a call nested in another) are
            # dropped, not merged: the outer wins, the next round finds the inner.
            for _round in range(6):
                calls, line_of, starts = scan(text)
                edits = []
                for c in calls:
                    if nested_multiline(c, calls, line_of) \
                       or multiline_argument(text, c, line_of):
                        continue
                    s = shape(text, c, line_of, starts)
                    if s is None:
                        continue
                    op, cl, _seps, ln, _ind = c
                    edits.append((starts[ln], cl + 1, s[1]))
                if not edits:
                    break
                edits.sort(key=lambda e: e[0], reverse=True)
                claimed = None
                for (a, b, new) in edits:
                    if claimed is not None and b > claimed:
                        continue
                    text = text[:a] + new + text[b:]
                    claimed = a
            text, eq = unpad_equals(text)
            eqfixed += eq
            text, _gone = blank_lines(text)
            if text != original:
                if tokens(text) != tokens(original):
                    print('  !! %s  TOKENS DIFFER - not written' % rel)
                    refused += 1
                    continue
                io.open(path, 'w', encoding='utf-8', errors='surrogateescape',
                        newline='').write(
                            text.replace('\n', nl) if nl == '\r\n' else text)
                rewritten += 1
        else:
            _t2, eq = unpad_equals(text)
            eqfixed += eq
            calls, line_of, starts = scan(text)
            for c in calls:
                if nested_multiline(c, calls, line_of) \
                   or multiline_argument(text, c, line_of):
                    skipped += 1
                    continue
                s = shape(text, c, line_of, starts)
                if s is not None:
                    kind, _new, lineno, cols, args = s
                    violations.append((rel, lineno, kind, args, cols))
            blanks += [(rel, lineno, rule) for lineno, rule in blank_lines(text)[1]]

    if apply:
        print('%d file(s) rewritten' % rewritten)
        # Non-zero when a file was refused: "formatted" and "left alone because
        # the token check failed" must not look the same.
        return 1 if refused else 0

    by_kind = {}
    for v in violations:
        by_kind[v[2]] = by_kind.get(v[2], 0) + 1
    by_rule = {}
    for b in blanks:
        by_rule[b[2]] = by_rule.get(b[2], 0) + 1

    print('call wrapping: %d violation(s)' % len(violations))
    print('padded `=`   : %d line(s)' % eqfixed)
    for k in sorted(by_kind):
        print('  %-10s %d' % (k, by_kind[k]))
    if skipped:
        print('  %-10s %d (nested multi-line call, left alone)' % ('skipped', skipped))
    print('blank lines  : %d line(s)' % len(blanks))
    for k in sorted(by_rule):
        print('  %-10s %d (%s)' % (k, by_rule[k], BLANK_RULES[k]))

    if markdown:
        with io.open(markdown, 'w', encoding='utf-8') as fh:
            fh.write('# Call wrapping\n\n')
            if not violations:
                fh.write('No violations.\n')
            else:
                fh.write('%d violation(s). The rule: arguments stay on one line '
                         'when they fit in %d columns and there are at most %d of '
                         'them; otherwise the `(` ends its line, every argument '
                         'takes its own line one indent in, and the `)` returns to '
                         "the call's own column.\n\n"
                         % (len(violations), LIMIT, MAX_ARGS))
                fh.write('| File | Line | What | Args | Cols |\n')
                fh.write('|---|---:|---|---:|---:|\n')
                for rel, lineno, kind, args, cols in violations[:400]:
                    what = ('fits - put it on one line' if kind == 'unwrap'
                            else 'wrap it: one argument per line, hanging indent')
                    fh.write('| `%s` | %d | %s | %d | %d |\n'
                             % (rel, lineno, what, args, cols))
                if len(violations) > 400:
                    fh.write('\n_%d more not listed._\n' % (len(violations) - 400))
            if skipped:
                fh.write('\n%d call(s) skipped: a nested call spans lines, so the '
                         'outer one is left for a person.\n' % skipped)
            fh.write('\n# Blank lines\n\n')
            if not blanks:
                fh.write('No violations.\n')
            else:
                fh.write('%d blank line(s) to remove. No two blank lines in a row, '
                         'and none after a line ending in `{`, before a line '
                         'starting with `}`, or between a comment and the line '
                         'after it.\n\n' % len(blanks))
                fh.write('| File | Line | What |\n')
                fh.write('|---|---:|---|\n')
                for rel, lineno, rule in blanks[:400]:
                    fh.write('| `%s` | %d | %s |\n' % (rel, lineno, BLANK_RULES[rule]))
                if len(blanks) > 400:
                    fh.write('\n_%d more not listed._\n' % (len(blanks) - 400))
        print('wrote %s' % markdown)

    return 1 if (violations or eqfixed or blanks) else 0


if __name__ == '__main__':
    sys.exit(run(sys.argv[1:]))
