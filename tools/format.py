"""Call-site wrapping, checked or fixed.

    python tools/format.py                 report violations
    python tools/format.py --apply         rewrite them
    python tools/format.py --markdown OUT  write the report as a table

THE RULE

  A call's arguments stay on ONE LINE when they fit inside 100 columns and
  there are no more than six of them.

  Otherwise the call takes a HANGING INDENT: the `(` ends its line, every
  argument gets its own line one indent level in, and the `)` returns to the
  column the call's own line starts at.

      bibo::serial::printf(
          "OK sensors i2c=%d tof=%d addr=0x%02X\\n",
          i2c,
          tof,
          addr
      );

  Not paren-alignment. Aligning to the open paren pushes arguments far to the
  right for exactly the calls that are already too long - a qualified name like
  bibo::serial::printf costs 21 columns before an argument is written - and the
  indent then changes whenever the function is renamed.

WHY BOTH HALVES ARE NEEDED. "Do not wrap" alone forces 200-column lines onto
printf calls with a long format string. "Wrap freely" alone is what the tree
had: 296 calls that fit on one line and wrapped anyway, and 718 wrapped in a
dozen different shapes.

SAFETY. This only ever moves whitespace, so the token stream must come out
identical - --apply proves that per file with a comment- and string-aware
tokenizer and refuses to write when it does not hold. A call whose arguments
contain a nested call that ITSELF spans lines is reported and left alone:
re-indenting the outer one would leave the inner hanging off a column that no
longer exists.
"""
import io
import os
import re
import subprocess
import sys

APPLY = '--apply' in sys.argv
LIMIT = 100
INDENT = 4
MAX_ARGS = 6

# Not calls. `if(`, `sizeof(` and the casts take parentheses and are not
# argument lists; wrapping them by this rule would be nonsense.
KEYWORDS = {'if', 'while', 'for', 'switch', 'return', 'catch', 'sizeof',
            'static_cast', 'reinterpret_cast', 'const_cast', 'dynamic_cast',
            'defined', 'decltype', 'noexcept', 'alignof', 'and', 'or', 'not'}

# A declaration head. Those are the style audit's 'wrapped parameter list'
# rule, and having two rules disagree about one line helps nobody.
DECL = re.compile(
    r'^\s*(?:\[\[nodiscard\]\]\s*)?(?:static\s+|inline\s+|constexpr\s+|const\s+|virtual\s+)*'
    r'(?:Void|Bool|Int8|Int16|Int32|Int64|UInt8|UInt16|UInt32|UInt64|Float32|'
    r'Float64|Size|Str|Char|Utf8|CharSeq|Pin|auto|void|bool|int)\b')


def blank_noise(text):
    """Comment and string BODIES become spaces, offsets preserved.

    A `(` inside a literal is not a call and a comma inside one is not a
    separator - and a naive scan finds both.
    """
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
    """Whitespace runs to one space - OUTSIDE string and char literals only.

    THE BUG THIS EXISTS TO PREVENT, because it already happened once. A plain
    re.sub(r'\\s+', ' ') over an argument list rewrites the CONTENTS of every
    literal in it, so

        std::printf("  FAIL  %s: got %.3f\\n", what, got)

    came back as "` FAIL %s...`" - two spaces to one, in output that four test
    suites assert on exactly. It reached 24 files before a test caught it.
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


def tokens(text):
    """A normal form that IGNORES layout and RESPECTS literals.

    The first version of this collapsed all whitespace including a literal's
    contents, which made it blind to exactly the damage the formatter was
    capable of doing. A check that cannot fail on the tool's own worst bug is
    not a check.
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
        if DECL.match(lines[ln]) or lines[ln].lstrip().startswith('#'):
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
    if not multi and wants:
        pass                             # one line but should not be
    if multi and wants:
        # Already wrapped - is it the right shape?
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
    """Does any single argument already span more than one line?

    IF SO THE CALL IS LEFT ALONE, and this is not fussiness. Putting each
    argument on "its own line" means collapsing whatever it currently spans,
    and two shapes in this tree do not survive that:

      a // comment inside the argument list - collapsed, it swallows the rest
      of the line and the call stops compiling. lsp.cxx's initialize request is
      exactly this, and it is why the token check refused five files.

      a run of adjacent string literals split over eight lines - legal to
      collapse, and it produces one 400-column line, which is the opposite of
      what this rule is for.

    Both are a person's decision, not a formatter's.
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

    # A line comment anywhere inside the parentheses. blank_noise turned its
    # body to spaces, so a `//` in the original that is blank in the cleaned
    # copy is a real comment rather than two characters inside a string.
    for i in range(op, min(cl, len(code) - 1)):
        if text[i] == '/' and text[i + 1] == '/' \
           and code[i] == ' ' and code[i + 1] == ' ':
            return True
    return False


# ---- padded `=` ------------------------------------------------------------
#
# THE RULE: one space before `=`, never a run of them to line a column up.
#
#     Int32   servoNow    = STEER_CAL_CENTER;      <- no
#     Int32 servoNow = STEER_CAL_CENTER;           <- yes
#
# Column alignment looks tidy in the editor it was typed in and costs
# something every time afterwards: renaming one field re-pads its whole block,
# so a one-line change arrives as a twelve-line diff and the actual edit is
# hidden among them. It also decays - the moment a longer name is added and
# nobody re-pads, the column is a lie.
#
# Only the run BEFORE `=` is touched. Nothing after it moves, and `==`, `<=`,
# `>=`, `!=`, `+=` and friends are left alone: this is about padding, not
# about spacing around operators.
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

        # The `=` must be real code, not something inside a string or comment.
        eq = len(m.group(1)) + len(m.group(2))
        if eq >= len(clean[i]) or clean[i][eq] != '=':
            continue

        # A trailing `\` makes this a macro continuation, where the padding is
        # sometimes load-bearing to the eye and always harmless to leave.
        if line.rstrip().endswith('\\'):
            continue

        lines[i] = m.group(1) + ' =' + line[eq + 1:]
        hits += 1

    return '\n'.join(lines), hits


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')


def sources():
    return [f for f in subprocess.check_output(
                ['git', 'ls-files'], cwd=ROOT).decode().split()
            if f.endswith(('.cxx', '.hxx'))
            and not f.startswith('vendor/') and 'third_party' not in f]


def run():
  files = sources()

  violations = []
  skipped = 0
  rewritten = 0
  eqfixed = 0

  for rel in files:
    path = os.path.join(ROOT, rel)
    if not os.path.isfile(path):
        continue

    raw = io.open(path, encoding='utf-8', errors='surrogateescape',
                  newline='').read()
    nl = '\r\n' if '\r\n' in raw else '\n'
    text = raw.replace('\r\n', '\n')

    if APPLY:
        # ONE SCAN PER ROUND, edits applied BACK TO FRONT.
        #
        # Rewriting one call and rescanning the whole file is O(file) per edit,
        # and app_ui.cxx alone had 338 of them - the loop did not finish in
        # seven minutes. Applying from the end means every earlier offset is
        # still valid, so a round costs one scan.
        #
        # Overlapping edits are dropped rather than merged: a call nested
        # inside another produces two spans covering the same text, and
        # applying both would write the inner one into a region the outer had
        # already replaced. The outer wins this round, the inner is found by
        # the next.
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
                    continue        # overlaps one already applied this round
                text = text[:a] + new + text[b:]
                claimed = a

        text, eq = unpad_equals(text)
        eqfixed += eq

        if text != raw.replace('\r\n', '\n'):
            if tokens(text) != tokens(raw):
                print('  !! %s  TOKENS DIFFER - not written' % rel)
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
            if nested_multiline(c, calls, line_of)                or multiline_argument(text, c, line_of):
                skipped += 1
                continue
            s = shape(text, c, line_of, starts)
            if s is not None:
                kind, _new, lineno, cols, args = s
                violations.append((rel, lineno, kind, args, cols))

  if APPLY:
      print('%d file(s) rewritten' % rewritten)
      return 0

  # ---- the report --------------------------------------------------------
  by_kind = {}
  for v in violations:
      by_kind[v[2]] = by_kind.get(v[2], 0) + 1

  print('call wrapping: %d violation(s)' % len(violations))
  print('padded `=`   : %d line(s)' % eqfixed)
  for k in sorted(by_kind):
      print('  %-10s %d' % (k, by_kind[k]))
  if skipped:
      print('  %-10s %d (nested multi-line call, left alone)' % ('skipped', skipped))

  if '--markdown' in sys.argv:
      out = sys.argv[sys.argv.index('--markdown') + 1]
      with io.open(out, 'w', encoding='utf-8') as fh:
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
      print('wrote %s' % out)

  return 1 if (violations or eqfixed) else 0


if __name__ == '__main__':
    sys.exit(run())
