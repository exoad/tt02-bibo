// The Log tab: what the page saw happen, one timestamped line each, newest
// at the bottom - the pilot's last words from /json, the feed coming and
// going, the board answering or not, and what LOOK and STOP were told. Small
// and honest: no styling beyond the well it sits in, and nothing is said
// twice in a row.

const MAX = 400;
const lines = [];
let el = null, last = '';

export function init(node) { el = node; render(); }

export function add(text) {
  if (text === last) return;
  last = text;
  const t = new Date();
  const two = function (n) { return (n < 10 ? '0' : '') + n; };
  lines.push(two(t.getHours()) + ':' + two(t.getMinutes()) + ':' + two(t.getSeconds()) + '  ' + text);
  if (lines.length > MAX) lines.shift();
  render();
}

// Scrolls to the newest line; called when the tab is shown, since a hidden
// pre has no scroll height to go to.
export function follow() {
  if (el && el.parentNode) el.parentNode.scrollTop = el.parentNode.scrollHeight;
}

function render() {
  if (!el) return;
  el.textContent = lines.join('\n');
  follow();
}
