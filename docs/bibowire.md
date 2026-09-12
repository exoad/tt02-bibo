# bibowire v1 — the viewer ↔ Orange Pi protocol

**Status:** specification, implementable as written.
**Replaces:** `scanwire` **for the viewer only**. `scanwire` on TCP 8011/8012, `feed.cxx`, `scanfeed`, `/tmp/bibo-scan.txt` and the phone dashboard are **untouched and stay running**. (The phone dashboard has since been removed; the rest still runs.)
**Shared module:** `firmware/pilot/src/bibowire.hxx` / `.cxx`, on `pilotlib`, compiled into both the board's program and the viewer.

---

## 0. The decision, and the disagreements

Three judges picked three different winners. **Design 2 (binary framed, TCP telemetry + UDP control) is the spine** — it is the only design no judge ranked last, the only one with a drop priority *across* message kinds (which is what makes a camera safe to add to a 220 kbit/s link at all), and the only one whose transport-independence is demonstrated rather than promised, because the identical 16-byte frame already runs on both TCP and UDP in v1.0.

Where the judges disagreed, one sentence each:

| Disagreement | Decision | Why |
|---|---|---|
| Text (J2) vs binary (J1, J3) | **Binary** | The pilot must keep formatting the `F` text line every tick for the dashboard regardless (`app/main.cxx:804-809`), so the human-readable path survives on 8011 for free — which was the only thing a text viewer wire was buying. (The dashboard has since been removed; the `F` line is still formatted every tick for the text feed.) |
| UDP-only (J1) vs TCP+UDP (J3) | **Both, control on UDP** | J1's real complaint — a stall's backlog of stale steering delivered in order — is a property of the *control* channel only, and `feed.cxx` cannot be deleted anyway (the dashboard is a feed client and `scanfeed.cxx:236` relays through the pilot's own feed port; the dashboard has since been removed, the hub and the relay remain). |
| `CONTROL_DEAD_MS` = 400 (D2) vs 300 (D4) | **300** | J2 is right that this repo's rule is that the Pi's stop *comfortably beats* the Pico's, the way `REV_WAIT_MS = 200` is deliberately half of `DEADMAN_MS 400u`; two 400s racing means the winner is the one that reports nothing to anybody. |
| Camera on this link (D2/D3) vs relocated to HTTP (D1) | **On this link, as `CLASS_BULK`** | Relocating it means the system's largest stream gets no version handshake, no session id and no shared module — the one rule this protocol exists to keep. |
| CRC32C exempted for large payloads (D2's own weakness) | **No exemption, ever** | 40 KB at 30 fps is 1.2 MB/s, which at the ARMv8 CRC32C instruction's ~8 B/cycle is ~150 K cycles/s — 0.006 % of one core, so the special case D2 feared is not worth its own branch. |

**Fatal flaws fixed.** D3's fatal flaw (two viewers interleaving at 20 Hz, and viewer B's stream feeding the deadman that was protecting viewer A) is closed twice over: exactly one **control holder** exists, and the deadman is fed **only** by `CONTROL` frames carrying the holder's `sessionId`. D3's second hazard (deadman disarmed in `look`/`drive`, e-stop needing the link that just died) is closed by making the `CONTROL` stream the operator's *consent* in those modes. D2's near-miss (Pi dead-timer equal to the Pico's) is closed by 300 ms plus a `static_assert`. D2's `assumedMode`-looks-like-a-hardware-fault problem is closed by `CTLSTATE.refuse`, a named reason on the wire twenty times a second.

**Grafted in, with attribution:** `armEpoch` and `Opt<T>` stale accessors and `describe()` from Design 4; the compile-time deadman ordering, "**parsed**, not received" and immediate-stop-on-positive-evidence from Design 3; the 20 Hz what-the-board-*did* line, its deadman countdown computed by the same code that trips, and one-waypoint-per-message from Design 1.

---

## 1. What this is, and the one paragraph of reasoning

bibowire carries one lidar revolution, one autonomy decision per revolution, the board's and car's state, and the operator's control, between the Windows viewer and the pilot on the Orange Pi, over a phone hotspot that stalls for one to five seconds and drops entirely when the phone moves.

The shape follows from one asymmetry. **Telemetry is a picture and control is a command, and they fail in opposite directions.** A picture that arrives late is worthless but harmless — the right answer is to drop the old one and send the new one, which needs a reliable in-order channel only so that a 2.5 KB revolution arrives whole. A command that arrives late is *dangerous*: a three-second stall on an ordered stream delivers sixty queued steering commands in order when it clears, and the board applies a three-second-old throttle before it can reach the fresh one — a car accelerating on a command from a viewer that was out of contact, at the moment the operator has started walking toward it. So telemetry rides TCP with a bounded, priority-classed, drop-oldest queue, and control rides UDP where the board drains the socket to empty and keeps only the newest sequence number. Everything else in this document is a consequence: one frame shape on both transports so there is one decoder to test; fixed-width integer bodies so there is no locale, no `strtoll` stopping early on `"12abc"`, and no allocation sized from a number a stranger on a hotspot wrote; and every age, every refusal reason and every count the board measured *about itself* on the wire at 20 Hz, because "reports success while measuring nothing" is this repo's named recurring bug and a stale picture drawn as live is that bug with a steering wheel attached.

---

## 2. Transport, ports, lifecycle

**Port 8020, TCP and UDP, the same number for both.** `scanwire::PORT` (8011) and `scanwire::PILOT_PORT` (8012) keep their meanings; 8020 is deliberately a clear gap from that pair, so `801x` means "the text feed a person can read" and `8020` means "bibowire" with no arithmetic.

```cpp
constexpr UInt16 PORT = 8020;
```

There is **no fallback port and no relay**. `scanwire`'s two-port dance exists because `scanfeed` idles under systemd on 8011 and nobody has root on the board to stop it; nothing idles on 8020, and a second protocol quietly moving next door is how you get two feeds nobody can tell apart.

**Sockets.** Every socket non-blocking, `poll()`-driven, on the view-feed thread — the arrangement `feed.hxx` already runs and already justifies. On TCP, both ends set `TCP_NODELAY` unconditionally (a 40-byte `CMDACK` must not sit in Nagle's queue behind the 2540-byte `SCAN` it follows) and `SO_KEEPALIVE` with `TCP_KEEPIDLE 2`, `TCP_KEEPINTVL 1`, `TCP_KEEPCNT 3` (a phone that walks out of range stops ACKing without a FIN, and the default keepalive is two hours). UDP receive buffer 64 KiB on the board — deliberately **small**, because a deep queue of control datagrams is a queue of stale steering commands and the board wants the newest, not the most. Viewer `SO_RCVBUF` 256 KiB.

**Lifecycle.**

1. Viewer opens TCP to `bibobox.local:8020` (the **name**, never a cached address — the hotspot's DHCP hands out a different one every outing) with a 3000 ms connect deadline, and binds a UDP socket.
2. Viewer sends `HELLO` as the first bytes on the connection and sends nothing else until `WELCOME` arrives.
3. Board answers `WELCOME`, then `LIDAR_INFO`, then `BOARD`, then the next `SCAN`. **State before scan, always**, so the viewer has something true to draw the moment the picture appears.
4. Steady state: TCP carries `SCAN`/`DECIDE` at 10 Hz, `BOARD` at 5 Hz, `PING` at 1 Hz, `EVENT`/`CMDACK` as they happen. UDP carries `CONTROL` up at 20 Hz and `CTLSTATE` down at 20 Hz.
5. Close: `LEAVE` (viewer, deliberate) or `BYE` (either end, with a reason and a sentence), then the socket shuts.

A `LEAVE`, a FIN or an RST from the control holder is **positive evidence the driver is gone** and stops the car on that tick — it does not wait out the 150 ms timer. The kernel already has that information, and spending a sixth of a second rediscovering it by timeout is a sixth of a second of a car driving on a command from a viewer that is provably not there.

---

## 3. Framing, byte for byte

Every frame, both directions, both transports, is the same shape. Little-endian, `static_assert`ed: both ends are LE forever (aarch64 Linux, x86-64 Windows), and byte-swapping every field to buy a portability nobody will use costs the Pi cycles it has better uses for.

```
HEADER — exactly 12 bytes
  off  sz  field
   0   2   magic    0x5742  (the bytes 0x42 0x57 — 'B','W' — appear in that order on the wire)
   2   1   type     Type tag; see §4
   3   1   ver      body version FOR THIS TYPE, starts at 1, never 0
   4   2   flags    see below
   6   2   seq      per-connection, per-direction FRAME counter, wraps at 65536
   8   4   len      payload bytes, always a multiple of 4, <= MAX_PAYLOAD

PAYLOAD — `len` bytes at offset 12, 4-byte aligned; every field naturally aligned within it

TRAILER — 4 bytes
  u32 crc32c over bytes [0, 12+len) — header AND payload

TOTAL = 16 + len
```

The header `seq` is a diagnostic counter for the frame-header ring (§7) and for loss accounting. **It is never the ordering rule for control** — `CONTROL` carries its own `UInt32 seq` in its body, because a 16-bit counter wrapping every 55 minutes at 20 Hz is not something a safety comparison should be built on.

**Flags (`UInt16`).**

| bit | name | meaning |
|---|---|---|
| 0 | `FLAG_ESTOP` | sender has emergency stop latched |
| 1 | `FLAG_DEADMAN` | sender's deadman is not in the LIVE state |
| 2 | `FLAG_MORE` | reserved for fragmenting a future camera frame; **MUST be 0 in v1**, and a frame carrying it is REFUSED |
| 3–15 | — | written zero; a reader **ignores** unknown bits |

The asymmetry is the rule that makes minor versions safe: a flag that changes **framing** must be refused by a reader that cannot honour it, and a flag that only **annotates** may be ignored, so v1.1 can add one. Bits 0 and 1 are set on *every* board→viewer frame, so "the car is stopped" is derivable from any frame that arrives, even one whose body this reader does not understand.

**Constants.**

```cpp
constexpr UInt16 PROTO_MAJOR        = 1;
constexpr UInt16 PROTO_MINOR        = 0;
constexpr Size   MAX_PAYLOAD        = 262128;   // frame <= 256 KiB; sized for a future JPEG
constexpr Size   MAX_INBOUND_PAYLOAD= 256;      // what the BOARD accepts from a viewer
constexpr Size   MAX_DATAGRAM       = 1400;     // UDP; see below
constexpr Size   MAX_SCAN_POINTS    = 1024;
constexpr Size   MAX_EVENT_TEXT     = 200;
constexpr Size   MAX_NAME           = 31;
constexpr Size   MAX_CLIENTS        = 4;
```

`MAX_DATAGRAM` is 1400 and not 1472: the board is also reachable over Tailscale, and WireGuard's ~60 bytes of overhead would fragment a 1472-byte datagram *inside* the tunnel. 1400 survives the tunnel and the hotspot both. No v1 UDP message exceeds 60 bytes; the bound exists so the receive buffer is a fixed `Array<UInt8, 1400>` and nothing sizes an allocation from a claim.

**Buffer bounds, exact, no heap in the receive path.**

- Board, per TCP client: `Array<UInt8, 1024>` receive ring. A header claiming `len > MAX_INBOUND_PAYLOAD` is not a big frame, it is a hostile or broken peer: `BYE(BYE_TOO_BIG)` and close, **without ever allocating for the claim**. This is the single most important bound in the design.
- Board, UDP: `Array<UInt8, 1400>` per `recvfrom`; a `CONTROL` frame is exactly 40 bytes, and anything that is not a well-formed known type is dropped and counted in `rxControlStale`.
- Board, per TCP client outbound: 96 KiB **or** 12 frames, whichever fills first, with the drop classes of §6.
- Viewer: 512 KiB receive ring, one preallocated scratch buffer of `MAX_PAYLOAD` taken at connect and reused forever.

**Decoder.** One function, allocation-free; the returned `Frame` points *into* the caller's buffer.

```cpp
enum class Take
{
    TAKE_FRAME = 0,
    TAKE_NEED_MORE,   // the ONLY non-error partial answer; consumes nothing
    TAKE_RESYNC,      // `consumed` is the junk skipped; counted, never silent
    TAKE_TOO_BIG,
    TAKE_BAD_FLAG,    // a framing flag this reader cannot honour
};
```

**Resync is deterministic.** Scan forward for the pair `0x42 0x57`; reject the candidate unless `type` is a known tag, `ver` is nonzero, `len <= MAX_PAYLOAD` and `len % 4 == 0` and the CRC over the whole candidate verifies. Otherwise advance **exactly one byte** and try again. (The known-tag condition belongs to *resync only* — scanning forward for a lock. At a frame boundary the CRC alone decides, or an unknown type could not be skipped and §7's promise would be false. See §12.1.) Magic plus CRC together put a false frame lock near 2⁻⁴⁸ per candidate rather than the 2⁻¹⁶ a magic alone would give.

**Field access is by explicit `rd16`/`rd32`/`rd64` `memcpy` helpers, never by casting a struct pointer at the payload.** On LE both compile to a single load, and the helper version cannot be silently broken by a compiler's padding decision or by an aarch64 alignment fault on a field added next year.

**No floating point anywhere on the wire.** Every quantity is a fixed-point integer with its unit in its name. That is not compactness — it is the locale bug `proto.cxx` and `scanwire.cxx` both already met (`printf("%.3f")` honours the locale, a machine set to a comma decimal emits `0,250`, the far end reads `0`, and that is a silent hard-left at the first corner on somebody else's laptop) deleted at the root: there is no formatting step to be locale-sensitive, no NaN, no denormal, and every round-trip test is byte-exact rather than approximately equal.

---

## 4. The handshake and versioning

### `HELLO` — 0x01, viewer→board, TCP, first bytes on the connection

```
 0  u16  protoMajor
 2  u16  protoMinor
 4  u32  featureMask       types this viewer understands
 8  u32  viewerBuild       git short hash, as a hex number
12  u16  viewerUdpPort     where the board sends CTLSTATE
14  u16  wantControl       0 = observer, 1 = asking for the control slot
16  u16  controlHz         intended CONTROL rate, informational
18  u8   nameLen           <= MAX_NAME (31)
19  u8   reserved0         zero
20  u8[nameLen] name       ASCII, then 0..3 zero bytes so len % 4 == 0
```

Names are a **length-prefixed tail**, not a fixed 16-byte field. D2's fixed field truncated a NetworkManager profile or a hostname silently, and putting the variable part last means no fixed offset ever moves.

### `WELCOME` — 0x02, board→viewer, TCP

```
 0  u16  protoMajor
 2  u16  protoMinor
 4  u32  sessionId         random, NEVER 0, new for every accepted HELLO
 8  u32  bootId            new for every pilot process
12  u32  featureMask       what this board will send
16  u16  controlUdpPort    = PORT
18  u16  controlPeriodMs   = 50
20  u16  staleMs           = 150
22  u16  deadMs            = 300
24  u16  accepted          0 refused, 1 accepted as CONTROL, 2 accepted as OBSERVER
26  u16  refusal           0 none, 1 version, 2 control taken, 3 too many viewers, 4 not ready
28  u64  boardMonoUs
36  u8   armEpoch
37  u8   capabilities      b0 canDrive, b1 hasLidar, b2 hasPico, b3 hasBattery
38  u8   nameLen
39  u8   textLen           <= 96 — the sentence a person reads
40  u8[nameLen] boardName, then u8[textLen] text, then pad to 4
```

**The board answers every well-formed `HELLO`, including one it refuses.** Silence is never an answer, because on this link silence already means four other things.

**Version is decided on one rule: `protoMajor` must be EQUAL.** Not `>=`, not "compatible". On mismatch:

```
BYE(reason = BYE_VERSION,
    text  = "board speaks bibowire 2.0, viewer sent 1.0 - rebuild the viewer from
             the same commit as the board (board build 3f9a1c2, 2026-09-10)")
```

and close. The viewer applies the same test to `WELCOME` and shows that sentence in its connection banner rather than a spinner. The text is not decoration: a refusal that says only "incompatible" sends a person to read source in a field, and the two build stamps are what turn it into an action. Because `magic` and `type` and `ver` are the first four bytes on the socket, a mismatch is caught before a single field of a body is interpreted.

**`protoMinor` is free to differ**, and minor is defined narrowly so that is safe. A minor bump may **only**: add a new message type; add trailing fields to an existing body (with that type's `ver` bumped); or define a previously reserved *annotating* flag bit. It may never move a field, change a unit, or reuse a type tag. A reader given a `ver` **higher** than it knows reads the prefix it understands and ignores the tail — which works precisely because `len` is authoritative and every body is fixed-width up to the point the new field begins. A reader given a `ver` **lower** than it knows must **not fabricate the missing tail**; it marks those fields absent, which is what the explicit sentinels are for (`0xFFFF` battMilliV, `255` health, `-32768` cpuCentiC, `0xFFFFFFFF` picoSilentMs). Absence is representable everywhere, so a new field never has to be faked by a zero.

**What `WELCOME` hands back, and why each one exists:**

- **`sessionId`** — every `CONTROL` datagram carries it, and the board drops any datagram whose `sessionId` is not the current one, counting it in `rxControlStale`. Concretely: after a hotspot blip the viewer reconnects, and datagrams from the previous session may still be in flight in a buffer somewhere; without the id they would drive the car with a second-old stick position.
- **`bootId`** — a viewer that reconnects and sees a *different* `bootId` throws away everything it knew (scan, decision, arm state, mode, `revIndex` continuity) before drawing anything. This is the specific defence against the most convincing stale picture there is: a healthy new socket to a restarted car, still showing the previous run.
- **`boardMonoUs`** — the base for every timestamp in the protocol.
- **the three deadman numbers** — so the viewer's control cadence and its countdown come from the **board** rather than from a constant compiled into the viewer months earlier.

### `BYE` — 0x03, either direction, TCP

```
0  u16  reason
2  u16  textLen
4  u8[textLen] ASCII, pad to 4
```

Reasons: `1 VERSION`, `2 TOO_BIG`, `3 BAD_CRC`, `4 BAD_SESSION`, `5 TIMEOUT`, `6 SHUTDOWN`, `7 SUPERSEDED`, `8 REFUSED`, `9 BAD_FLAG`.

### `PING` / `PONG` — 0x04 / 0x05, either direction, TCP, 16 bytes

```
0  u64  token          echoed verbatim
8  u64  senderMonoUs
```

### `LEAVE` — 0x06, viewer→board, TCP, 8 bytes

```
0  u32  sessionId
4  u32  reserved0
```

Releases the control slot **immediately**, and stops the car on that tick if this viewer held it.

### The control slot

`wantControl = 1` asks for it. **At most one viewer holds it.** A second asker is accepted as an observer with `refusal = 2` and a sentence naming who holds it and since when; it receives all telemetry, and its `CONTROL` datagrams are counted in `rxControlStale` and **discarded**, never blended. The slot is released by `LEAVE`, by close, or by 1000 ms of control silence.

1000 ms and not 300: the car has already been stopped by the deadman at 300, and handing the wheel to somebody else 300 ms into a stall — while the first operator is still holding the throttle and about to come back — would be worse than the stall.

### The reverse-path probe

Within **1000 ms** of sending `WELCOME` to a viewer that asked for control, the board must have received **at least 5 `CONTROL` datagrams from it**. If it has not:

```
EVENT(warn, "no control datagrams on UDP 8020 from 192.168.43.7 - telemetry is up
             and the car will not move; the viewer is falling back to TCP control")
```

and the board sets `refuse = REFUSE_NO_UDP` in `CTLSTATE`. This closes the worst asymmetric failure the two-transport choice creates, and it closes it by **measuring the path rather than assuming it because the other direction is fine** — which is this repo's recurring bug class applied to the link itself.

### TCP control fallback

If the **viewer** has received no `CTLSTATE` datagram within 1000 ms of `WELCOME`, it begins sending `CONTROL` frames on the **TCP** connection instead, at the same 20 Hz, and displays a persistent `degraded control (TCP)` banner. The board accepts `CONTROL` on TCP **always**, with identical rules, identical deadman, identical `sessionId` and `seq` checks — it is the same frame on a different socket, so there is nothing to negotiate and no second code path. While fallback is active the board mirrors `CTLSTATE` onto TCP as well.

The head-of-line exposure this creates is **bounded by an invariant, not by hope**: at most **one** `SCAN` frame may be queued per client at any instant (`CLASS_LIVE`, depth 1, drop-oldest, §6), so a `CONTROL` or `CTLSTATE` frame waits behind at most 2540 bytes — about 20 ms on a working hotspot link.

---

## 5. Every message

Type tags are grouped so a reader can classify by range: `0x00–0x0F` session, `0x10–0x3F` board→viewer, `0x40–0x4F` viewer→board, `0xF0+` introspection.

| tag | name | dir | transport | cadence | class |
|---|---|---|---|---|---|
| 0x01 | `HELLO` | V→B | TCP | once | vital |
| 0x02 | `WELCOME` | B→V | TCP | once | vital |
| 0x03 | `BYE` | both | TCP | once | vital |
| 0x04/0x05 | `PING`/`PONG` | both | TCP | 1 Hz | vital |
| 0x06 | `LEAVE` | V→B | TCP | once | vital |
| 0x10 | `SCAN` | B→V | TCP | 10 Hz | **live** |
| 0x11 | `DECIDE` | B→V | TCP | 10 Hz | **live** |
| 0x12 | `BOARD` | B→V | TCP | 5 Hz | vital |
| 0x13 | `LIDAR_INFO` | B→V | TCP | on connect + change | vital |
| 0x14 | `EVENT` | B→V | TCP | as they happen | vital |
| 0x15 | `CTLSTATE` | B→V | **UDP** | 20 Hz | — |
| 0x16 | `CMDACK` | B→V | TCP | one per `COMMAND` | vital |
| 0x20 | `CAMERA` | B→V | TCP | reserved | **bulk** |
| 0x21 | `POSE` | B→V | TCP | reserved | live |
| 0x22 | `PATH` | B→V | TCP | reserved | vital |
| 0x23 | `WAYPOINT` | B→V | TCP | reserved | vital |
| 0x40 | `CONTROL` | V→B | **UDP** | 20 Hz | — |
| 0x41 | `COMMAND` | V→B | TCP | on an act | — |
| 0x42 | `SUBSCRIBE` | V→B | TCP | on change | — |
| 0xF0/0xF1 | `DESCRIBE`/`SCHEMA` | both | TCP | on request | vital |

### `SCAN` — 0x10, board→viewer, TCP, 10 Hz

```
 0  u64  tMonoUs             board clock when the revolution COMPLETED
 8  u32  revIndex            monotonic since pilot boot, never 0; gaps are how the
                             viewer knows exactly what it missed
12  u16  freqMilliHz         10000 = 10.000 Hz
14  u16  count               <= MAX_SCAN_POINTS (1024)
16  u8   health              0 good, 1 warn, 2 error, 255 unknown (lidar::Device::health
                             -1 becomes 255)
17  u8   motor               0/1
18  u16  droppedSinceLast    revolutions this sender discarded for THIS client
20  u16  scanDivisor         the divisor in force when this frame was made
22  u16  reserved0           zero
24  ... count x { u16 angleCentiDeg (0..35999) ; u16 distMm (0 = NO RETURN) }
    ... count x u8 quality (0..63)
    ... 0..3 zero bytes

len = 24 + 4*count + ((count + 3) & ~3u)
500 points: len = 2524, frame = 2540 bytes
```

Units and ranges are `scanwire`'s exactly — 360° wraps to 0 and is never 36000, `distMm == 0` means no return and not zero range, quality clamps at 63 — so every edge case the existing 79 `scanwire` checks already pin carries over in meaning.

**`scanDivisor` is echoed on the frame it governs.** This is Design 3's `rec=` rule generalised: a capture that starts mid-session is fully interpretable without knowing what was negotiated at connect time, and an inferred field is a field that can be inferred wrong. The same rule binds the future camera's codec tag.

**Two parallel arrays rather than one 6-byte interleaved record**, because the quality array is then a straight `memcpy` from the `Vec<UInt8>` that `lidar::grab(out, timeoutMs, &quality)` already fills. The point array is **not** a memcpy and this document will not claim it is: `reactive::Ray` is two `Float32` (degrees, millimetres), so the point array is a 500-iteration convert-and-round loop. It is the same loop `frameLine` already runs, minus the `snprintf`.

`count` is a promise about the frame. A `SCAN` whose `count` disagrees with its `len` is **refused outright** — a frame with the wrong count is not a shorter frame, which is `test_scanwire.cxx`'s own words about the text format and is the same rule here.

### `DECIDE` — 0x11, board→viewer, TCP, immediately after the `SCAN` it describes

```
 0  u32  revIndex        ties it to that SCAN; 0 = a BLIND tick with no revolution behind it
 4  u32  clearanceMm
 8  u16  hits
10  i16  steerMilli      -1000..1000, left negative
12  i16  throttleMilli   -1000..1000, negative is reverse
14  u8   mode            reactive::Mode: 0 CRUISE 1 SLOW 2 STOP 3 REVERSE 4 BLIND
15  u8   stop            0/1
16  u8   source          0 manual, 1 look, 2 drive - WHO produced these numbers
17  u8   reserved0
18  u16  reserved1
20  u32  modeMs

len = 24, frame = 40 bytes
```

A `DECIDE` whose `revIndex` names a `SCAN` the viewer never received is **discarded, not drawn**. The corridor and heading overlay belongs to one revolution, and drawing it over a different one produces a picture that is individually plausible and jointly false.

### `BOARD` — 0x12, board→viewer, TCP, 5 Hz

```
 0  u64  tMonoUs
 8  u32  upS                 pilot uptime
12  i16  cpuCentiC           -32768 = unknown
14  u16  battMilliV          0xFFFF = NOT MEASURED (0 is a real reading of a dead
                             pack, so the sentinel is never 0)
16  u8   picoLink            0 down, 1 up, 2 up-but-silent
17  u8   picoArmed           0/1/2 unknown
18  u8   pilotMode           0 manual, 1 look, 2 drive
19  u8   deadman             0 live, 1 soft, 2 dead, 3 estop latched
20  u8   lidarHealth         0/1/2/255
21  u8   lidarSpinning
22  u8   armEpoch
23  u8   controlHolder       0 nobody, 1 you, 2 another viewer
24  u32  loopWorstUs         worst control-tick duration since the previous BOARD
28  u32  loopLateCount       ticks that missed their deadline, saturating, since boot
32  u32  picoSilentMs        carlink::silentForMs(); 0xFFFFFFFF = no link
36  u32  revolutions         session total
40  u32  timeouts            session total grab() timeouts
44  u32  txDroppedFrames
48  u32  rxControl
52  u32  rxControlStale
56  u32  ipv4                wlan0, 0 = unknown
60  u32  encodeAvgNs         what serving THIS viewer cost per frame
64  u32  encodeMaxNs
68  u8   wifiNameLen         <= 32
69  u8   clients
70  u16  reserved0
72  u8[wifiNameLen] wifiName (NetworkManager connection name), pad to 4

len <= 104, frame <= 120 bytes
```

**The passphrase is not a field and never will be.**

`encodeAvgNs`/`encodeMaxNs` and `loopWorstUs`/`loopLateCount` are the cost claim of §9 made readable off the running system. A performance claim nobody can read off the running system is the same species of bug as a test that measures nothing.

**One obligation on the pilot:** the dashboard's JSON (`/tmp/bibo-pilot.json`, `app/main.cxx:942`) and the `BOARD` frame must be filled from the **same struct in the same tick**, so the phone and the viewer can never disagree about what the car thinks. (The dashboard and its JSON have since been removed; the same rule now binds the pilot's once-a-second console line and the `BOARD` frame, both read from `Snapshot` in `app/main.cxx`.)

### `LIDAR_INFO` — 0x13, board→viewer, TCP, on connect and on change

```
 0  u16  model
 2  u8   fwMajor
 3  u8   fwMinor
 4  u16  hwRev
 6  u16  reserved0
 8  u8[16] serial      raw bytes; the viewer renders the 32 hex digits
24  u32  baud
28  u32  reserved1

len = 32
```

### `EVENT` — 0x14, board→viewer, TCP — the prose channel

```
 0  u64  tMonoUs
 8  u8   severity        0 info, 1 warn, 2 error
 9  u8   code
10  u16  textLen         <= MAX_EVENT_TEXT (200)
12  u16  droppedSince    events suppressed by the rate limit since the last EVENT
14  u16  reserved0
16  u8[textLen] ASCII, pad to 4
```

Rate-limited to 10/s, and `droppedSince` says how many were suppressed, so the viewer knows events were dropped rather than believing it saw them all.

Every `lidar::reason()`, every `carlink::detail()`, every refusal sentence the board already writes for a person travels here **verbatim**. A binary protocol that drops the sentences and keeps only the codes is how a project loses the one thing that makes a fault diagnosable, and `link.hxx`'s distinction between `RESULT_NO_PORT`, `RESULT_DENIED` and `RESULT_BUSY` — "unplugged is not broken", because each sends a person to a different place — is exactly the value that lives in the sentence.

### `CTLSTATE` — 0x15, board→viewer, **UDP**, 20 Hz — the honesty line

This is Design 1's `A` line, and every field is what the board **did**, never what it was asked. It is on UDP because it is the answer to a UDP question, and because the one message that tells the operator "the car heard you" must not be able to queue behind a 2.5 KB scan.

```
 0  u64  tMonoUs
 8  u32  ackSeq           the CONTROL seq most recently APPLIED; 0 = none yet
12  u32  controlAgeMs     since that CONTROL was applied; 0xFFFFFFFF = never
16  i16  steerNowMilli    where the wheels ACTUALLY are (drive::State::steerNowMilli)
18  i16  throttleMilli    what was SENT to the Pico this tick
20  u16  escUs            the pulse actually commanded; 0xFFFF = unknown
22  u16  neutralInMs      ms until CONTROL_STALE_MS fires; 0 = it already has
24  u16  disarmInMs       ms until CONTROL_DEAD_MS fires; 0 = it already has
26  u8   armed
27  u8   armEpoch
28  u8   deadman          0 live, 1 soft, 2 dead, 3 estop latched
29  u8   refuse           WHY throttle is not being applied; 0 = it is
30  u8   holder           0 nobody, 1 you, 2 another viewer
31  u8   pilotMode
32  u32  scanAgeMs        since the last revolution reached the board
36  u32  picoSilentMs
40  u32  lastCmdId        newest COMMAND applied

len = 44, frame = 60 bytes
```

`steerNowMilli` is the **actual**, not the target, because `docs/firmware-api.md:150` says in as many words that anything reporting what the car is doing must read the actual, and that a UI following the target draws a wheel before the car has turned it.

**`refuse` is the fix for the "one wrong byte bricks a healthy car" problem.** `assumedMode` and `armEpoch` both refuse throttle when they disagree, which is correct — but a refusal whose only symptom is "throttle dead, steering fine" looks like an ESC or Pico fault, and the ESC and the Pico are where a person goes looking first. So the reason is a named field twenty times a second:

```cpp
enum class Refuse : UInt8
{
    REFUSE_NONE = 0,
    REFUSE_NOT_ARMED,
    REFUSE_DEADMAN_SOFT,
    REFUSE_DEADMAN_DEAD,
    REFUSE_ESTOP,
    REFUSE_EPOCH,        // the viewer is asserting an arm generation that is over
    REFUSE_MODE,         // assumedMode disagrees with the board's actual mode
    REFUSE_NOT_HOLDER,
    REFUSE_PICO_DOWN,
    REFUSE_NO_UDP,
};
```

`neutralInMs` and `disarmInMs` are the deadman's own countdown, computed by the **same pure function that will do the tripping** (§6), so the viewer can render "neutral in 90 ms" during a hiccup and the number cannot disagree with the mechanism.

### `CMDACK` — 0x16, board→viewer, TCP, exactly one per `COMMAND`

```
 0  u32  cmdId          echoes COMMAND.cmdId
 4  u8   verb
 5  u8   result         0 ok, 1 refused, 2 unknown verb, 3 not permitted in this
                        state, 4 the Pico did not answer
 6  u16  textLen
 8  u8   armEpoch       the epoch in force AFTER this command
 9  u8   reserved0
10  u16  reserved1
12  u8[textLen] ASCII, pad to 4
```

### `CONTROL` — 0x40, viewer→board, **UDP**, 20 Hz

```
 0  u32  sessionId       from WELCOME
 4  u32  seq             strictly increasing from 1; at 20 Hz a u32 lasts 6.8 years
 8  u64  tMonoUs         the VIEWER's clock, only ever compared with itself
16  i16  steerMilli      -1000..1000
18  i16  throttleMilli   -1000..1000
20  u16  buttons         b0 ESTOP, b1 ENABLE, b2 MOTOR_WANTED; b3..b15 zero
22  u8   armEpoch        the epoch the viewer BELIEVES it is armed under
23  u8   assumedMode     what the viewer believes is active

len = 24, frame = 40 bytes
```

Sent **every 50 ms unconditionally**, changed or not, for as long as this viewer holds the slot. A protocol that sends control on change makes "nothing changed" and "the link died" the same wire event, which is the exact failure this project names as its recurring bug class. The constant stream is what makes silence mean something — there is no separate heartbeat, because a separate heartbeat is a thing that can keep beating while the control path is dead.

### `COMMAND` — 0x41, viewer→board, TCP

```
 0  u32  sessionId
 4  u32  cmdId       viewer-monotonic, never 0
 8  u8   verb
 9  u8   arg0
10  u16  arg1
12  u16  arg2
14  u8   armEpoch
15  u8   reserved0

len = 16
```

Verbs: `1 ARM`, `2 DISARM`, `3 ESTOP`, `4 CLEAR_ESTOP`, `5 MOTOR_ON`, `6 MOTOR_OFF`, `7 SET_MODE` (arg0 = 0 manual / 1 look / 2 drive), `8 SET_ESC_LIMITS` (arg1 = min µs, arg2 = max µs), `9 SET_SERVO_LIMITS` (arg1 = min µs, arg2 = max µs), `10 SET_SERVO_TRIM` (arg1 = centre µs), `11 SET_SLEW` (arg0 = 0 both / 1 steer / 2 throttle, arg1 = µs per 20 ms tick).

**The tuning verbs — 8, 9, 10, 11 — are refused while the car is armed**, with `result = 3` and a sentence saying so. They exist because the car's trim used to live in a hub that is gone: the steering's end stops and centre, the throttle's working range, and how fast either output is allowed to move. Re-tuning the range a live throttle is being clamped to is the only way this can hurt somebody, and the car is disarmed by default, so the rule costs an operator nothing.

They are **not** `CONTROL` fields. A limit is set deliberately and once; carrying it twenty times a second in a stream whose purpose is repetition would make an accidental slider drag indistinguishable from the operator's intent, and would lose the acknowledgement that says which value the car actually took.

Units are microseconds of pulse everywhere except `SET_SLEW`, whose µs-per-tick becomes µs/s at 50 ticks a second — and a lock-to-lock *time*, which is the unit an operator thinks in and the one a viewer should show beside the slider. **None of these survive a reboot**: the Pico holds them in RAM, and `firmware/lib/chassis/cal.hxx` is the file that survives a reflash. A viewer that lets somebody tune for an hour without saying that is a viewer that loses their afternoon.

On TCP because these change state **once** and must not be lost. Every verb is answered by exactly one `CMDACK`. `ESTOP` exists *both* here and as a `CONTROL` button bit, so it survives either transport failing.

Discrete acts are not fields of `CONTROL`, because a deliberate act must not be carried twenty times a second by a stream whose whole purpose is to be repeated.

### `SUBSCRIBE` — 0x42, viewer→board, TCP

```
0  u32  sessionId
4  u32  typeMask       bit per telemetry type
8  u16  scanDivisor    1 = every revolution, 3 = every third
10 u16  reserved0

len = 12
```

The **viewer**, not the board, decides what it can afford. This is also how the camera lands later without a redesign: it is a bit nobody sets today. The board never sends a type the mask did not claim.

### `DESCRIBE` / `SCHEMA` — 0xF0 / 0xF1

```
DESCRIBE  0  u8 type (0 = all) | 1 u8 reserved | 2 u16 reserved      len = 4
SCHEMA    0  u16 textLen | 2 u16 reserved | 4 ASCII, pad to 4
```

### Reserved, defined now, not sent in v1

```
CAMERA   0x20   0 u64 tMonoUs | 8 u32 frameIndex | 12 u16 width | 14 u16 height
                16 u8 codec (1 = JPEG) | 17 u8 flags | 18 u16 reserved0
                20 u32 byteLen | then byteLen bytes, pad to 4

POSE     0x21   0 u64 tMonoUs | 8 i32 xMm | 12 i32 yMm | 16 i32 headingMilliRad
                20 u32 sigmaXyMm | 24 u32 sigmaHeadingMilliRad | 28 u8 source
                29 u8 valid | 30 u16 reserved0                        len = 32

PATH     0x22   0 u64 tMonoUs | 8 u32 seq | 12 u16 count | 14 u16 reserved0
                then count x { i32 xMm, i32 yMm }

WAYPOINT 0x23   0 u64 tMonoUs | 8 u32 seq | 12 u16 index | 14 u16 total
                16 i32 xMm | 20 i32 yMm | 24 u16 flags | 26 u16 reserved0
                28 u32 reserved1                                       len = 32
```

`WAYPOINT` is its own type and carries **one waypoint, indexed** — it is not `PATH` with a `kind` byte. The operation a person actually performs on waypoints is nudging one, and folding them into `PATH` makes editing waypoint 7 a resend of the whole set.

**An unknown type is skipped by `len` and counted, never fatal.** That is what the length prefix is *for*, and it is the whole extensibility story in one sentence.

---

## 6. Control and the deadman

### Modes — defined by *who writes steer and throttle*

- **MANUAL (0)** — the `CONTROL` datagram's `steerMilli`/`throttleMilli` go to the Pico, subject to arm state and the deadman.
- **LOOK (1)** — the autonomy runs and publishes `DECIDE` every revolution, and throttle is forced to 0 at the point it would reach `proto::escUs`. Not "the autonomy is off": the autonomy is on and being watched, which is the whole point of the mode. This is the existing `--dry`.
- **DRIVE (2)** — the autonomy writes steer and throttle. The `CONTROL` datagram is the operator's **consent**, not their input: its stick values are ignored, and its `ENABLE` bit and its cadence are not.

Mode changes only through `COMMAND SET_MODE` on TCP, answered by `CMDACK`. Refused with `result = 3` and a sentence when the car is armed and moving — a mode change is a change of who is driving, and doing it at speed is a decision that deserves a stop first.

### Ordering, and the handoff to the tick

The board keeps the highest applied `CONTROL.seq` **per session**. A datagram with `seq <= highest applied` is discarded and counted in `rxControlStale` — never applied, never queued. Newest wins; retransmitting a stale command is worse than dropping it, which is the same conclusion `docs/conventions.md:150` already reached for the Pico link.

Comparison is `static_cast<Int32>(a - b) > 0` on the `UInt32`s, so a restarted viewer that begins again at 1 compares correctly rather than freezing the board on a huge stale seq. The high-water mark is **reset by the handshake**, per session — not by a heuristic. The obvious implementation is the bug: a board that kept a high-water mark across a reconnect would silently ignore every command from a viewer that restarted at `seq = 1`, while the socket looked perfect.

**Handoff to the tick, with no lock.** The UDP receive thread writes the decoded `CONTROL` into `slot[seq & 1]`, then stores `seq` with release ordering. The 50 Hz tick loads `seq` with acquire, reads the slot, re-loads `seq`, and retries if it moved. A seqlock over two 24-byte slots: the tick never takes a mutex, never allocates, and can never read a half-written command. The socket layer's entire contribution to the control loop is one atomic load.

### `armEpoch` — the mechanism that makes recovery safe

`armEpoch` is a `UInt8` the **board** owns. `COMMAND ARM` arms and reports the epoch under which it did, in `CMDACK` and in every subsequent `CTLSTATE` and `BOARD`. Every `CONTROL` carries the epoch the viewer believes.

**A `CONTROL` whose `armEpoch` does not match the board's current epoch has its THROTTLE ignored. Steering is still applied.** The refusal is `REFUSE_EPOCH` in `CTLSTATE` and counts in a board counter.

The board increments the epoch on: **a deadman disarm, an e-stop, a Pico link loss, a control-slot change, and its own restart.**

This is the one failure most likely to injure someone, and it is the one a level-triggered protocol would create if the epoch were not there: a four-second stall, the board disarms at 300 ms, the link returns, and the viewer is still sending `throttle = 400` twenty times a second because the operator's finger never left the key. `seq > lastApplied` does **not** catch this — the new datagrams carry higher seqs and look perfectly valid. The epoch closes it in one byte, and it closes the multi-viewer handoff case for free: when a second viewer takes the slot, the epoch bump automatically disarms the displaced one.

Recovery is deliberate: a fresh `CONTROL` stream live for `REARM_STREAM_MS` **and** an explicit `COMMAND ARM`. **Reconnecting the socket does not re-arm the car.**

### `ARM`, `DISARM`, `ESTOP`

`COMMAND ARM` is refused unless (a) estop is not latched, (b) a `CONTROL` stream from this holder has been live for `>= REARM_STREAM_MS`, and (c) the Pico link is up and answering (`carlink::isOpen()`). Each refusal carries its own sentence: you cannot arm a car you are not already holding, and the board never sends an arm into a closed port and reports success.

*Implemented 2026-09-12 in `viewfeed.cxx` (`onArm`); until then the board refused every `ARM` and `DISARM` with result 3, so the only way to arm a manual car was `--arm` on the pilot's command line.* Two refusals were added beyond (a)-(c): the pilot must be in `MANUAL` (a viewer arms only a car it is driving), and the `ARM` must carry the board's current `armEpoch`, so an `ARM` sent before something disarmed the car cannot land after it. "Up and answering" is `BOARD.picoLink == 1` - positive evidence, never "the pilot has not said yet". The arm is held **under the epoch**: `bumpEpoch()` clears it, so every event that moves the epoch - including the deadman tripping and the Pico link going down, which the board now bumps on - disarms without a second list to keep in step. The pilot's tick sends `ESC ARM` / `ESC DISARM` to the Pico on the edge, and in `MANUAL` no throttle passes without a standing arm. **The same edge engages the steering**: `SERVO ON` before that tick's `STEER`, `SERVO OFF` on disarm or when the holder leaves (`STOP` already releases it). The Pico boots with the steering pin released and its `STEER` only moves a target, so until this was added every `STEER` the pilot sent was answered `OK` and moved nothing.

`DISARM` always succeeds, including with no Pico link, and then reports `result = 4` with *"disarmed locally; the Pico did not answer, its own 400 ms deadman will stop the car"*.

**Emergency stop has two paths on purpose.** `CONTROL`'s `ESTOP` bit rides the 20 Hz UDP stream, so it arrives within one 50 ms window and needs no round trip. `COMMAND ESTOP` rides TCP and is acknowledged. Either **latches**: `proto::stop()` to the Pico that same tick, arm state drops, `armEpoch` increments, `FLAG_ESTOP` set on every outbound frame, `deadman = 3`. The latch clears **only** via `COMMAND CLEAR_ESTOP` while disarmed, and then the operator must `ARM` again. Three deliberate steps, because a stop that can be undone by releasing a key is a stop that will be undone by accident.

`MOTOR` is the **lidar's** motor, `scanwire`'s meaning, and the board obeys or refuses by the same policy `feed::Motor` already encodes — the pilot holds the lidar and refuses, with the refusal as a `CMDACK` sentence rather than silence.

### The deadman chain — timings and owners

```cpp
constexpr Int32 CONTROL_PERIOD_MS   =  50;   // L0, the viewer
constexpr Int32 CONTROL_STALE_MS    = 150;   // L1, the Pi: throttle to neutral
constexpr Int32 CONTROL_DEAD_MS     = 300;   // L2, the Pi: STOP, disarm, epoch++
constexpr Int32 REARM_STREAM_MS     = 500;
constexpr Int32 TICK_MS             =  20;
constexpr Int32 PICO_HOP_BUDGET_MS  = 100;   // Pi -> USB CDC -> Pico, measured worst case
constexpr Int32 PICO_DEADMAN_MS     = 400;   // firmware/app/main.cxx:49, NOT ours to change

static_assert(
    CONTROL_DEAD_MS + PICO_HOP_BUDGET_MS <= PICO_DEADMAN_MS,
    "the Pi's stop must beat the Pico's, or the blunt layer fires first and prints "
    "ERR deadman onto a cable nobody is watching"
);
```

That `static_assert` is the single cheapest safety artifact in this document. Every design in this competition confessed that its deadman will make the car stutter outdoors and that someone will eventually raise the constant because they were right that it felt bad. This is the only thing standing in the way of that edit, and what it protects is not the stop — it is the *diagnosability* of the stop.

**L0 — the viewer, 50 ms.** Sends `CONTROL` at 20 Hz while it holds the slot, changed or not. Not a deadman: the keepalive. Owner: the viewer process.

**L1 — bibowire on the Pi, 150 ms → SOFT.** No `CONTROL` **from the holder** applied within 150 ms and `throttleMilli` is forced to 0 and sent to the Pico **on that tick**. Steering is **held at the last commanded value**, not centred — a car that snaps to centre mid-corner changes its line at the moment it stopped being commanded — and it costs nothing, keeps the Pico's own timer fed, and is the command whose reply proves the board came back. Owner: the pilot's 50 Hz tick.

150 ms because one lost datagram must not cut the throttle: Wi-Fi drops single frames routinely and a car that stutters on every lost packet is a car nobody will drive. Three consecutive losses at 20 Hz is no longer a lossy link, it is a link that has stopped.

**L2 — bibowire on the Pi, 300 ms → DEAD.** `proto::stop()` to the Pico (neutral, disarm, release), arm state drops, `armEpoch` increments, `deadman = 2` and `FLAG_DEADMAN` on every frame. **It does not recover on its own**, because a link that came back is not the same fact as an operator who is ready. Owner: the pilot's tick.

L1 and L2 together cover everything about the **link**: the tab closing, the phone locking or backgrounding, the hotspot dropping, the laptop sleeping, the operator walking out of range. None of those are visible to the car by any other means.

**L3 — the pilot's own loop, unchanged.** No revolution within `REV_WAIT_MS = 200` → the decision is `MODE_BLIND`, which is a stop (`reactive.hxx` trap 1). The Pico silent beyond `carlink::Config::silenceMs = 500` → throttle held at neutral. These cover the **sensor** and the **cable**, which a viewer deadman cannot see. Owner: `app/main.cxx`, untouched by this spec.

**L4 — the Pico, 400 ms, unchanged.** `firmware/app/main.cxx:49`. It is the only layer that covers the **Orange Pi itself** hanging, a kernel stall, or the USB link dying, because nothing running on the Pi can save you from the Pi. From the Pico's side a dead viewer is indistinguishable from a live one — the Pi keeps sending `STEER` and `ESC` on its own schedule whether or not a viewer exists — which is the whole reason L1 and L2 have to exist. **Do not tighten it.**

Because the Pi commands the Pico every 20 ms in **every** state, including neutral while SOFT and `STOP` while DEAD, the Pico's deadman only ever fires when the Pi has stopped running. A last resort that fires routinely is a last resort nobody notices has fired. **This is load-bearing and is written down as such:** the day anyone makes the Pi stop transmitting while DEAD, two timers race.

### Whose silence counts

**The deadman is fed only by `CONTROL` frames that (a) carry the current `sessionId` and (b) come from the current control holder.** An observer's datagrams, a stale session's datagrams and a second viewer's datagrams are counted and discarded and **do not feed the timer**. This is the fix for Design 3's fatal flaw: without it, viewer B's stream keeps the deadman alive while viewer A — whose laptop just slept — is protected by nothing, and A's UI shows a live picture of a car it is no longer driving.

**And it is "parsed", not "received".** Bytes half-read in an accumulator are not a command, and counting them as liveness is this repo's recurring bug class applied to the safety timer.

### When there is no holder at all

If **no viewer holds the control slot**, bibowire's deadman does not apply — the pilot runs under L3 and L4 exactly as it does today with nobody watching. The moment a viewer takes the slot, in **any** mode including `drive`, its `CONTROL` cadence becomes the consent the deadman watches, and losing it stops the car.

That is a deliberate answer to the objection that an autonomous car should not stop when the operator's laptop stalls: an autonomous run *supervised* by a viewer stops when the supervisor vanishes, and an autonomous run that was never supervised was never promised a supervisor. The alternative — Design 3's rule that the deadman is unarmed in `drive` and `ESTOP` is the only stop — puts the kill switch behind the exact link that just failed.

### The deadman is a pure function

```cpp
namespace deadman
{

  enum class State
  {
      STATE_LIVE = 0,
      STATE_SOFT,
      STATE_DEAD,
      STATE_ESTOP,
  };

  struct Inputs
  {
      Int64 nowMs = 0;
      Int64 lastControlMs = 0;   // when the HOLDER's newest CONTROL was APPLIED
      Bool  haveHolder = false;
      Bool  enable = false;
      Bool  epochMatches = false;
      Bool  modeAgrees = false;
      Bool  estopLatched = false;
  };

  struct Output
  {
      State  state = State::STATE_DEAD;
      Refuse refuse = Refuse::REFUSE_DEADMAN_DEAD;
      Int32  neutralInMs = 0;
      Int32  disarmInMs = 0;
  };

  [[nodiscard]] Output step(const Inputs& in);

}
```

No clock inside it, no socket, no Pico — the shape `reactive::step` already uses, taking `dtMs` from the caller. Two consequences: the safety property is exercised on a laptop with no hardware, in microseconds rather than by sleeping, which is the only way a safety property here gets tested more than once; and **the viewer runs the identical function on its own copy of the numbers** and renders the remaining margin as a live countdown, so an operator watching "neutral in 90 ms" during a hiccup is watching the same arithmetic that will do the tripping.

`nowMs < lastControlMs` returns `STATE_DEAD`. A negative age is a bug somewhere upstream, and the safe reading of a bug is "stopped"; treating it as freshness is how a sign error becomes a car that will not stop.

### Latency budget

- **Link healthy**, operator's hand off the enable to wheels not driven: 50 ms (worst wait for the next `CONTROL`) + link latency (tens of ms) + 20 ms (next tick) + 20 ms (next servo frame) ≈ **90–120 ms**.
- **Link dead**: 150 ms to throttle zero, 300 ms to a full `STOP`. Only if the **Pi itself** dies does the Pico's 400 ms come into play — about **700 ms** worst case from the last thing that worked.

### Downstream of the board

Control becomes the existing text over `carlink`: `proto::steer(fraction)` (never `snprintf("%f")`) and `proto::escUs(us)` or `proto::stop()`. bibowire does not reinvent the Pico's protocol and **never lets a viewer address the Pico directly** — the board is the only owner of that port, which is what keeps the pilot and a manual driver from both holding the stick, and it is why `status_server.py`'s manual control (since removed) already refused to open the port while the pilot was running.

### A known hole this protocol does not fix

`docs/conventions.md:195-203` records that `THROTTLE_CAL_MIN` (1541, measured on the brushed 1060) is invalidated by the brushless 21.5T at 10.71:1, which breaks static friction at a *lower* pulse — and `firmware/app/main.cxx:1834` decides the car is being driven by `dm.escArmed && dm.escTargetUs > dm.escMinUs`. **So L4 has a blind spot: a car creeping at what the firmware calls idle is a car the last resort does not consider to be moving.** L1, L2 and L3 do not share it — they act on input silence regardless of throttle — but the fix is re-measuring on a stand, and no protocol substitutes for a calibration.

---

## 7. When things go wrong

### The viewer stops sending control (phone sleeps, tab backgrounded, process hangs)

**Board:** SOFT at 150 ms — throttle 0 to the Pico this tick, steering held, `FLAG_DEADMAN` set, `refuse = REFUSE_DEADMAN_SOFT`. DEAD at 300 ms — `proto::stop()`, disarm, `armEpoch++`. Control slot released at 1000 ms. **Viewer:** shows the countdown hit zero and the `deadman` field change; it does not infer anything from `send()` returning.

### A hotspot stall of one to five seconds

**This is the routine case and it must not be handled by the exceptional path.**

**UDP control is unaffected by TCP's queue.** The viewer keeps sending; the board keeps draining to empty and keeping only the newest `seq`. When bytes flow again the board applies the **newest** datagram and every older one is discarded unapplied and counted. There is no backlog of stale steering to apply, because there is no queue.

**TCP telemetry:** the board's per-client ring coalesces (below); at most one `SCAN` is ever queued, so the pending bytes are bounded at about 2.5 KB plus one of each vital frame, **regardless of how long the stall lasts**. Nothing is retransmitted and nothing is replayed. When the link returns the viewer receives the *current* revolution, and `droppedSinceLast` says how many it missed.

**The session is not torn down.** No reconnect, no reconnect storm, no epoch churn, no lost control slot — at the moment the link is worst. That is the property Judge 1 was right to demand of the field, and the two-transport shape gives it more strongly than a single socket could: a telemetry stall does not touch control **at all**.

**Viewer rendering during the stall:** see "staleness" below.

### A slow or malicious viewer, and the drop classes

Every telemetry type has a class, enforced on the view-feed thread against a bounded ring of **96 KiB or 12 frames, whichever fills first**:

- **`CLASS_VITAL`** (`WELCOME`, `BYE`, `EVENT`, `CMDACK`, `BOARD`, `LIDAR_INFO`, `PATH`, `WAYPOINT`, `SCHEMA`) — **never dropped**. If the vital ring fills, the **client is closed**: a viewer that cannot absorb 120 bytes of state has gone, whatever its socket says. A client whose oldest unsent vital byte is more than `BEHIND_MS = 500` old is closed — `feed.cxx:26`'s number, unchanged.
- **`CLASS_LIVE`** (`SCAN`, `DECIDE`, `POSE`) — **drop-oldest, depth 1**. The newest revolution wins and the count goes into the next `SCAN`'s `droppedSinceLast`. Additionally, **any revolution more than 200 ms old at send time is dropped before it is ever queued** — sending it would consume the bandwidth the current revolution needs, in order to show something already wrong.
- **`CLASS_BULK`** (`CAMERA`, later) — **discarded first, always, before any live or vital frame.**

`publish()` never blocks the caller: mutex, push, one byte to the self-pipe, return — `feed.hxx`'s existing rule, restated with classes. The control tick's total interaction with this protocol is one atomic acquire-load for the `CONTROL` snapshot and one `publish()` per revolution. The tick never encodes, never CRCs, never calls `send()`, and never waits for a viewer.

An inbound frame claiming `len > MAX_INBOUND_PAYLOAD` gets `BYE(TOO_BIG)` and close, with no allocation. A second `HELLO` on an existing connection is a protocol error and closes it. A fifth client gets `BYE(REFUSED)` naming the four addresses already connected.

### A half-open connection

Covered three times over, none of them relying on TCP noticing: `SO_KEEPALIVE` at 2 s / 1 s / 3; the board's `PING` every 1000 ms with `BYE(TIMEOUT)` if no `PONG` within 4000 ms; and the viewer's own redial if **no frame of any type** has arrived in 3000 ms. At 5 Hz `BOARD` and 1 Hz `PING`, silence that long is not a quiet moment. And the car stopped 2700 ms before any of that mattered.

### Reconnect

Viewer backoff **250 ms, 500 ms, 1 s, 2 s, 4 s, then 4 s forever, each with ±20 % jitter**, never giving up. Jitter because every client on the hotspot (then the viewer and the phone dashboard, since removed) comes back at the same instant when it returns, and two clients synchronised on the same schedule hammer the board in lockstep. Never giving up because outdoors the link comes back when the phone stops moving, and a viewer that stopped trying makes that recovery a manual step in a field.

On reconnect the **full handshake runs again** and a **new `sessionId`** is issued. Nothing is resumed; there is no session-resumption path in this protocol at all, and that is the point — the only state worth resuming is the live picture, which is worthless by the time the link is back. Stale datagrams from the previous session are rejected by `sessionId` and counted.

What the viewer carries across is its knowledge of `bootId`: same `bootId` means the board kept running and the operator's mental model is still valid; a different `bootId` means the pilot restarted and the viewer **clears everything** — scan, decision, arm state, mode, `revIndex` baseline — before drawing a single point.

**The car during a reconnect: it stopped at DEAD and it stays stopped, and the `armEpoch` changed when it did.** The viewer shows "board reconnected — disarmed" and the operator arms deliberately. A car that resumes the last stick position when the link returns after four seconds is the failure mode that puts a car into a fence.

### An unknown message, an unknown flag, an unknown version

- **Unknown type:** skipped by exactly `len`, counted. Never fatal, never a disconnect. This is what lets an older viewer keep driving a newer board.
- **Unknown annotating flag (bits 3–15):** ignored.
- **`FLAG_MORE` in v1:** the frame is **refused** — a flag that changes framing may not be ignored.
- **`ver` higher than known:** prefix read, tail ignored.
- **`ver` lower than known:** missing fields marked **absent** via their sentinels, never fabricated.
- **Version mismatch (major):** `BYE(VERSION)` with a sentence naming **both numbers, both build stamps and the action**, before any body is interpreted. The viewer displays the sentence.

### A corrupt or truncated frame

- **Truncated:** `TAKE_NEED_MORE`, consuming nothing. There is no path that yields a short message.
- **CRC failure on TCP:** something worse than corruption has happened on a checksummed stream — `BYE(BAD_CRC)`, close, and an `EVENT`.
- **CRC failure on UDP:** datagram dropped and counted; no state change.
- **Junk in the stream:** deterministic resync, with the skipped byte count reported and counted — never silent.
- **`SCAN` whose `count` disagrees with `len`:** refused outright.

### Wrong service, and a person with `nc`

A TCP connection whose first two bytes are not `0x42 0x57` gets **one plain ASCII line** before its `BYE`:

```
ERR bibowire v1 binary on 8020; scanwire text is on 8011; run `biboctl watch` on the board to read this port
```

The moment a person is most confused by a binary port is the moment they connect to it by hand, so that is the one case that answers in words.

### Two viewers both want to drive

The second is accepted as an observer with `refusal = 2` and a sentence naming who holds the slot and since when. Its `CONTROL` is counted and discarded and does not feed the deadman. There is never a moment when two `seq` streams are applied to one car, and there is never a moment when one operator's protection is being fed by another's keyboard.

### UDP blocked, TCP fine

Caught within 1000 ms of `WELCOME` by the datagram-count probe, reported as a sentence naming the symptom, and **routed around** by the TCP control fallback with a persistent degraded banner. This is the nastiest symptom the two-transport choice creates, and it is the one case where being able to keep driving matters more than transport purity.

### The lidar dies while the link is fine

`SCAN` stops. `DECIDE` continues every tick with `revIndex = 0` and `mode = BLIND`, which is a stop, exactly as the pilot already publishes a `D` line with no `F` behind it. `CTLSTATE.scanAgeMs` climbs through 200, 500, 2000; `BOARD.lidarHealth` and `timeouts` say why; after `LIDAR_LOST_TIMEOUTS` (10, ≈ 2 s) the pilot marks the run failed as it already does. **The link stays green and the scan goes away** — which is the truth. A link that is healthy must never make a dead sensor look alive.

### The Pico cable comes out

`BOARD.picoLink = 0`, `picoSilentMs = 0xFFFFFFFF`, `CTLSTATE.refuse = REFUSE_PICO_DOWN`, an `EVENT` carrying `carlink::detail()` verbatim, and `armEpoch++`. `ARM` is refused with `no_link` rather than sent into a closed port and reported as success. The pilot retries the port once a second, quietly, as it already does. The Pico's own 400 ms fires independently.

### Staleness, on the viewer

Two different lies are possible and each needs its own mechanism.

**Lie 1 — the link stalled and the last picture is still on screen.** Caught by the viewer's own clock against the arrival of any frame. Because `CTLSTATE` arrives every 50 ms on UDP and `BOARD` every 200 ms on TCP, silence is unambiguous and fast.

**Lie 2 — the link is perfect and the data behind it is dead.** Caught by the **board's own ages**, carried at 20 Hz: `scanAgeMs`, `picoSilentMs`, `controlAgeMs`. All three are the board measuring itself, not the viewer inferring.

**Age is never smaller than the arrival floor.** The viewer maintains a clock offset from the last 16 `PING`/`PONG` round trips, taking the **minimum** RTT (on a hotspot the mean is dominated by stalls; the minimum is the closest thing to the true path) and half of it as the one-way delay. It then computes two ages — one from the offset, one from local arrival — and uses the **larger**. A clock offset that drifts optimistic can only ever be corrected upward by the fact that the bytes have not arrived yet. This is the one place the design deliberately distrusts its own cleverness.

**Three rendering rules, and the third is the one that matters:**

- `ageMs <= 400` — drawn normally.
- `400 < ageMs <= 1500` — drawn desaturated, with the age in milliseconds printed over it.
- `ageMs > 1500` — **not drawn**. The panel shows `no scan for 3.2 s` on an empty background.

A greyed-out picture is still a picture, and people read pictures as current no matter what colour they are. **Absence is the only rendering a person cannot misread.**

**And the viewer cannot draw a stale value by forgetting to check**, because there is no value to draw:

```cpp
[[nodiscard]] Opt<Revolution> revolution(Int64 nowMs) const;
[[nodiscard]] Opt<Decision>   decision(Int64 nowMs) const;
[[nodiscard]] Opt<BoardState> board(Int64 nowMs) const;
```

The staleness test is **inside the accessor** and the empty case has no points in it. This is the same argument `lidar.hxx` already makes for emptying `out` on a failed `grab()` — "a caller that ignores the Bool and hands `out` to `reactive::step` gets `STATUS_BLIND` and a stopped car, where a stale revolution would get a car confidently driving on what the room looked like a second ago" — applied to the viewer, where the consequence is a person's judgement rather than a controller's.

**Gaps are counted, not smoothed.** `revIndex` is monotonic and `droppedSinceLast` says how many the sender discarded. The viewer shows `revolutions 4412-4419 missing`, never an interpolated sweep between two revolutions a second apart. A smooth animation over missing data is a lie the eye cannot detect.

### The frame-header ring

The board keeps the last **256 frame headers per direction** — type, len, seq, tMonoUs, CRC verdict — in a fixed 4 KiB array, dumped by `biboctl trace` and written to the journal on any abnormal close. A disconnect can be post-mortemed on the board with `journalctl -u bibo-pilot` from a phone, which is the actual field situation.

---

## 8. Debugging in a field, with only a phone

Binary takes `nc bibobox.local 8011 | head` away from the viewer path, and `scanwire.hxx` names that property as a reason it is text. Here is what pays for it, in things that ship rather than intentions.

**1. The text feed stays, untouched. This is the real answer.** `scanfeed` on TCP 8011, `feed.cxx`, `scanwire`'s 79 checks, `/tmp/bibo-scan.txt` and the relay chain are all unchanged, because the phone dashboard is a client of them and is explicitly out of scope (the dashboard has since been removed). So from a phone with Termux or JuiceSSH over the hotspot:

```
nc bibobox.local 8011
```

still prints `INFO`, `HEALTH`, `MOTOR`, and then `F` and `D` lines, ten a second, in units a human holds — 9000 is 90 degrees, 1240 is 1.24 metres, `-457` is a bit less than half left. **The board ends up with a machine path and a human path, and that is a defensible division rather than a regression; the mistake would be deleting the human one.**

**2. `http://bibobox.local/dash` has since been removed.** It showed the live scan, the pilot's decision, the board's state and the manual driving controls to a phone walking behind the car; from a phone, 1 and 3 are what remain.

**3. `biboctl`** — one static aarch64 binary at `/usr/local/bin/biboctl`, installed by `firmware/pilot/tools/status/install.sh` alongside the scan feed, and **built from `bibowire.cxx`, the same object file the pilot links**. It cannot describe a layout the board does not use.

```
biboctl watch          connect to 127.0.0.1:8020 as a real viewer, print ONE LINE PER
                       FRAME in scanwire's own vocabulary:
                         F 512 10123 0,3410,47 70,3388,47 ...
                         D cruise 3410 12 -120 350 0
                         B up=812 cpu=54.2 pico=up mode=drive deadman=live batt=n/a
                         C ack=8814 age=41ms steer=-120 esc=1602 neutral-in=109ms
biboctl watch --raw    decoded headers + 16-column hexdump with offsets + CRC verdict
biboctl drive          a keyboard client sending real CONTROL datagrams at 20 Hz with
                       ENABLE held - the car can be driven, armed, stopped and its
                       deadman exercised from a phone over ssh with no laptop anywhere
biboctl send ARM       one-shot COMMANDs, printing the CMDACK sentence
biboctl schema         the frame catalog (below)
biboctl trace          the board's frame-header ring
biboctl bench          codec cost in nanoseconds per frame
```

The text protocol does not die; it stops being what the machines send and becomes how a person reads what they sent. The `F` and `D` lines `biboctl watch` prints are **byte-identical** to `scanwire::formatFrame`'s, because it calls that function.

**4. `describe()` lives in the shared module**, so `biboctl`, the viewer's log pane, the pilot's console and **the test failure output** all render a frame the same way, and it cannot drift from the codec because it sits beside it.

**5. A self-describing catalog that cannot drift.** `bibowire.cxx` holds a `constexpr Array<FieldDesc, N>` — type tag, name, version, field offsets, widths, units — and **both** the decoder's bounds checks **and** the `DESCRIBE`/`SCHEMA` response are generated from it. Ask any board what it speaks and it answers in ASCII:

```
0x10 SCAN v1 24+4n+n : u64 tMonoUs us ; u32 revIndex ; u16 freqMilliHz mHz ; u16 count ; u8 health ; ...
```

A `static_assert` that the table covers every enumerator, plus a runtime check in the test, means a message added without a catalog entry **does not compile**. This kills the classic binary-protocol failure, which is not unreadability — it is a document that drifted from the build. `SCHEMA` also carries the board's build stamp, and `install.sh` refuses to install a `biboctl` whose stamp does not match the pilot's, because the whole field-debugging argument rests on that binary being present and current.

**6. The wrong-protocol dialer gets a sentence**, not a hex dump (§7).

---

## 9. What it costs

### Bytes on the wire, one viewer, 500 points at 10 Hz

**Board → viewer**

| | size | rate | |
|---|---|---|---|
| `SCAN` | 2540 B | 10 Hz | 25.4 KB/s |
| `DECIDE` | 40 B | 10 Hz | 0.4 KB/s |
| `BOARD` | ≤ 120 B | 5 Hz | 0.6 KB/s |
| `CTLSTATE` (UDP) | 60 B | 20 Hz | 1.2 KB/s |
| `PING` | 32 B | 1 Hz | 0.03 KB/s |
| | | **total** | **≈ 27.6 KB/s ≈ 221 kbit/s** |

**Viewer → board:** `CONTROL` 40 B × 20 Hz = 0.8 KB/s, plus `PONG` — **≈ 0.83 KB/s ≈ 6.6 kbit/s**.

A ten-minute outing is about 17 MB, all of it local to the hotspot; none of it crosses the phone's cellular uplink.

**Against what exists:** a real 500-point `F` line measures **6513 bytes** (13.0 B/point), so `scanwire` today is ≈ 65 KB/s of scan alone with nothing at all upstream. bibowire carries strictly more — board state, control echo, events, acks, and a control channel that did not exist — in **less than half the bytes**.

The compaction comes from three places: a point costs 5 bytes instead of ~13 ASCII characters, there are no field names on the wire, and nothing is a decimal string. Angles at centi-degrees and distances at whole millimetres are **lossless with respect to the device** — the C1 does not measure finer — so this is not a precision compromise.

**What was deliberately not taken:** delta-encoding the angle and varint-coding the distance would reach ≈ 1.2 KB per revolution, another 2×. Refused, because a fixed 4-byte stride means point *i* is at `24 + 4*i` with no decode pass, and — the real reason — a single corrupted byte in a varint stream shifts every subsequent point, producing **a scan that looks like a plausible room in the wrong place**. A fixed stride corrupts one point. On a car that steers by what it sees, a wrong-but-plausible picture is the most expensive failure available, and 1.3 KB per revolution is not worth buying it.

At 221 kbit/s the link is not the constraint; the seconds-long stalls are, and no encoding fixes those. What the halved size actually buys is the size of the burst that has to drain when a stall ends — and the drop classes bound that at one scan plus one of each vital frame regardless of how long the stall lasted.

### CPU on the Pi

The budget to beat is stated: the HTTP dashboard (since removed) cost about **1.4 %** of one core with one viewer, and the control loop must not miss its 20 ms deadline.

**Per revolution:**

- Point array: a **500-iteration convert-and-round loop** (`Float32` degrees → `UInt16` centi-degrees, `Float32` mm → `UInt16` mm), ~10–15 cycles each ≈ **3 µs** at 2.4 GHz. This document does not claim it is a `memcpy`: `reactive::Ray` is two floats.
- Quality array: a true `memcpy` of 500 bytes ≈ **0.1 µs**.
- 24 bytes of header written by six helpers ≈ negligible.
- CRC32C over 2524 bytes: the RK3588's Cortex-A76 has the instruction, ~8 B/cycle ≈ **0.13 µs**; the table fallback ≈ 1 µs.

**≈ 3.5 µs per revolution, 35 µs/s at 10 Hz.**

**Per second, everything else:** 20 × (`recvfrom` of 40 B + six loads + a seqlock store), 20 × `CTLSTATE` encode, 5 × `BOARD` encode, one `PING`. All small-constant work on fixed-size buffers. Syscalls: ~35 TCP writes/s plus ~20 `sendto`/s, at 5–10 µs each ≈ **0.5 ms/s**.

**Total: well under 0.1 % of one core, and the dominant term is the kernel's socket path rather than anything this protocol does.** Roughly a fifteenth of the dashboard's cost.

**The honest correction to the usual binary argument.** It is *not* true that binary framing deletes 15,000 integer-to-ASCII conversions per second from this board. `app/main.cxx:804-809` formats the `F` line and writes it to `SCAN_FILE` every tick for the phone dashboard, which is out of scope and not changing, so **those conversions are a sunk cost that stays**. (The dashboard and `SCAN_FILE` have since been removed; the pilot still formats the `F` line every tick for the text feed on 8011, so the conversions still stay.) bibowire's encode is **additive**: about 35 µs/s on top. It is worth paying because it halves the bytes in flight during a stall, it moves the *parse* to the laptop, and it makes the frame structure something a bounded decoder can reject in four rules — not because it deletes work the board was doing.

**And the cost is measured, not asserted.** `BOARD.loopWorstUs` and `loopLateCount` carry the worst control-tick duration and the running count of missed deadlines; `BOARD.encodeAvgNs`/`encodeMaxNs` carry what serving this viewer cost per frame; the pilot's existing exit summary (`viewer cost per tick avg %.0f us, max %.0f us`, `app/main.cxx:1000`) is extended to cover bibowire; and `biboctl bench` encodes and decodes 10,000 synthetic revolutions on the board itself and prints nanoseconds per frame, so a codec regression is a number someone can produce in the field.

**Memory:** one 1400 B UDP receive buffer, one 96 KiB outbound ring per client (≤ 4), one preallocated encode buffer, a 4 KiB frame-header ring. Nothing per-revolution, nothing that grows, no heap in the send or receive path.

---

## 10. How the camera, a pose and a path arrive later

Four mechanisms, each already exercised by something in v1 so none is theoretical.

**1. The length prefix.** An unknown type is skipped by `len` and counted. One `UInt32` per frame, forever, and it is the entire compatibility story.

**2. The per-type `ver` byte.** `SCAN` can reach v2 while `DECIDE` stays v1. A trailing field added under a higher `ver` reads as "not present" to an old reader — which works only because every body is fixed-width up to the point the new field begins, and because **every optional quantity has an explicit absent sentinel**. Absence is representable everywhere, so a new field never has to be faked by a zero.

**3. Reserved types, laid out now** (§5). Writing the layouts before anyone implements them is what stops the first camera frame from being the thing that forces a header change: a 24-byte camera header with `byteLen` and a codec tag fits under the same 12-byte frame header and the same 256 KiB bound, and `FLAG_MORE` is already reserved for the day a frame needs fragmenting.

**4. `SUBSCRIBE`.** The camera arrives **switched off**; a viewer that wants it asks, and can ask for `scanDivisor = 3` in the same breath so a second watcher costs a third of the scan bandwidth instead of doubling it.

**And the drop classes are what make the camera safe to add at all.** `CAMERA` is `CLASS_BULK` and is discarded before any scan or state frame, so the day someone points a 640×480 JPEG stream down this socket, the thing that degrades is the camera and **not the car's picture of the world**. No other design in this competition had a priority ordering across message kinds, and it is the reason this one was chosen.

Concretely:

- **A camera:** `CAMERA` (0x20), `CLASS_BULK`, subscription bit, codec tag **echoed on every frame** so a capture is self-describing. CRC covers it like everything else — 40 KB at 30 fps is 1.2 MB/s of CRC, which at the hardware instruction is 0.006 % of one core, so there is no exemption and no special case. At ~200 KB/s it does not fit alongside the scan on this hotspot; `SUBSCRIBE`'s `scanDivisor` is how a viewer says which it wants, and that is a fact about the link rather than about the protocol.
- **A SLAM pose:** `POSE` (0x21), 48 bytes on the wire at 10–20 Hz, `CLASS_LIVE` — a pose is a statement about now, and a lost one is replaced in 100 ms.
- **A planned path:** `PATH` (0x22), `CLASS_VITAL`, sent on change rather than per tick. It is state that must arrive complete and stay correct, which is exactly why it rides the reliable transport instead of needing an ack-bitmap retransmit scheme invented on top of UDP.
- **Waypoints:** `WAYPOINT` (0x23), one per frame, indexed, `CLASS_VITAL`, so editing waypoint 7 is one 48-byte frame rather than a resend of the set.
- **Battery:** already a field, already shipping as `0xFFFF` = not measured. The day the sensor is fitted, one value changes on the board and **zero lines change in the viewer**.

**Two rules written down now, because a future maintainer will be tempted by each.**

- **No prose field ever becomes a payload channel.** `EVENT`, `CMDACK`, `BYE` and `SCHEMA` carry sentences for people. The shortcut of stuffing structured data into one of them, to avoid defining a type, is forbidden — define the type; that is what the tag space is for.
- **Nothing self-describing goes in a body.** No TLVs, no key-value pairs, no optional fields. Self-describing bodies are how a binary protocol grows a parser that allocates, branches, and eventually has a bug that only shows up on the board. Fixed-width bodies plus a version byte plus a length prefix cover every extension this car will actually need, and they keep the decoder a function you can read in one sitting.

**What would force a redesign, honestly:** a payload needing ordered reliable delivery at high rate — a continuous lossless point cloud for offline mapping. That belongs on the board's SD card and a post-run copy, which is what `docs/conventions.md` already says about telemetry, not on the field link.

---

## 11. The C++ shape

### Files

| file | what |
|---|---|
| `firmware/pilot/src/bibowire.hxx` / `.cxx` | **the shared module.** Pure: no sockets, no clock, no device. Added to `pilotlib`'s source list. Compiled into the pilot **and** the viewer, so the two cannot drift — the rule `scanwire.cxx` and `reactive.cxx` already follow. |
| `firmware/pilot/src/viewfeed.hxx` / `.cxx` | the board's socket half: TCP server, UDP socket, per-client rings, drop classes, frame-header ring. New code, ~700 lines, shaped on `feed.cxx` but separate from it — `feed.cxx` moves **lines** for the dashboard and must keep doing exactly that (the dashboard has since been removed; the hub and `nc` still read those lines). |
| `firmware/pilot/tools/biboctl.cxx` | the field tool (§8). Links `pilotlib`. |
| `firmware/pilot/tests/test_bibowire.cxx` | the pure suite. **In `verify.bat`.** |
| `firmware/pilot/tests/build_bibowire_test.bat` | copied from `build_scanwire_test.bat`, one source path changed. |
| `firmware/pilot/tests/test_viewfeed.cxx` | the socket suite. **ctest on Linux only, NOT in `verify.bat`** — the same reason `test_feed` is excluded (`pilot/CMakeLists.txt`): on MSVC every socket call refuses, so a `.bat` could only prove `start()` returns false. Said out loud rather than discovered. |

**Build wiring, exactly:**

```cmake
add_library(pilotlib STATIC
    src/proto.cxx src/link.cxx src/autonomy.cxx src/reactive.cxx src/lidar.cxx
    src/scanwire.cxx src/feed.cxx src/bibowire.cxx src/viewfeed.cxx)

foreach(t proto pilot reactive scanwire feed bibowire viewfeed)
```

```bat
call :suite bibowire "%HERE%pilot\tests\build_bibowire_test.bat"
```

A suite that is not in that list is a suite that does not exist.

### What the module exposes

Written to this repo's conventions: Pascal-case aliases from `shared/shared.hxx`, no bare `int`/`float`, `[[nodiscard]]` where the answer can be found no other way, enum members `SCREAMING_SNAKE` prefixed with their enum's name, `#pragma once`, namespace brace on its own line with a two-space-indented body, named casts only, **no parameter list wraps** — which is why the encoder takes a `Head` and a `Body` rather than eight arguments.

```cpp
#pragma once

#include "shared.hxx"

namespace bibowire
{

  constexpr UInt16 PORT = 8020;
  constexpr UInt16 PROTO_MAJOR = 1;
  constexpr UInt16 PROTO_MINOR = 0;
  // ... the rest of the constants from section 3

  enum class Type : UInt8
  {
      TYPE_HELLO = 0x01,
      TYPE_WELCOME = 0x02,
      TYPE_BYE = 0x03,
      TYPE_PING = 0x04,
      TYPE_PONG = 0x05,
      TYPE_LEAVE = 0x06,
      TYPE_SCAN = 0x10,
      TYPE_DECIDE = 0x11,
      TYPE_BOARD = 0x12,
      TYPE_LIDAR_INFO = 0x13,
      TYPE_EVENT = 0x14,
      TYPE_CTLSTATE = 0x15,
      TYPE_CMDACK = 0x16,
      TYPE_CAMERA = 0x20,
      TYPE_POSE = 0x21,
      TYPE_PATH = 0x22,
      TYPE_WAYPOINT = 0x23,
      TYPE_CONTROL = 0x40,
      TYPE_COMMAND = 0x41,
      TYPE_SUBSCRIBE = 0x42,
      TYPE_DESCRIBE = 0xF0,
      TYPE_SCHEMA = 0xF1,
  };

  enum class Class
  {
      CLASS_VITAL = 0,
      CLASS_LIVE,
      CLASS_BULK,
  };

  enum class Take
  {
      TAKE_FRAME = 0,
      TAKE_NEED_MORE,
      TAKE_RESYNC,
      TAKE_TOO_BIG,
      TAKE_BAD_FLAG,
  };

  struct Head
  {
      Type   type = Type::TYPE_PING;
      UInt8  ver = 1;
      UInt16 flags = 0;
      UInt16 seq = 0;
  };

  // Borrowed, not owned: `bytes` points INTO the caller's buffer and is valid
  // only until that buffer is refilled. No allocation anywhere in take().
  struct Body
  {
      const UInt8* bytes = nullptr;
      Size         len = 0;
  };

  struct Frame
  {
      Head head;
      Body body;
  };

  // ---- framing ---------------------------------------------------------------
  [[nodiscard]] Take take(const UInt8* buf, Size len, Frame* out, Size* consumed);
  [[nodiscard]] Size put(const Head& h, const Body& b, UInt8* out, Size cap);
  [[nodiscard]] Class classOf(Type t);
  [[nodiscard]] Bool  knownType(UInt8 tag);
  [[nodiscard]] UInt32 crc32c(const UInt8* data, Size len);

  // ---- one typed pair per message, so no call site assembles a header --------
  // Each writeX returns bytes written, or 0 when it would not fit; each readX
  // returns false when the body is the wrong length or a field is out of range,
  // and NEVER partially fills `out`.
  [[nodiscard]] Size writeScan(const Scan& s, UInt8* out, Size cap);
  [[nodiscard]] Bool readScan(const Body& b, UInt8 ver, Scan* out);
  // ... and so on for every Type above

  // ---- the shared renderer ---------------------------------------------------
  // ONE implementation, used by biboctl, the viewer's log pane, the pilot's
  // console and the tests' failure output - so a frame is printed the same way
  // everywhere and the printer cannot drift from the codec beside it.
  [[nodiscard]] Str describe(const Frame& f);

}
```

The deadman lives in `bibowire::deadman` exactly as shown in §6, and `Refuse` is declared beside it.

### The test cases that must exist for this to be believed

`firmware/pilot/tests/test_bibowire.cxx`, the same shape and runner as `test_scanwire.cxx`: file-static `checks` and `failures`, `check(Bool, const Char*)`, `checkStr(const Str&, const Char*, const Char*)` printing got/want on failure, `std::printf("\n%d checks, %d failed\n\n", ...)`, exit 0 on pass and 1 on fail. **Target ≥ 140 checks** against `test_scanwire`'s 79. String assertions go through `describe()`, so a failure reads as text rather than as hex.

**Round trips**
1. Every type, field by field, at its extremes: angle 0 and 35999, distance 0 (no return) and 12000, quality 0 and 63, `count` 0 and 1024, steer/throttle ±1000, `battMilliV` absent, `cpuCentiC` absent, `picoSilentMs` absent, `health` 255.
2. An angle of exactly 36000 is rejected (the device says 0 — `scanwire`'s own case).
3. A quality of 64 is rejected.
4. `describe()` of one frame of each type, compared with `checkStr` against an exact expected string.

**Framing**
5. **Truncation sweep:** every frame cut at every length from 0 to `len - 1` gives `TAKE_NEED_MORE`, consumes **nothing**, and never produces a message.
6. **Corruption sweep:** flip every byte of a known-good 500-point `SCAN` in turn — 2540 cases — and require each to be caught by the CRC or a bounds check. **None may yield a valid-looking frame with a different `count`.**
7. **Resync:** a valid frame preceded by 0–64 bytes of garbage, including garbage that spells `0x42 0x57`, is found and the skipped bytes are counted **exactly**.
8. `len = 0xFFFFFFFF` is rejected with no allocation; so is `len % 4 != 0`; so is `ver = 0`.
9. A frame split across every possible byte boundary of a three-frame stream, fed one byte at a time then two then three up to whole, produces identical output every time.
10. A `SCAN` whose `count` disagrees with its `len` is refused — not read as a shorter revolution.

**Compatibility**
11. Unknown type `0x7E` is skipped by **exactly** `len`, and the frame after it parses.
12. Unknown annotating flag bit 3 is ignored; `FLAG_MORE` yields `TAKE_BAD_FLAG`.
13. `ver` higher than known: prefix read, tail ignored, known fields exact.
14. `ver` lower than known: the absent fields read as their sentinels and are **never fabricated**.
15. `HELLO` with `protoMajor = 2` into a v1 board yields `BYE(VERSION)` with **non-empty** text — the test asserts the sentence exists, because an empty explanation is the failure the whole rule exists to prevent.

**The catalog**
16. `static_assert` that the catalog covers every `Type` enumerator, plus a runtime check that every entry's declared body length agrees with its `writeX`.
17. `SCHEMA` output for `SCAN` compared with `checkStr` against its exact expected line.

**Control**
18. Applying a `CONTROL` with `seq <= highestApplied` does **not** move the commanded steer or throttle, and increments the stale counter.
19. `static_cast<Int32>(a - b) > 0` wrap comparison: a viewer restarting at `seq = 1` after a reset is accepted once the high-water mark is reset by the handshake, and rejected if it is not.
20. A `CONTROL` carrying a stale `armEpoch` leaves throttle at the previous value's neutral and **still applies steering**, with `refuse = REFUSE_EPOCH`.
21. A `CONTROL` carrying a foreign `sessionId` is discarded and does not feed the deadman.
22. A `CONTROL` from a viewer that is not the control holder is discarded and **does not feed the deadman** — the Design 3 fatal flaw, pinned by a test.
23. `assumedMode` disagreement yields `refuse = REFUSE_MODE`, throttle 0, steering held.

**The deadman — a table of ~40 rows through `deadman::step`**
24. Silence at 149 ms → `STATE_LIVE`; at 150 → `STATE_SOFT`; at 299 → `STATE_SOFT`; at 300 → `STATE_DEAD`.
25. A `CONTROL` applied at 149 ms resets it to `STATE_LIVE`.
26. `nowMs < lastControlMs` → `STATE_DEAD`, **not** `STATE_LIVE`.
27. `haveHolder == false` → no deadman state is imposed and the pilot's own rules stand.
28. `enable` clear → `STATE_SOFT` regardless of age.
29. `estopLatched` outranks all three.
30. `neutralInMs` and `disarmInMs` count down correctly at 0, 74, 149, 150, 299, 300 ms and are 0 (not negative, not wrapped) after firing.
31. `STATE_DEAD` does not become `STATE_LIVE` on a fresh stream alone — it requires the explicit re-arm path.
32. `static_assert(CONTROL_DEAD_MS + PICO_HOP_BUDGET_MS <= PICO_DEADMAN_MS)` compiles, and a test asserts the three constants individually so raising one is a visible diff.

**Locale and fuzz**
33. Format the whole message set under a comma-decimal locale and assert **byte-identical** output. Nothing on this wire is a float, and this is the line that catches the day somebody adds one.
34. A fuzz loop: 200,000 pseudo-random byte sequences through `take()` — no crash, no read past the buffer, and the decoder **always makes progress** (`consumed > 0`, or `TAKE_NEED_MORE` with more bytes required than are present).

**The socket suite** (`test_viewfeed.cxx`, ctest on Linux, not in `verify.bat`), in `test_feed.cxx`'s shape:
35. One client that stops reading is coalesced to one `SCAN`, then dropped at `BEHIND_MS`, **while another keeps receiving live revolutions**.
36. A `CLASS_BULK` frame is discarded before any `CLASS_LIVE` frame when the ring is full; a `CLASS_VITAL` frame that cannot be queued closes the client.
37. A client whose inbound frame claims `len > MAX_INBOUND_PAYLOAD` is closed with `BYE(TOO_BIG)` and nothing is allocated.
38. A second viewer asking for control is accepted as an observer, and its `CONTROL` datagrams are counted and discarded.
39. A datagram carrying a previous session's `sessionId` is rejected after a reconnect.
40. The reverse-path probe emits its `EVENT` when no `CONTROL` datagram arrives within 1000 ms of `WELCOME`.

The whole pure suite is exactly that — **pure**: no socket, no clock, no device. That is the property that let `scanwire`, `proto` and `reactive` be trusted before the hardware they describe was plugged in, and it is the only reason a deadman gets tested more than once.

## 12. What the implementation found

Nine places where this document, read literally, could not be implemented. They are
recorded here rather than quietly fixed in code, because the socket half and the
viewer client still get written from this file and would otherwise rediscover each
one — or resolve it the other way. Where the prose above and
`firmware/pilot/src/bibowire.cxx` disagree, **the code and its 312 checks win.**

**12.1 The known-tag test is a resync filter, not an acceptance test.** §3 rejects a
candidate whose `type` is not a known tag. §5, §7 and case 11 promise that an
unknown type is skipped by exactly `len` and is never fatal. Both cannot hold at
one test. The known-tag condition applies only while scanning forward for a lock,
never at a frame boundary, where the CRC alone is enough. Implemented the other
way, case 11 cannot pass and an older viewer stops being able to drive a newer
board — the one property §7 says the length prefix exists to buy.

**12.2 `TAKE_TOO_BIG` and `TAKE_BAD_FLAG` must also require a known tag.** Found by
a test, not by reading. Without it, four bytes of junk that happen to spell the
magic with a nonzero `ver` close a healthy connection instead of being resynced
past. A terminal answer now needs `atBoundary && knownType`.

**12.3 `WELCOME` puts `u64 boardMonoUs` at offset 28**, which §3's "every field
naturally aligned" forbids. The byte layout is authoritative. The problem is wider
than one field: a payload starts at frame offset 12, so **no `u64` in any body is
8-aligned relative to the frame**. The alignment claim is true only of a payload
considered alone — which is exactly why the `memcpy` helpers are load-bearing
rather than stylistic.

**12.4 §4's "lower `ver` → sentinels" has no instance in v1.0.** `ver` starts at 1
and every field exists at v1, so a lower `ver` can only be 0, which is invalid
framing. The higher-`ver` prefix rule is implemented and tested, as are the
sentinel semantics that do ship. The lower-`ver` branch waits for a type to reach
v2 rather than being written blind.

**12.5 Case 8's `len = 0xFFFFFFFF` never reaches the size bound** — `len % 4 == 0`
rejects it first, so the case as written tests a different rule than it claims.
Both mechanisms are now tested separately, with `0xFFFFFFFC` for the real
`TOO_BIG` path.

**12.6 §5 leaves the class column "—" for `CTLSTATE`, `CONTROL`, `COMMAND` and
`SUBSCRIBE`**, while §11 fixes `Class` at three members. All four are
`CLASS_VITAL`, the answer that cannot silently drop. Open question for the socket
half: `CTLSTATE` mirrored onto TCP at 20 Hz as never-droppable may want
special-casing, since a vital frame that cannot be queued closes the client.

**12.7 `BYE_VERSION` collided with the `Bye` message struct.** The enum is `Reason`
with `REASON_*` members. Wire values are unchanged.

**12.8 Case 28, read literally, lets a released enable downgrade a DEAD link to
SOFT.** The precedence is estop, then dead, then soft-by-age, then
soft-by-refusal, with the boundary asserted explicitly.

**12.9 Cases 35–40 are the socket suite and are not implemented.** They belong to
`test_viewfeed.cxx` beside the `viewfeed` half, which does not exist yet. Cases
1–34 are implemented and the test file's header says which.

**12.10 "Tuning is refused while armed" was unimplementable as written, because the board could not see whether the car was armed.** The guard reads `Applied::armed`, and **nothing in the pilot ever called `viewfeed::applied()`** — so that field held 0 for the life of the process and the refusal never fired once. `BoardState::picoArmed` was no help either: it was hard-coded to 2, *"this program never asks the board its arm state"*, which is also why the viewer's Car panel could only ever print `armed --`.

The fix costs nothing on the wire. The Pico answers **every** `STEER` with `printDrive()`, and the pilot sends `STEER` on every tick — so `armed=`, `esc=` and `steer_now=` were already arriving fifty times a second and being counted as `++ok` with their contents discarded. The pilot now parses that line and calls `applied()`, which makes the guard load-bearing and the `armed` readout real at the same time.

Two traps found while doing it, both of which pass every gate. First, `proto::field` takes the value from directly after the key it matched, so **the `=` is part of the key**: `fieldInt(rest, "armed", v)` hands `strtol` the string `"=0"` and returns false on every call, leaving the guard dead and looking exactly like a working one. Second, `armed` has no "unknown" sentinel, so an unread value is 0 — permissive for this guard rather than dangerous, and it lasts one tick, but it is a default that means *not armed* rather than *not measured*.

**Status.** The pure suite runs **312 checks**, against the ≥140 §11 asks for. A
known limit, commented rather than papered over: the catalog's `static_assert`s
prove it covers `ALL_TYPES`, that tags are unique, and that the hand-written
`knownType` switch agrees with the table across all 256 tags — so adding a `Type`
and forgetting *either* fails to compile. Adding one and forgetting *both* is not
compile-detectable without reflection.