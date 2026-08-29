# Wiring

## Two boards

There are two RP2350 boards in this project and they are not interchangeable at
the image level, only at the wiring level.

| | Board | Role |
|---|---|---|
| **Workhorse** | Pico 2 **W** | The breadboard mule. Everything is developed and proven here first. |
| **Car** | Pico **2** (no wireless) | The board that goes in the vehicle. |

**The pin map below applies unchanged to both.** Every pin this project uses —
GP0–GP1, GP4–GP5, GP9–GP13, GP15–GP22, GP26–GP28 — is a plain GPIO on both
boards, so the harness, the ToF wiring, the SPI bus and the servo/ESC leads all
carry straight over. Nothing has to be re-planned to move to the car.

What differs is the four pins **neither board lets you use freely**, and they
differ completely. From the SDK's own board headers:

| Pin | Pico 2 | Pico 2 W |
|---|---|---|
| GP23 | SMPS mode control | `WL_REG_ON` — wireless power |
| GP24 | VBUS sense | `WL_DATA` — wireless data |
| GP25 | **User LED** | `WL_CS` — wireless chip select |
| GP29 | VSYS sense (ADC3) | `WL_CLOCK` (also VSYS sense, shared) |

On the W the LED, SMPS control and VBUS sense all move onto the CYW43439's own
GPIO space (its pins 0, 1 and 2) — which is why the LED there cannot be reached
with `gpio_put` at any pin number, and why `gpio_put(25, 1)` from a Pico 1
example compiles, runs, and does nothing.

**This project never uses GP23, GP24, GP25 or GP29**, which is what makes the
two boards drop-in for each other. Keep it that way: a design that reaches for
GP25 as "the LED" works on the car and silently drives the wireless chip's chip
select on the mule.

### Lighting, and the pins it is borrowing

The lighting MODEL is permanent and lives in `firmware/lib/lights.h`: ten
lamps - head, tail, indicator and reverse, left and right - and all ten are
computed whether or not an LED exists for them. **What each lamp means** moved
to `firmware/lib/cue.h` on 2026-08-28: `lights.h` is the output layer now and
`cue.h` decides. See [firmware-api.md](firmware-api.md).

What is temporary is the BINDING, a table at the top of `lights.h` saying which
GPIO currently shows which lamp. Today:

| Lamp | Pin | Note |
|---|---|---|
| headL | GP11 | borrowed from ToF #2 XSHUT — added 2026-08-28 |
| headR | GP10 | borrowed from ToF #1 XSHUT — added 2026-08-28 |
| tailL | GP15 | **borrowed from the wheel encoder** |
| tailR | GP14 | genuinely spare — the encoder B channel, unused on a forward-only car |
| indFL | GP13 | borrowed from ToF #4 XSHUT |
| indFR | GP12 | borrowed from ToF #3 XSHUT |
| indRL, indRR | — | the second indicator pair, not wired yet |
| revL/R | — | computed, reported, not wired |

**All four ToF XSHUT lines are now lamps.** GP10-GP13 were listed below as the
bumper sensors' shutdown pins, and every one of them currently has an LED on
it. They are free only because the I2C bus is empty - `SCAN` answers 0 - and
they stop being free the moment a *single* ToF is fitted, not just the third
and fourth. The hub's System panel still lists "ToF bumpers (GP10-13)" as a
subsystem; that row describes where those sensors are going, not what is on
those pins today.

The car gets **two pairs of indicators**, front and rear. All four are in the
model and computed; the rear pair simply has no pin. Front and rear on a side
share one flash structurally rather than by two timers agreeing — two lamps on
the same corner blinking a frame apart is instantly wrong, and per-lamp timers
drift.

GP15 goes back to the encoder the moment the Hall sensor is fitted — that is the
one borrowing with a deadline on it. GP10 through GP13 stop being free the
moment any ToF goes on the bus.

The permanent five-pin map is the one above: GP2/GP3 indicators, GP6/GP7 tails,
GP8 both heads. Moving there is editing that table and nothing else.

**The tail lamps mean "no throttle", not "braking".** There is no brake on this
car and no way to measure whether it is slowing until the encoder is on, so the
lamp reports the one throttle fact available. A car standing still with the ESC
disarmed therefore has its tails lit, which is right by the rule and wrong for a
real car.

### Building and flashing for each

```
firmware\build.bat              ->  firmware\build\        (pico2_w, the mule)
firmware\build.bat pico2        ->  firmware\build-pico2\  (pico2, the car)
```

Separate trees, because changing the board invalidates most of a build tree and
sharing one would mean a full rebuild — on the W, the whole cyw43 stack — every
time you switched. In the hub's Pico 2 W view, each catalog entry shows the
board it was built for; the car's images are the ones marked `pico2`.

**A wrong image does not announce itself.** The RP2350 accepts either UF2 and
boots it. A W image on the car's plain Pico 2 runs, enumerates, and answers
every serial command — with a dead LED, because it spent startup bringing up a
wireless chip that is not in the package. Check the board tag before flashing,
and check `ID`'s `board=` field after.

The firmware asks the SDK which board it is compiled for rather than being told:
`lib/hal.h` switches the LED implementation on `CYW43_WL_GPIO_LED_PIN` versus
`PICO_DEFAULT_LED_PIN`, and `CMakeLists.txt` links `pico_cyw43_arch_none` only
when `PICO_CYW43_SUPPORTED` is set. Nothing in this repo hard-codes a board name
except the default.

---

## Pin map (both boards)

| Pin | Function |
|---|---|
| GP0 | Servo signal |
| GP1 | ESC signal |
| GP4 | I2C SDA (ToF sensors, IMU, display) |
| GP5 | I2C SCL |
| GP9 | UART RX (lidar, if ever wired to the Pico) |
| GP10 | ToF #1 XSHUT — front level. **Currently the RIGHT headlight.** |
| GP11 | ToF #2 XSHUT — front angled ~20° down, curb detection. **Currently the LEFT headlight.** |
| GP12 | ToF #3 XSHUT. **Currently the right front indicator.** |
| GP13 | ToF #4 XSHUT. **Currently the left front indicator.** |
| GP15 | Encoder signal |
| GP20 | TFT reset (RES) |
| GP22 | MicroSD CS — CS can be any GPIO, which is why it lives here |
| GP26 | SPI1 SCK (MicroSD CLK) |
| GP27 | SPI1 MOSI (MicroSD) |
| GP28 | SPI1 MISO (MicroSD) — **must** be GP8/12/24/28, the silicon says so |
| GP21 | TFT data/command (DC) |
| GP16 | SPI MISO (SD card) |
| GP17 | SPI CS — shared bus: TFT now, MicroSD needs its own CS |
| GP18 | SPI SCK |
| GP19 | SPI MOSI |
| GND | **Common ground with ESC — mandatory** |

Servo and ESC PWM both run at 50 Hz.

---

## Invariants

These are the rules that, when broken, produce failures that look like software
bugs. Check them first.

- **Common ground between Pico and ESC is required.** Signal and ground cross
  between the two power domains; power never does. A missing common ground
  presents as erratic or absent servo response and is not a code bug.

- **A breadboard's power rails are SPLIT in the middle.** Bridge each one with a
  jumper before trusting it. The printed red and blue stripes run straight past
  the break, so the rail looks continuous when it is two independent halves, and
  a ground on one half is not the same node as a ground on the other.

  This cost an entire evening on 2026-08-27. The servo twitched at power-up,
  held a position when asked to hold, and ignored every command to move — which
  is precisely what the invariant above predicts, and it was diagnosed instead
  as a damaged servo, a dead feedback pot, a sagging BEC, and swapped signal
  pins, in that order. The Pico's ground and the BEC's ground were four
  centimetres apart on what looked like one rail.

  If a servo behaves strangely, check this SECOND, straight after checking the
  signal pin. Before the servo, before the supply, before the code.

- **Never connect BEC 5V to the Pico while USB is attached.** During development
  the Pico is USB-powered from a power bank. Back-feeding the 5V rail from the
  BEC with USB also connected risks both.

- **Consolidating to BEC-only power later needs a 1000 µF / 25 V cap** across the
  5 V rail. The BEC budget is 2 A total; the lidar draws ~230 mA running on top
  of a servo, which is tight — it likely wants its own supply.

  Size it on the **start** current, not the running one: the C1 pulls **800 mA
  to spin up** against 230 mA once turning, and the datasheet wants ripple under
  150 mV. An under-fed C1 does not fail cleanly — it reports short or missing
  returns, which reads as a dirty window or a dead sensor rather than as a power
  problem.

- **All VL53L1X boot at I2C address 0x29.** Firmware must sequence the XSHUT
  lines at startup to bring them up one at a time and assign unique addresses.
  This is why each sensor gets its own GPIO rather than sharing one.

- **The car goes on a stand, wheels off the ground, for every first run of new
  code.** No exceptions.

---

## Two power domains

Isolated, joined by exactly one wire.

```
  CAR DOMAIN                            COMPUTE DOMAIN
  NiMH pack -> ESC -> motor             separate supply -> SBC
                |                                          |
              BEC 5V                                    USB out
                |                                       /      \
          servo, receiver                          Pico       lidar

                        \___ COMMON GROUND ___/
```

---

## Connectors and polarity

**ESC — Hobbywing QuicRun/THW 1060**, 60 A, 5 V/2 A BEC, Deans male battery
connector.

- Set battery type to **NiMH**, not LiPo. LiPo mode cuts off early on this pack.
- Motor wiring: ESC **yellow (+) → motor yellow**, ESC **blue (−) → motor green**.
  If the motor spins backwards, swap them — harmless.
- A spare kit ESC is kept as a known-good swap for fault isolation.

**Servo — Power HD 1501MG.** 17 kg·cm @ 6 V, 0.14 s/60°, deadband ≤ 4 µs.

- Cable is black/white; the outer conductor **with the white stripe is signal**.
- **Middle pin is +5 V** — servo convention, always.
- The remaining outer conductor is ground.

**Battery — 2× Tenergy 7.2 V 3800 mAh NiMH**, 383 g each, Tamiya male connector.
Rapid charge 1900 mA × 2.1 hr — use the charger's 2 A setting.

**Adapter:** Deans **female** → Tamiya **male**. The first one bought was the
wrong direction; the correct one is in hand and works.

**Radio — Flysky FS-GT2 + FS-GR3E receiver.** CH1 = servo, CH2 = ESC. BIND/CH3
and VCC are unused. The receiver is powered by the ESC's BEC through the CH2
lead.

---

## Lidar connector

RPLIDAR C1, XH2.54-5P. Pin order is **VCC, TX, RX, GND** = red, yellow, green,
black. One of the five positions is unpopulated — that is correct, not a fault.

From the C1M1 datasheet rev 1.1, Figure 2-6 — external interface signal
definition:

| Colour | Signal | Type | Description | Min | Typ | Max |
|---|---|---|---|---|---|---|
| Red | VCC | Power | Total power | 4.8 V | 5 V | 5.2 V |
| Yellow | TX | Output | Serial output of the scanner core | 0 V | / | 3.5 V |
| Green | RX | Input | Serial input of the scanner core | 0 V | / | 3.5 V |
| Black | GND | Power | Ground | 0 V | 0 V | 0 V |

Serial is **3.3 V TTL UART, 460800 baud, 8N1** (Figure 2-8). The datasheet lists
no other rate — 460800 is the only one, not a preferred one.

Power (Figure 2-7): **800 mA to start, 230 mA typical running** (260 mA max at
10 Hz), ripple **≤ 150 mV**. The start figure is the one that sizes the supply.

Key measurement numbers (Figure 2-1): 0.05–12 m range on a 70 %-reflective
target and 0.05–6 m on a 10 %-reflective one, ±30 mm accuracy, **15 mm
resolution**, 5 kHz sample rate, 8–12 Hz scan rate (10 Hz typical), 0.72°
angular resolution, 360° field of view with a 0–1.5° scan-plane flatness, 40 klux
ambient limit, IP54. Working temperature −10 to 40 °C, and it will not **start**
below 0 °C (Figure 2-9).

Currently connected over USB to the host, not to the Pico. GP9 is reserved for
UART RX if the reactive lidar layer is ever moved onto the Pico.

The same facts are drawn in the hub's Reference viewer under **Sensors →
RPLIDAR C1**.

---

## LED lighting

MIBIDAO pre-wired RC light pairs, 3–7 V, resistors already inline. Driven
through a **ULN2003** — the Pico's total GPIO current budget (~50 mA) cannot
drive ten LEDs directly, even at 3.3 V.

See conventions.md for the lighting *behaviour* spec (brake/indicator priority,
1.5 Hz flash, hazards on watchdog fire).
