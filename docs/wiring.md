# Wiring

## Pico 2 W pin map

| Pin | Function |
|---|---|
| GP0 | Servo signal |
| GP1 | ESC signal |
| GP4 | I2C SDA (ToF sensors, IMU, display) |
| GP5 | I2C SCL |
| GP9 | UART RX (lidar, if ever wired to the Pico) |
| GP10 | ToF #1 XSHUT — front level |
| GP11 | ToF #2 XSHUT — front angled ~20° down, curb detection |
| GP12 | ToF #3 XSHUT |
| GP13 | ToF #4 XSHUT |
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

- **Never connect BEC 5V to the Pico while USB is attached.** During development
  the Pico is USB-powered from a power bank. Back-feeding the 5V rail from the
  BEC with USB also connected risks both.

- **Consolidating to BEC-only power later needs a 1000 µF / 25 V cap** across the
  5 V rail. The BEC budget is 2 A total; the lidar draws ~230 mA on top of a
  servo, which is tight — it likely wants its own supply.

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

Currently connected over USB to the host, not to the Pico. GP9 is reserved for
UART RX if the reactive lidar layer is ever moved onto the Pico.

---

## LED lighting

MIBIDAO pre-wired RC light pairs, 3–7 V, resistors already inline. Driven
through a **ULN2003** — the Pico's total GPIO current budget (~50 mA) cannot
drive ten LEDs directly, even at 3.3 V.

See conventions.md for the lighting *behaviour* spec (brake/indicator priority,
1.5 Hz flash, hazards on watchdog fire).
