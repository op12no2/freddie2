# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for Freddie 2, an autonomous robot and the successor to `../freddie`.
The big differences from Freddie 1: an off-the-shelf robotics board (DFRobot
Romeo ESP32-S3, DFR0994) instead of hand-wired modules, and a mecanum-wheel
chassis (Waveshare Robot Chassis Kit MS) with four independently driven
wheels, so the robot can translate in any direction as well as rotate.

The firmware (`main/freddie2_main.c`) is currently a bench-test console:
chip info at boot, peripheral init, then single-letter commands over the
USB-C console (`?` prints the list). All hardware is wired and verified:
motors (`t` wheel test, `m`/`a`/`d` drive, `x` floor demo, `s` stop),
Qwiic buzzer (`b` beep, `p` vocab phrases, `r` babble, `g` glide, `q`
raw sound segment), and the LD2410C radar (`h` live stream). Boot is
deliberately quiet — no sounds or motion until commanded.

The next chapter is behavioral code: a Freddie 1-style single tick task
and a watcher that reacts to the radar (someone arriving/approaching/
leaving) with sounds and motion — see `../freddie/CLAUDE.md` for how
that firmware was structured. Two hard-won lessons for it: (1) any
consumer reading the radar slower than its ~10 Hz stream must drain the
UART backlog and take the latest frame, or it reacts to seconds-old
data; (2) sounds are `seg_t` sequences played by `seg_play()` — phrases
are data tables, and the babble generator shows how to synthesize them.

Motor commands default to a 3 s auto-stop so a forgotten command can't
drive the robot off the bench; keep that property as the code grows.
The DRV8876s are driven in PH/EN mode: EN pin = LEDC PWM at 20 kHz,
PH pin = direction. Sound-vocabulary tuning is treated as an ongoing
pastime of the project owner — expect hand-edits to the `PH_*` tables.

On order: a Fermion 1.54" 240x240 IPS TFT (DFR0649, ST7789) for the
GDI FPC connector, which also brings a microSD slot over the same
ribbon (GDI_SDCS = IO0, SPI on IO15/16/17 — note the Romeo's onboard
SD socket shares these same lines and CS, so only one slot may hold a
card at a time). GDI quirk from the schematic: GDI_BLK (backlight) is
IO21 = M2's direction pin, so the backlight will flick with the
front-right wheel's direction — cosmetic, accepted; GDI touch lines
collide with M1's pins, so touch screens are out (this one isn't).

There is no test suite — this is embedded C for one physical device.
Correctness is checked by flashing and watching the serial console.

## Build / flash / monitor

Requires the ESP-IDF toolchain installed at `~/esp/esp-idf` (v6.1-dev at
time of writing). The dev host is a headless Raspberry Pi 5 connected to
the board over USB-C.

```sh
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

The board flashes over the ESP32-S3's **native USB-Serial/JTAG** on its
USB-C port, so it enumerates as `/dev/ttyACM0` (not `ttyUSB0` — that was
Freddie 1's external UART bridge). The console is routed to the same port
(`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG` in `sdkconfig.defaults`), so one
cable does flash + logs. If flashing won't start, hold BOOT and tap RST to
force download mode.

## Hardware

**Board**: DFRobot Romeo ESP32-S3 (DFR0994) — ESP32-S3, 16 MB flash, 8 MB
octal PSRAM, four TI DRV8876 motor drivers (one per wheel, 2.5 A each), so
all four mecanum wheels are independently driven. Wiki:
https://wiki.dfrobot.com/dfr0994 — board schematic is in the repo at
`docs/DFR0994-schematics-v1.0.pdf`.

**Power** (verified against the schematic): three inputs, and which ones
are live matters for what you can test:
- USB-C (5 V) — powers all logic. Bench mode: USB-C alone is enough for
  everything except spinning motors. The USB rail has no path to VM, so
  USB can never power motors.
- VIN (7–24 V) — battery input for the MCU side (2S LiPo in this build),
  through a 3 A fuse into a buck converter.
- VM (5–24 V) — motor supply. Motors only turn if VM is powered, so
  motor code "doing nothing" on the bench is expected, not a bug. In this
  build JP1 is shorted, tying VM to the battery.

USB-C and battery may be connected simultaneously — the USB and VIN 5 V
rails are diode-ORed everywhere they meet, so neither back-feeds the
other. No unplugging order to worry about. But note: with the battery
attached and JP1 shorted, **motors are live during flashing** — put the
robot on a block with wheels free before flashing motor code. The power
switch S1 gates both supplies, so it turns the board off even on USB.

**Motor channels** (EN = PWM, PH = direction):
- M1: GPIO 12 (EN), GPIO 13 (PH)
- M2: GPIO 14 (EN), GPIO 21 (PH)
- M3: GPIO 9 (EN), GPIO 10 (PH)
- M4: GPIO 47 (EN), GPIO 11 (PH)

Wheel mapping (verified with the `t` console command on the wired
chassis): M1 = front-left, M2 = front-right, M3 = back-left,
M4 = back-right, no polarity flips — encoded once in the `FL_CH`/`FL_POL`
macros in the main file. Mecanum drive means all four wheels get
independent signed speeds.

Mecanum wheel placement (verified on the floor with the `x` demo): the
kit's L-labelled wheels go front-left and back-right, R-labelled wheels
front-right and back-left — same-handed wheels sit on a diagonal, never
on the same side. Check: viewed from above, the roller diagonals form an
X converging on the robot's center. Get this wrong and forward/backward
still works but strafing degrades into dragging.

Motor-related jumpers, all shorted in this build: the PMODE jumper
(shorted = PH/EN mode, which is what the firmware assumes; open = PWM
mode) and per-channel EN/PH jumpers (e.g. M1EN/M1PH) that connect the
ESP32 GPIOs to the driver inputs. A popped EN/PH jumper looks exactly
like a dead motor channel — check them before debugging firmware.

**I2C**: GPIO 1 (SDA), GPIO 2 (SCL). On the bus: SparkFun Qwiic Buzzer
at 0x34 (ATtiny register map — freq/volume/duration/active registers,
probed at boot, `b` console command beeps it).

**LD2410C human presence radar** (verified with the `h` console command):
UART1 at 256000 8N1 — ESP TX = GPIO 40 → sensor RX, ESP RX = GPIO 41 ←
sensor TX, powered from the 5 V servo rail (logic is 3.3 V despite the
5 V supply). Streams ~10 Hz binary reports (presence state, moving/still
target distance + energy); `h` streams them to the console. Indoors the
still-target energy is pegged at 100 at close range — behavior code
should key off state/distance changes and moving energy, and the sensor's
per-gate sensitivities are configurable over UART or via Hi-Link's BLE
phone app when tuning is needed.

(An OV3660 camera on the DVP connector was briefly brought up — frame
capture verified, then removed in favour of the radar. The working code,
including the schematic-correct data-pin order the wiki gets wrong, is
in git history at commit a3c5883 if vision is ever revisited.)

## Repo layout

- `main/freddie2_main.c` — the firmware (kept as a single file unless it
  earns splitting).
- `main/CMakeLists.txt` / `CMakeLists.txt` — ESP-IDF component/project
  registration. Add new ESP-IDF component deps to `PRIV_REQUIRES` in
  `main/CMakeLists.txt` as they're used.
- `sdkconfig.defaults` — target, flash size, PSRAM, console routing.
  `sdkconfig` is regenerated from it; put durable changes in
  `sdkconfig.defaults`.
- `README.md` — parts list, doc links, build instructions.
- `build/` — generated, gitignored.
