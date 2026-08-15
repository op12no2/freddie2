# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for Freddie 2, an autonomous robot and the successor to `../freddie`.
The big differences from Freddie 1: an off-the-shelf robotics board (DFRobot
Romeo ESP32-S3, DFR0994) instead of hand-wired modules, and a mecanum-wheel
chassis (Waveshare Robot Chassis Kit MS) with four independently driven
wheels, so the robot can translate in any direction as well as rotate.

The firmware (`main/freddie2_main.c`) is currently a motor bench-test
console: chip info at boot, then single-letter commands over the USB-C
console (`?` prints the list — `t` runs each motor in turn to identify
wheel wiring, `m`/`a`/`d` drive motors with an auto-stop timeout,
`s` stops). Behavioral code will grow from here, likely following
Freddie 1's single-file, single-tick-task style — see
`../freddie/CLAUDE.md` for how that firmware was structured.

Motor commands default to a 3 s auto-stop so a forgotten command can't
drive the robot off the bench; keep that property as the code grows.
The DRV8876s are driven in PH/EN mode: EN pin = LEDC PWM at 20 kHz,
PH pin = direction. The wheel-position mapping macros (`FL_CH`/`FL_POL`
etc.) are placeholders until verified with `t` on the wired chassis.

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

Motor-related jumpers, all shorted in this build: the PMODE jumper
(shorted = PH/EN mode, which is what the firmware assumes; open = PWM
mode) and per-channel EN/PH jumpers (e.g. M1EN/M1PH) that connect the
ESP32 GPIOs to the driver inputs. A popped EN/PH jumper looks exactly
like a dead motor channel — check them before debugging firmware.

**I2C**: GPIO 1 (SDA), GPIO 2 (SCL). Planned peripherals on this bus:
SparkFun Qwiic Buzzer, and serial-attached Hi-Link LD2410C 24 GHz human
presence sensor (UART, not I2C — pin assignment TBD).

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
