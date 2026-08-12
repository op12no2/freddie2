# Freddie 2

A little autonomous robot.

## Parts

Excluding build choices and stock components.

- [Gens Ace LiPo G-Tech 2S 7.4V 1000mAh 30C Soaring with XT60](https://www.modelsport.co.uk/product/1361979)
- [Expressif ESP32-S3-DevKitC-1 Development Board](https://thepihut.com/products/esp32-s3-devkitc-1-development-board)
- [2 off Pololu DRV8833 Dual Motor Driver Carrier](https://thepihut.com/products/pololu-drv8833-dual-motor-driver-carrier)
- [Adafruit MPM3610 5V Buck Converter Breakout - 21V In 5V Out at 1.2A](https://thepihut.com/products/adafruit-mpm3610-5v-buck-converter-breakout-21v-in-5v-out-at-1-2a)
- [USB-C Breakout - Horizontal](https://thepihut.com/products/usb-c-breakout-horizontal)
- [SparkFun Qwiic Buzzer](https://thepihut.com/products/sparkfun-qwiic-buzzer)
- [Hi-Link LD2410C 24GHz Human Presence Sensor](https://thepihut.com/products/ld2410c-24ghz-human-presence-sensor)
- [Waveshare Robot Chassis Kit MS](https://thepihut.com/products/robot-chassis-kit-ms)

## Docs

- [ESP32 data sheet](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/index.html)
- [DRV8833 data sheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf)
- [LD2410C data sheet](https://www.hlktech.net/index.php?id=1095)
- [Qwiic buzzer data sheet](https://docs.sparkfun.com/SparkFun_Qwiic_Buzzer/hardware_overview/#qwiic-and-i2c)
- [Freddie 2 Schematic](./freddie2.pdf)

## Build

```sh
cd freddie2
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

