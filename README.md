# Freddie 2

A little autonomous robot.

## Parts

Excluding build choices and stock components.

- [DFROBOT Romeo ESP32-S3 Robotics Dev Board](https://thepihut.com/products/romeo-esp32-s3-development-board-for-robotics-fpv-rc-car)
- [Gens Ace LiPo G-Tech 2S 7.4V 1000mAh 30C Soaring with XT60](https://www.modelsport.co.uk/product/1361979)
- [SparkFun Qwiic Buzzer](https://thepihut.com/products/sparkfun-qwiic-buzzer)
- [Waveshare Robot Chassis Kit MS](https://thepihut.com/products/robot-chassis-kit-ms)

## Docs

- [ESP32 wiki](https://wiki.dfrobot.com/dfr0994)
- [Qwiic buzzer data sheet](https://docs.sparkfun.com/SparkFun_Qwiic_Buzzer/hardware_overview/#qwiic-and-i2c)
- [Romeo ESP32-S3 board schematic](./docs/DFR0994-schematics-v1.0.pdf)
- [Freddie 2 Schematic](./freddie2.pdf)

## Build

```sh
cd freddie2
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

