# Pico IMU Monitor

## Overview 
A command-line IMU monitor for the Raspberry Pi Pico (RP2350) that reads motion data from the TDK ICM-45686 over I2C. Written in C++ using the Pico SDK and built with CMake — no IDE required, just a terminal and a compiler.

This project was built to learn embedded C++ development from the ground up: raw register access, I2C protocol implementation, byte-order handling, and sensor calibration, all without hiding behind Arduino abstractions or PlatformIO magic.

## Prerequisites
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- ARM GCC toolchain ('arm-none-eabi-gcc')
- CMake >= 3.13
- A serial terminal ('screen' or 'minicon')

see the [official Pico SDK getting started guide](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) for detailed SDK installation

## Hardware 
- Raspberry Pi Pico (RP2350)-
- TDK ICM-45686 IMU (I2C , address= 0x68)

## Wire Table 
|RP2350          |IMU         |Note                             |           
|----------------|------------|---------------------------------|
|  3v3 (out)     |  VCC       |  Power supply                   |
|  GND           |  GND       |  Common ground                  |
|  SDA01 (GPIO1) |  SDA       |  I2C0 data                      |
|  SCL01 (GPIO2) |  SCL       |  I2C0 clock                     |
|  GND   (GPIO3) |  SAO/SDO   |  Tied low for I2C address `0x68`|
|  3v3out        |  CS        |  Tie high force I2C mode        |


## Features 

- [x] Raw register-level I2C communication
- [x] Little-endian sensor data integration
- [x] IPREG indirect register access (Low Pass Filter (LPF) Configuration)
- [x] Interative serial menu 
- [ ] Gyro/Accel calibration
- [ ] Sensor Fusion


## Pico-SDK setup 

### 1. Install dependencies 

```bash
# Ubuntu/Debian
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

Or Follow the [official guide](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
### 2.Clone the SDK 
```bash
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
export PICO_SDK_PATH=$(pwd)
```
> **Tip:** Add `export PICO_SDK_PATH=/path/to/pico-sdk` to your `~/.bashrc` or `~/.zshrc` so it persists across terminal sessions.
## Building this project
```bash
git clone https://github.com/pichmonnin/pico-imu-monitor.git
cd pico-imu-monitor
mkdir build && cd build 
cmake .. 
make -j$(nproc)
```

The build produces `pico-imu-monitor.elf` inside of `build/` directory 
## Flashing using picotool 
Ensure the Pico is in BOOTSEL mode, then run:

```bash
# Verify the device is detected
lsusb | grep "RP2350"
# Expected: Bus XXX Device XXX: ID 2e8a:000f Raspberry Pi RP2350 Boot

# Flash and reboot
picotool load build/pico-imu-monitor.elf
picotool reboot
```

## Usage
1. Connect to the Pico's USB serial port at **115200 baud**:
   ```bash
   screen /dev/ttyACM0 115200
   ```
2. You will see the menu:
   ```
   === PICO CONTROL MENU ===
   1. Show sensor data
   m. Back to Main Menu 
   =========================
   ```
3. Press **`1`** to start the live sensor stream.
4. Press **`m`** at any time to return to the menu.

## Video of demonstration


https://github.com/user-attachments/assets/122f06e0-afa9-4eee-8c4c-5f12174d2a91
