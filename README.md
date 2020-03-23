# greetings
This repo is an experimental collection of firmware and software for the
Betz drive, with emphasis on real-time control frequency and reducing
current control noise. The firmware is currently written bare-metal on
the STM32, without an OS or platform library of any sort.

# notes
 * MCU: STM32F405RG

# getting ready
```
sudo apt install gcc-arm-none-eabi openocd
```
If you want to flash firmware directly to the board, and you don't have
a udev rule set up yet for your SWD programmer, you'll need something
like this (the VID/PID is for the Olimex USB-TINY-H programmer):
```
ACTION=="add", ATTR{idVendor}=="0403", ATTR{idProduct}=="6010", MODE:="666"
```
then run `sudo udevadm trigger` to reload the rules.

# compiling firmware
```
cd firmware
mkdir build
cd build
cmake ..
make
```

# flashing stuff
First, connect a SWD programmer to the header on the board. If it doesn't
exist already, add an appropriate rule in `/etc/udev/rules.d` so you don't
need `sudo` to access the device. Then, from the firmware `build` directory:

```
make PROGRAM.flash_swd
```
