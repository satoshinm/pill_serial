pill\_serial: USB-to-serial x 3
===============================

Triple USB-to-serial adapter firmware for flashing onto an STM32F103C8T6 "blue pill" minimum development board

Run `make` to build, then flash the `src/pill_serial.bin` file to a blue pill over the PA9/PA10 serial port with BOOT0=1.
Thenplug in the blue pill into your PC using USB, and three virtual (ACM CDC, "/dev/usbmodem") serial ports should appear.
These correspond to the three USART ports available on the board, in order:

| TX pin | RX pin |
| ------ | ------ |
| PB10   | PB11   |
| PA2    | PA3    |
| PA9    | PA10   |

This code is heavily based on the [Black Magic Debug Probe firmware](https://github.com/blacksphere/blackmagic).
Note if you only need one serial port, you may be better off using the Black Magic Probe, since it also provides a JTAG/SWD probe,
whereas pill\_serial only provides serial ports.
