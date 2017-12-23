# pill\_serial

Based on [libopencm3-examples usb_cdcacm](https://github.com/libopencm3/libopencm3-examples/tree/master/examples/stm32/f1/stm32-h103/usb_cdcacm)

To build run:

    make libopencm3
    make bin

then flash the `pill_serial.bin` file to an STM32F103 blue pill

This example implements a USB CDC-ACM device (aka Virtual Serial Port)
to demonstrate the use of the USB device stack.

