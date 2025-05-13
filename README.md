# L6470 Motor Driver Software

## Intro
This repository contains a software driver implementation (in C) for the L6470 motor driver from STM <sup>[1]</sup>. 

The L6470 device is a fully integrated microstepping motor driver with motion engine and a SPI interface.
The L6470 device is suitable for driving two-phase bipolar stepper motors with microstepping, and contains 
an unique control system with support of up to 1/128 step resolution.

The L6470 driver has a standard 8-bit Serial peripheral interface (SPI) to the host microcontroller (always master). 
The maximum rate is 5Mbit/s. One option for the L6470 is to place it in a SPI daisy chain configuration to spare
GPIO pins on the host microcontroller for chip select.

The driver software in this repository was tested on a STM NUCLEO-H743ZI2 evaluation board together with the X-NUCLEO-IHM02A1
expansion board <sup>[2]</sup> and two compatible stepper motors. The X-NUCLEO-IHM02A1 board is equipped with two L6470 motor drivers in a daisy
chain configuration.

## Design & implementation
The design and implementation is kept simple. Hence, it comes with some limitations. The single source file basically contains
public functions for the various application commands found in <sup>[1]</sup>. Also, a set of public 'utility' functions are provided for
conversions.

### Limitations
The implementation does not handle SPI communication. It provides a hook (or callback function) for sending the created SPI commands.
This is a design choice. It is up to the user whether to make usage for this hook. If used, the application command is send to the 
specified L6470 device immediately. If unused, an application command is just created (not send), and it is up to the user when to send it.
This enables the user to send commands to multiple daisy chained L6470 devices simultaneous (or close to).

### ToDo
- Handle GetStatus response
- L6470_Config implementation
- Elaporate on alarms

## Status
The released version present is only a draft, and will be updated asap.

## References
[1] <https://www.st.com/en/motor-drivers/l6470.html>

[2] <https://www.st.com/en/ecosystems/x-nucleo-ihm02a1.html>
 

