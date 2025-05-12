# L6470 Motor Driver Software

This repository contains a software driver implementation (in C) for the L6470 motor driver from STM. 

The L6470 device is a fully integrated microstepping motor driver with motion engine and a SPI interface.
The L6470 device is suitable for driving two-phase bipolar stepper motors with microstepping, and contains 
an unique control system with support of up to 1/128 step resolution.

The L6470 driver has a standard 8-bit Serial peripheral interface (SPI) to the host microcontroller (always master). 
The maximum rate is 5Mbit/s. One option for the L6470 is to place it in a SPI daisy chain configuration to spare
chip select ($\overline(CS)$) pins on the host microcontroller.

The repository currently contains two files. A single source file (L6470.c) and a header file (L6470.h)

The driver software in this repository was tested on a STM NUCLEO-H743ZI2 evaluation board together with the X-NUCLEO-IHM02A1
expansion board and two compatible stepper motors. he X-NUCLEO-IHM02A1 board is equipped with two L6470 motor drivers in a daisy
chain configuration.
 

