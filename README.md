# L6470_Driver project
This repository contains an embedded software project based on the STM NUCLEO-H743ZI2 evaluation board,
the expansion board X-NUCLEO-IHM02A1, together with one or two stepper motors compatible with the expansion board.

The X-NUCLEO-IHM02A1 board is equipped with two L6470 motor drivers. The two L6470 drivers are daisy chained.
 
L6470. 
The driver software for the L6470 motor driver is written i C. A L6470 driver instance can either be instantiated as
standalone or as daisy chained. If instantiated as standalone, the application commands issued for a driver is send
immediately (over SPI) to it. If instantiated in a daisy chain configuration, the application commands are created, but it is up
to the controlling unit when to send it to the driver. 
