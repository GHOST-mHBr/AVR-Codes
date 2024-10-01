# Simple alarm clock with Microchip's AVR and avr-libc

This mini-project implements a simple clock using AVR's MCUs. The project is written based on ATmega8 but you can use any Microchip's MCU with an asynchronous timer.

To display the time, you will need a common cathode four-digit multiplexed 7-segment display. Since the common 7-segment displays work with 2-3 volts, you will need 8 resistors to connect the display to the MCU. Also to set the clock and alarm, you will need three push-buttons, three pull-up resistors and three debouncing capacitors.

I will upload an schematic (and probably a PCB design) soon.

## Simulation

You can find Proteus simulation files inside the `test` folder.

> :warning: Note that the simulation and schematic files inside this folder are not suitable for a concrete circuit. There is no power supply, no resistors between 7-segment and the MCU, no debouncing cap, etc.

## Bugs?

Please if you found any bugs consider creating an issue.
