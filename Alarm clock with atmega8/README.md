# Simple alarm clock with Microchip's AVR and avr-libc

This mini-project implements a simple alarm clock using AVR's MCUs. The project is written based on ATmega8 but you can use any Microchip's MCU with an asynchronous timer.

To display time, you will need a common cathode four-digit multiplexed 7-segment display. Since the common 7-segment displays work with 2-3 volts, you will need 8 resistors to connect the display to the MCU. Also to set the clock and alarm, you will need 3 push-buttons, 6 resistors and 3 debouncing capacitors.

You can find schematic and PCB files [here](https://github.com/GHOST-mHBr/PCBs/tree/master/Alarm-Clock)

## Simulation

Proteus simulation files placed inside the `test` folder.

> :warning: Note that the simulation and schematic files inside this folder are not suitable for a concrete circuit. There is no power supply, no resistors between 7-segment and the MCU, no debouncing cap, etc.
> So if you really want to make the circuit, take a look at schematic.

## Bugs?

If you found any bugs consider creating an issue please.
