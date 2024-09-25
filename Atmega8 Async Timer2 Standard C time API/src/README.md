# Implementing RTC
In this repository, I've implemented an RTC using Atmega8 and its asynchronous timer2. The intersting part about this code is that it is compatible with Unix's time api(at least part of it).
For example you can find current time using `localtime()` function!

## NOTES
* Please note that this code is not completely compatible with Unix time API (after all, this is a tiny MCU with very limited resources!).
For more inforamation about restrictions, refer to the avr-libc documents (timer.h file)

* Also keep in mind that in order to calculate time, we need a watch crystal connected directly to the pins TOSC1, TOSC2 (a watch crystal is a crystal with the frequency 32768Hz)
 
