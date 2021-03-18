```c
This is a test program for the Electrodragon AVR128DB64 development board.
https://www.electrodragon.com/product/avr128db64-mini-develpment-board-avr128/

Developed on MPLAB X IDE using the XC8 compiler and debugged with SNAP.

This program can easily be modified to run on most any other development
board that makes use of Microchip's AVR-DB MCUs, including the AVR128DB48 Curiosity
Nano.

This software provides functionality that includes:

A printf() capability to print formatted messages to the console.

Receive UART characters are accumulated via an interrupt routine and placed on
a FIFO for use by application code.

TCB0 is used to cause an interrupt every 1 ms, to create an
Arduino-like millisx() function, that returns a 32 bit value that represents
the number of milliseconds since the last system reset.

TCB1 is used to create an event output at a rate of 16,0000 times a second.
That event causes ADC0 to start a conversion.  At the end of the conversion ADC0
will cause an interrupt.  That interrupt will read the value of ADC0, reduce
the resolution of the value from 12 bits to 10 bits,  change the justification of
the datum to be compatible with DAC0 and then write the value read by the ADC0 to
DAC0.  At 16,000 times a second when a 1 kHz sine wave of 1Vpp is applied to ADC0
with a +1VDC offset (to keep the input signal within GND and VREF) that sampled
sine wave can be clearly be seen with an oscilloscope at the output of DAC0.

A switch on PC7 (active low) is configured to cause an interrupt and
set a global semaphore flag for use by a polled function.  

Every 5 seconds a Hello World message is printed to the console.

Every 500 ms an LED (active high) on PC6 is toggled.

Every 100 ms a polled function checks the status of the switch interrupt
semaphore and prints a message to the console when found set.

Every 750 ms the status of the PC7 switch is checked and if found active
(closed) a message is printed to the console.

Every 623 ms the last conversion value of the ADC is printed to the console.

```


