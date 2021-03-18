/* 
 * File:   main.c
 * Author: Paul Newland
 *
 * Created on March 15, 2021
 * 
 */

///////////////////////////////////////////////////
//  This is a test program for the Electrodragon AVR128DB64 development board.
//  https://www.electrodragon.com/product/avr128db64-mini-develpment-board-avr128/
//
//  Developed on MPLAB X IDE using the XC8 compiler and debugged with SNAP.
//
//  This program can easily be modified to run on most any other development
//  board that makes use of Microchip's AVR-DB MCUs, including the AVR128DB48 Curiosity
//  Nano.
//
//  This software provides functionality that includes:
//
//  A printf() capability to print formatted messages to the console.
//
//  Receive UART characters are accumulated via an interrupt routine and placed on
//  a FIFO for use by application code.
//
//  TCB0 is used to cause an interrupt every 1 ms, to create an
//  Arduino-like millisx() function, that returns a 32 bit value that represents
//  the number of milliseconds since the last system reset.
//
//  TCB1 is used to create an event output at a rate of 16,0000 times a second.
//  That event causes ADC0 to start a conversion.  At the end of the conversion ADC0
//  will cause an interrupt.  That interrupt will read the value of ADC0, reduce
//  the resolution of the value from 12 bits to 10 bits,  change the justification of
//  the datum to be compatible with DAC0 and then write the value read by the ADC0 to
//  DAC0.  At 16,000 times a second when a 1 kHz sine wave of 1Vpp is applied to ADC0
//  with a +1VDC offset (to keep the input signal within GND and VREF) that sampled
//  sine wave can be clearly be seen with an oscilloscope at the output of DAC0.
//
//  A switch on PC7 (active low) is configured to cause an interrupt and
//  set a global semaphore flag for use by a polled function.  
//
//  Every 5 seconds a Hello World message is printed to the console.
//
//  Every 500 ms an LED (active high) on PC6 is toggled.  Pressing and holding
//  the PC7 push button will cause the LED to remain in its current state
//  while the button is in the down position.
//
//  Every 100 ms a polled function checks the status of the switch interrupt
//  semaphore and prints a message to the console when found set.
//
//  Every 750 ms the status of the PC7 switch is checked and if found active
//  (closed) a message is printed to the console.
//
//  Every 623 ms the last conversion value of the ADC is printed to the console.
//
///////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <string.h>
#include <stdbool.h>

#define CLK_16MHZ_INT  1
#if defined(CLK_16MHZ_INT)
   #define F_CPU 16000000UL
#endif 

#define USART4_BAUDRATE 57600

#define PROGRAM_FUSES 0  // set to 1 if this program is to program fuses
#define SET_LOCKBITS 0  // set to 1 if this program is to set lockbits
 



//=======================================================================
// Common Code below is often used for multiple projects
//=======================================================================

int USART4_printChar(char c, FILE *stream);
// allow for printf statements
FILE USART_stream = FDEV_SETUP_STREAM(USART4_printChar, NULL, _FDEV_SETUP_WRITE);

// fuse settings
#if defined(PROGRAM_FUSES ) && PROGRAM_FUSES
    FUSES = {
      .WDTCFG   = 0x00, // WDTCFG {PERIOD=OFF, WINDOW=OFF}
      .BODCFG   = 0x10, // BODCFG {SLEEP=DISABLE, ACTIVE=DISABLE, SAMPFREQ=32Hz, LVL=BODLEVEL0}
      .OSCCFG   = 0x78, // OSCCFG {CLKSEL=OSCHF}
      .SYSCFG0  = 0xF6, // SYSCFG0 {EESAVE=CLEAR, CRCSEL=CRC32, CRCSRC=NOCRC}
      .SYSCFG1  = 0xE8, // SYSCFG1 {SUT=0MS, MVSYSCFG=DUAL}
      .CODESIZE = 0x00, // CODESIZE {CODESIZE=User range:  0x0 - 0xFF}
      .BOOTSIZE = 0x00, // BOOTSIZE {BOOTSIZE=User range:  0x0 - 0xFF}
    };
#endif
    
// lock bits
#if defined(SET_LOCKBITS ) && SET_LOCKBITS
    #define LOCKBITS32 uint32_t __lock LOCKMEM
    LOCKBITS32 = 0x5CC5C55C; // KEY {KEY=NOLOCK}
#endif
    
//=======================================================================
// In this program the tertiary functions will be position first,
// then the secondary functions, and then main() at the bottom
//=======================================================================


//=======================================================================
// function prototypes, as needed
void loadDAC0(uint16_t);
uint16_t fifo0Free(void);


//=======================================================================
//  systemInit will
//    - initialize the processor clock
//    - initialize timer TB0 for one int every millisecond
//    - set up GPIO pins for LED at PC6 and switch at PC7
//    - set up a UART4 for TX and RX, with printf() support for TX
//          and interrupt with FIFO on RX

void initSystem() {
  
#if defined(CLK_24MHZ_INT)
  
  //============================================
  // Enable internal oscillator for 24 MHz
  //============================================
  // enable the xtal 32 KHz clock, I don't think it turns on until autotune is enabled
  _PROTECTED_WRITE (CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm);
  //while(!(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)); // Wait until ext crystal 32Khz stable
 
  // select 16 MHz internal oscillator with autotune enabled
  //_PROTECTED_WRITE (CLKCTRL.OSCHFCTRLA, (CLKCTRL_FREQSEL_24M_gc | CLKCTRL_AUTOTUNE_bm));
  _PROTECTED_WRITE (CLKCTRL.OSCHFCTRLA, (CLKCTRL_FREQSEL_24M_gc));
  
  // set processor to run from internal HF oscillator, with clock output on A7
  _PROTECTED_WRITE (CLKCTRL.MCLKCTRLA, (CLKCTRL_CLKSEL_OSCHF_gc | CLKCTRL_CLKOUT_bm));
  while(!(CLKCTRL.MCLKSTATUS & CLKCTRL_OSCHFS_bm)); // Wait until internal HF osc stable    
  while(CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm); // Wait until main clock change complete

#elif defined(CLK_16MHZ_INT)

  //============================================
  // Enable internal oscillator for 16 MHz
  //============================================
  // enable the xtal 32 KHz clock, I don't think it turns on until autotune is enabled
  _PROTECTED_WRITE (CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm);
  //while(!(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)); // Wait until ext crystal 32Khz stable
 
  // select 16 MHz internal oscillator with autotune enabled
  _PROTECTED_WRITE (CLKCTRL.OSCHFCTRLA, (CLKCTRL_FREQSEL_16M_gc | CLKCTRL_AUTOTUNE_bm));
  
  // set processor to run from internal HF oscillator, with clock output on A7
  _PROTECTED_WRITE (CLKCTRL.MCLKCTRLA, (CLKCTRL_CLKSEL_OSCHF_gc | CLKCTRL_CLKOUT_bm));
  while(!(CLKCTRL.MCLKSTATUS & CLKCTRL_OSCHFS_bm)); // Wait until internal HF osc stable    
  while(CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm); // Wait until main clock change complete

#elif defined(CLK_16MHZ_XTL)
  
  //============================================
  // Enable external crystal oscillator for 16 MHz, 4K start up
  //============================================
 	_PROTECTED_WRITE ( CLKCTRL.XOSCHFCTRLA, (CLKCTRL_RUNSTDBY_bm
	  | CLKCTRL_CSUTHF_4K_gc
	  | CLKCTRL_FRQRANGE_16M_gc
	  | CLKCTRL_SELHF_CRYSTAL_gc
	  | CLKCTRL_ENABLE_bm) );
	while(!(CLKCTRL.MCLKSTATUS & CLKCTRL_EXTS_bm)); // Wait for Xtal startup
  _PROTECTED_WRITE (CLKCTRL.MCLKCTRLB, 0x00);  // Clear Main Clock Prescaler
  // Set the main clock to use XOSCHF as source, and enable the CLKOUT pin
	 _PROTECTED_WRITE (CLKCTRL.MCLKCTRLA, (CLKCTRL_CLKSEL_EXTCLK_gc | CLKCTRL_CLKOUT_bm) );
  while(CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm) ; // wait for change to complete
  // Clear RUNSTDBY for power save during sleep
	//_PROTECTED_WRITE (CLKCTRL.XOSCHFCTRLA, (CLKCTRL.XOSCHFCTRLA & ~CLKCTRL_RUNSTDBY_bm) );
#endif  
  
  
  //============================================
  // set up timer TCB0 for int every 1.000 ms
  //============================================
  TCB0.CTRLA = TCB_CLKSEL0_bm; // use clk/2, timer is off, all others off
  TCB0.CTRLB = 0x00; // periodic interrupt mode, all others are defaults
  TCB0.EVCTRL = 0x00; // take the defaults
  TCB0.INTCTRL = 0x00; // no interrupts yet
  TCB0.CCMP = (F_CPU/2000)-1; // 1000 interrupts per second
  TCB0.INTCTRL = TCB_CAPT_bm; // enable interrupt on capture
  TCB0.CTRLA |= TCB_ENABLE_bm; // turn on timer
  
  //============================================
  // set up GPIO pins as needed
  //============================================
  // set up LED at PC6
  PORTC.DIRSET = PIN6_bm; // set DDR bit for LED output
  PORTC.OUTSET = PIN6_bm;  // LED off
  // set up switch at PC7.  In this case we'll set up a switch closure to cause
  // an interrupt on falling edge.  If processor will ever be in POWER-DOWN sleep
  // mode be sure to user "either edge" or "level" so that a switch closure can wake
  // processor via interrupt (rising or falling won't cause interrupt when MCU is in
  // POWER-DOWN sleep mode).
  // Also enable pull up resistor. 
  PORTC.PIN7CTRL = PORT_ISC_FALLING_gc // input active, interrupt on falling edge
                 | PORT_PULLUPEN_bm; // enable pullupinput  
  

  //============================================
  // set up the system serial port, UART4
  //============================================
  USART4.BAUD = (uint16_t)( ( ((F_CPU * 4) / USART4_BAUDRATE) * 10 + 5 ) / 10 );
  //USART4.BAUD = 1111;
  USART4.CTRLB |= USART_RXEN_bm | USART_TXEN_bm; // enable TX and RX
  //PORTMUX.USARTROUTEA |= 0x00; //PORTMUX_USART00_bm; // USART0 mux
  PORTMUX.USARTROUTEB |= 0x00; //PORTMUX_USART40_bm; // USART4 mux
  PORTE.DIR |= PIN0_bm; // set the primary USART4 TX pin (PE0) as output
  stdout = &USART_stream;
}


//=======================================================================
// Serial Data Processing 
//  Each character received by the UART will cause an interrupt and those
//  characters will be added to an interrupt protected FIFO.  The application
//  can then pull data from the FIFO as needed, provided it's quick enough that
//  the FIFO is not over-run.  If the FIFO is full and a new character is received,
//  the oldest character on the FIFO will be dropped to make space for the new
//  character.  Note that all FIFO functions momentarily disable interrupts.

//////////////////////////////////////////////////////////
// FIFO 0, RX input from USART
//////////////////////////////////////////////////////////
#define fifo0BUFMAX  128
uint8_t fifo0Buf[fifo0BUFMAX];
uint16_t fifo0PutIdx = 0;
uint16_t fifo0GetIdx = 0;



// FIFO 0, Get byte from FIFO
//  call this ONLY when you KNOW a byte is available from FIFO
uint8_t fifo0Get() {
  uint8_t a; // value to be returned
  uint8_t tmpSREG;
  tmpSREG = SREG;  // save int status reg in tmpSREG
  cli(); // turn off interrupts for what will likely be only a few microseconds
    a = fifo0Buf[fifo0GetIdx++];
    if (fifo0GetIdx >= fifo0BUFMAX) {
      fifo0GetIdx = 0;
    }
  SREG = tmpSREG;  // restore interrupt status, likely re-enabling interrupts if they were enabled before
  return(a);
}
//FIFO 0, Put byte on FIFO
//  if fifo is full then oldest byte will be removed from FIFO
//  and  new byte will then be added to fifo
void fifo0Put(uint8_t a) {
  // put a byte of data onto Baudot TX Buffer
  uint8_t tmpSREG;
	tmpSREG = SREG;  // save int status reg in tmpSREG
  cli(); // turn off interrupts for what will likely be only a few microseconds
    if (fifo0Free() < 1) {
      fifo0Get(); // pull one byte from fifo to make new for new byte
    }
    fifo0Buf[fifo0PutIdx++] = a;
    if (fifo0PutIdx >= fifo0BUFMAX) {
      fifo0PutIdx = 0;
    }
  SREG = tmpSREG;  // restore interrupt status, likely re-enabling interrupts if they were enabled before
}
// FIFO 0, return number of data bytes available on fifo
uint16_t fifo0Av() {
  // return number of bytes available in fifo
  int16_t val;  // value to return
  uint8_t tmpSREG;
  tmpSREG = SREG;  // save int status reg in tmpSREG
  cli(); // turn off interrupts for what will likely be only a few microseconds
    val = fifo0PutIdx - fifo0GetIdx;
    if (val < 0) {
      val += fifo0BUFMAX;
    }
  SREG = tmpSREG;  // restore interrupt status, likely re-enabling interrupts if they were enabled before
  return(val); // return the number of bytes available in FIFO
}
// FIFO 0, return number of unused bytes in FIFO
uint16_t fifo0Free() {
  // free space remaining in FIFO
  // indices can't be allowed to become equal as that indicates empty FIFO
  uint16_t val; // value for return
  uint8_t tmpSREG;
  tmpSREG = SREG;  // save int status reg in tmpSREG
  cli(); // turn off interrupts for what will likely be only a few microseconds
    val = fifo0BUFMAX - fifo0PutIdx + fifo0GetIdx - 1;
    if (val >= fifo0BUFMAX) {
      val -= fifo0BUFMAX;
    }
  SREG = tmpSREG;  // restore interrupt status, likely re-enabling interrupts if they were enabled before
  return(val); // return the number of free byte in FIFO
}
// FIFO 0, clear FIFO, dump all data on FIFO
void fifo0Clear() {
  uint8_t tmpSREG;
  tmpSREG = SREG;  // save int status reg in tmpSREG
  cli(); // turn off interrupts for what will likely be only a few microseconds
     fifo0PutIdx = 0;
     fifo0GetIdx = 0;
   SREG = tmpSREG;  // restore interrupt status, likely re-enabling interrupts if they were enabled before
}

// easy read from FIFO, returns -1 if no data on FIFO.
// note that if 0xff is put onto FIFO, when it is found
// by serialRead() then this function will get "stuck")
uint8_t serialRead(void) {
  uint16_t val; 
  uint8_t dat;
  val = fifo0Av();
  if (val > 0) {
    dat = fifo0Get();
  }
  else {
    dat = -1;
  }
  return dat;
}


// return number of data bytes  on FIFO
uint16_t serialAvailable(void) {
  return fifo0Av();
}


// return value that will be provided by next fifo0Get(), without affecting FIFO)
// note that if nothing is on the FIFO this function will not report that fact, so
// do serialAvailable() first.)
uint8_t serialPeek(void) {
  uint8_t tmpSREG;
  tmpSREG = SREG;  // save int status reg in tmpSREG
  cli(); // turn off interrupts for what will likely be only a few microseconds
    uint8_t val = fifo0Buf[fifo0GetIdx]; // get char at top of Fifo
  SREG = tmpSREG;  // restore interrupt status, likely re-enabling interrupts if they were enabled before
  return val; // return that character
  }


//============================================
// UART functions
//============================================


void USART4_sendChar(char c) {
  while (!(USART4.STATUS & USART_DREIF_bm)); // wait for last char to be completed
  USART4.TXDATAL = c; // put the char to the uart TX
}

int USART4_printChar(char c, FILE *stream)
{ 
    USART4_sendChar(c);
    return 0; 
}




void USART4_sendString(const char *str)
{
  uint16_t i = 0;
  while( str[i] != '\0') 
  {
    USART4_sendChar(str[i++]);
  }
}





//=======================================================================
// settings to control the primary LED at PC6
void led0Off(void) {
  PORTC.OUTSET = PIN6_bm;  // LED off
}

void led0On(void) {
  PORTC.OUTCLR = PIN6_bm;  // LED on
}

void led0Toggle(void) {
  PORTC.OUTTGL = PIN6_bm; // toggle LED state
}


//============================================
// Interrupt Service Routines
//============================================

volatile static bool switchPC7Semaphore = false;  // must be VOLATILE!
//=======================================================================
// ISR to handle interrupt from switch closure on PC7
//=======================================================================
ISR(PORTC_PORT_vect) {
  if(PORTC.INTFLAGS & PIN7_bm)     {
      PORTC.INTFLAGS &= PIN7_bm; // clear interrupt
      switchPC7Semaphore = true; // set a semaphore
  }
}

//============================================
// ADC0 Interrupt Service Routine
// 
//============================================
volatile int16_t adc0Val;
volatile bool adc0Semaphore;
ISR(ADC0_RESRDY_vect) {
  //led0Toggle();  
  adc0Val = ADC0.RES;  // reading ADC0.RES will clear int flag
  loadDAC0(adc0Val>>2);  // output value onto DAC, DAC is 10 bits
  adc0Semaphore = true; // mark data as available, in case it is needed
}




//=======================================================================
// Establish a mechanism that provides capabilities like Arduino millis(),
// however in this case we will call it millisx() - note the x in function name.
// timer TCB0 will be used for this function

volatile uint32_t __millisecCtr = 0;
volatile uint32_t __microSecAcc = 0;


// interrupt routine for TCB0.  just count milliseconds in a global
// 32 bit variable.  
ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
  __millisecCtr++; // increment the number of milliseconds since starting
}

// return number of milliseconds since power-on
uint32_t millisx(void) {
  uint32_t val;
  //  uint8_t tmpSREG;
  //	tmpSREG = SREG;  // save int status reg in tmpSREG
  //    val = __millisecCtr;
  //  SREG = tmpSREG; // restore int status to that found on function entry
  do {
    val = __millisecCtr;
  } while (val != __millisecCtr);
  return val;
}





//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Application specific code below
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void initApplicationTimersAndIO(void) {
  
  //============================================
  // Timer TCB1 runs from CLK_PER divided by 2 to create an event
  // which then will kick the ADC to do a conversion.
  //============================================
  TCB1.CTRLA = TCB_CLKSEL0_bm; // use clk/2, timer is off, all others off
  TCB1.CTRLB = 0x00; // periodic interrupt mode, all others are defaults
  TCB1.EVCTRL = 0x00; // take the defaults
  TCB1.INTCTRL = 0x00; // no interrupts yet
  TCB1.CCMP = ((F_CPU/2) / 16000) - 1; // 16000 events per second
  //TCB1.INTCTRL = TCB_CAPT_bm; // comment out, not using ints for this application
  TCB1.CTRLA |= TCB_ENABLE_bm; // turn on timer
  
  
  //============================================
  // set up ADC for reading  
  //============================================  
  VREF.ADC0REF = VREF_REFSEL_2V048_gc; // select 2.048 Volts as ADC0 ref
  ADC0.MUXPOS = ADC_MUXPOS_AIN0_gc;  // select MUX input, A0 (PD0))
  ADC0.CTRLC |= ADC_PRESC_DIV4_gc; // clock si PER_CLK/4
  ADC0.CTRLA = ADC_ENABLE_bm  // ADC Enable: enabled
               | ADC_RESSEL_12BIT_gc;     // 12-bit mode
  ADC0.EVCTRL |= ADC_STARTEI_bm;  // ADC to be triggered by a rising edge event:
  ADC0.INTCTRL = ADC_RESRDY_bm; // allow interrupt when ADC conversion complete
   
   
  //============================================
  // set up DAC output on PD6 (the only option)
  //============================================  
  VREF.DAC0REF = VREF_REFSEL_2V048_gc; // select 2.048 Volts as DAC0 ref
  //The DAC output pin needs to have the digital input buffer and the pull-up resistor disabled in order to reduce its load.
  PORTD.PIN6CTRL &= ~PORT_ISC_gm;
  PORTD.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTD.PIN6CTRL &= ~PORT_PULLUPEN_bm;
  //  enable the DAC, Output Buffer, and Run in Standby mode
  DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm | DAC_RUNSTDBY_bm;
  

  
  //============================================
  // set up Event Channels
  //
  // Channel 0 is for concatenating TCB0 into TCB1
  //  but it will be commented out for this appliation
  //
  // Channel i is for TCB1 to kick ADC0 into starting a conversion
  //============================================  
  
  // Event Channel 0: TCB0 Overflows kicks event channel 0
  // which causes TCB1 to count
  //EVSYS.CHANNEL0 = EVSYS_CHANNEL0_TCB0_CAPT_gc;
  //EVSYS.USERTCB1COUNT = EVSYS_USER_CHANNEL0_gc;

  // Event Channel 1: TCB1 Overflows kicks event channel 1
  // which causes ADC to begin a conversion
  EVSYS.CHANNEL1 = EVSYS_CHANNEL0_TCB1_CAPT_gc;
  EVSYS.USERADC0START = EVSYS_USER_CHANNEL1_gc;    

}

//============================================
// ADC0_read
//
// Note that this is a BLOCKING function
//============================================   
uint16_t ADC0_read(void)
{
    /* Wait for ADC result to be ready */
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
    /* Clear the interrupt flag by reading the result */
    return ADC0.RES;
}


//============================================
// loadDAC0
//
// load data into DAC
//============================================ 
// this is a 10 bit DAC.  The DAC output
// register is 16 bits wide and is LEFT justified.
// the DAC output it not updated until the DAC HIGH byte
// is written, so write the DAC low byte first followed by
// the DAC high byte.  The DAC low byte holds the two least
// value bits in position B7 and B6.  To load to the DAC low
// byte take the 10 bit DAC value, mask off all but the two
// lowest bits, left shift six times and write that 8 bit result
// into low DAC byte.  Then take the 10 bit DAC value and shift
// it right two times, mask result so as to leave only the least
// eight bits intact and write that 8 bit value into DAC HIGH byte.
void loadDAC0(uint16_t value) {
  DAC0.DATAL = (value & 0x03) << 6;  // mask shift and load low byte
  DAC0.DATAH = (value >> 2) & 0xFF; // shift, mask and load low byte
}


//============================================
// Main())
//============================================   
int main(int argc, char** argv) {
  
  uint8_t switchC7;

  initSystem();
  initApplicationTimersAndIO();
    
  sei(); //  Enable Global Interrupts
  
  printf("System startup message\n");

  while (1) {
    
    // Every 500ms toggle the LED
    static uint32_t timeRef0 = 0;  // declare static variable for this timer
    if (millisx() - timeRef0 > 500) {
      // timeout occurred!
      timeRef0 = millisx(); // re-init the timeRef0
      if ((PORTC.IN & PIN7_bm)) {
        // if switch is open (high) then toggle LED)
        led0Toggle(); // toggle the LED
      }
    }    
    
    // Every 5 seconds print hello world message to console
    static uint32_t timeRef1 = 0;  // declare static variable for this timer
    if (millisx() - timeRef1 > 5000) {
      // timeout occurred!
      // it's been 5 seconds since last time
      static uint16_t lapCtr = 0;
      timeRef1 = millisx(); // re-init the timeRef0
      lapCtr++;  // increment the lap counter
      printf("Hello World, %u\n", lapCtr); // print message to console
    }
    
    // Every 100 ms do a polled check of interrupt semaphore caused by PC7 switch
    // If semaphore found, clear it, and print message to console
    static uint32_t timeRef2 = 0;  // declare static variable for this timer    
    if (millisx() - timeRef2 > 100) {
      // timeout occurred!
      timeRef2 = millisx(); // re-init the timeRef1
      // check for semaphore from TCB0 interrupt that occurs every millisecond.
      // because switches have mechanical bounce, there will probably be several
      // interrupts over the course of 10 ms for every switch closure.  However,
      // since this program is checking the semaphore status only once every 100 ms,
      // it will infrequently detect this bounce.
      if (switchPC7Semaphore == true) {
        // interrupt semaphore was detected
        switchPC7Semaphore = false; // clear the semaphore (should this write be interrupt protected?)
        printf("Switch on PC7 caused INTERRUPT!\n");  // print the diagnostic message
      }
    }
    
    // Every 750 ms do a polled check of PC7 switch itself
    static uint32_t timeRef3 = 0;  // declare static variable for this timer
    if (millisx() - timeRef3 > 750) {
      // timeout occurred!
      timeRef3 = millisx(); // re-init the timeRef2    
      // check the status of the switch at PC7 (ignoring the interrupt routine) and
      // place the switch status in the variable as either 1 or 0.  "PORTC.IN & PIN7_bm"
      // results in either a zero or some non-zero value.  applying the ! (not) operator
      // will change any non-zero value to zero, and a zero to 1.  Generally speaking,
      // using !! (double not) is an easy way to reduce any and all non-zero values to
      // a value of 1.  !!0 is always zero.
      switchC7 = !(PORTC.IN & PIN7_bm);  // if switch is closed, the expression returns a 1
      if (switchC7 == 1) {
        printf("Polled check indicates Switch at PC7 is closed!\n");  // print message when switch closed
      }
    }
    
    // Every 623 ms do a polled read of last ADC0 conversion value and show on console
    // Using timeout value of 623 ms just to do something unusual
    static uint32_t timeRef4 = 0;  // declare static variable for this timer
    if (millisx() - timeRef4 > 623) {
      // timeout occurred!
      timeRef4 = millisx(); // re-init the timeRef3   
      printf("ADC= %4d\n", ADC0.RES);
    }
  }
  return (EXIT_SUCCESS);
}


