/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/
#include <avr/wdt.h>
#include <avr/sleep.h>


//const int LED_BUILTIN = 13;
int data = 0;


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  Serial.println(""); // Make a clean restart
  delay(1000);
  // Init watchdog timer
//  watchdogSetup();
//  watchdogCounter = 0;
    setup_watchdog(9);
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec 
  // initialize digital pin LED_BUILTIN as an output.
  
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  Serial.println(data);
  if(data == 1){   //8*75 s == 600s == 10 mins
    data = 0; 
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    sleepNow();
  }
}
void sleepNow()         // here we put the arduino to sleep
{
    ACSR |=_BV(ACD);//Turn off ADC (analog to digital conversion)
    ADCSRA=0;//OFF ADC
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin
// turn off brown-out enable in software
    MCUCR = bit (BODS) | bit (BODSE);  // turn on brown-out enable select
    MCUCR = bit (BODS);        // this must be done within 4 clock cycles of above                               
    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
}
void setup_watchdog(int ii) {
  
  byte bb;

  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}
//WDT interrupt
ISR(WDT_vect) {
  ++data;
// wdt_reset();
}
