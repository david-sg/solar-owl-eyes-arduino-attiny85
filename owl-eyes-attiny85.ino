// owl eyes with 3 functions, and combo of 3 and random seed
// combined with japanese lantern power controls etc.

#include <avr/sleep.h>
#include <avr/power.h>    // Power management
#include <avr/wdt.h>      // Watchdog timer
//clear bit macro
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif




long readVcc() {
  ADMUX = _BV(MUX3) | _BV(MUX2);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}



// watchdog interrupt
ISR (WDT_vect)
{
  wdt_disable();  // disable watchdog
}  // end of WDT_vect

void resetWatchdog ()
{
  // clear various "reset" flags
  MCUSR = 0;
  // allow changes, disable reset, clear existing interrupt
  WDTCR = bit (WDCE) | bit (WDE) | bit (WDIF);
  // set interrupt mode and an interval (WDE must be changed from 1 to 0 here)
  WDTCR = bit (WDIE) | bit (WDP3) | bit (WDP1) | bit (WDP0);    // set WDIE, and 2 seconds delay
  // pat the dog
  wdt_reset();
}  // end of resetWatchdog

// 10 second test
//long cycles = 15;
// 4.5 hours (6intervals*60mins*4.5hrs)
long cycles = 300;

// pin 3 is the red led warning light
int red = 4;
// pin 1 is left eye
int left = 1;
//pin 0 is right eye
int right = 0;

int t = 500;

int light_power = 75;


void setup()
{
  resetWatchdog ();  // do this first in case WDT fires


  for (int i = 0; i < 5; i++) {
    pinMode(i, OUTPUT);
  }

  //add input on solar voltage
  pinMode(3, INPUT);
//  check pin 2 for random seed
  pinMode(2, INPUT);
  randomSeed(analogRead(2)); // randomize using noise from analog pin 2

}

void delay_time () {
  delay(random(3000, 4000));
}

void left_eye () {
  //left off right on
  digitalWrite(left, LOW);
  analogWrite(right, light_power);
  //  digitalWrite(right, HIGH);
  delay(t);
  //right off left on
  digitalWrite(right, LOW);
  analogWrite(left, light_power);
  //  digitalWrite(left, HIGH);
  delay(t);
  // both on
  analogWrite(left, light_power);
  //  digitalWrite(left, HIGH);
  analogWrite(right, light_power);
  //  digitalWrite(right, HIGH);
}

void right_eye () {
  //right off left on
  digitalWrite(right, LOW);
  analogWrite(left, light_power);
  //  digitalWrite(left, HIGH);
  delay(t);
  //left off right on
  digitalWrite(left, LOW);
  analogWrite(right, light_power);
  //  digitalWrite(right, HIGH);
  delay(t);
  // both on
  analogWrite(left, light_power);
  //  digitalWrite(left, HIGH);
  analogWrite(right, light_power);
  //  digitalWrite(right, HIGH);
}

void blink () {
  //blink once
  //both off
  digitalWrite(left, LOW);
  digitalWrite(right, LOW);

  delay(t);
  //both on
  analogWrite(left, light_power);
  //  digitalWrite(left, HIGH);
  analogWrite(right, light_power);
  //  digitalWrite(right, HIGH);
}

void combo () {
  switch (random(5)) {
    case 0:
      left_eye();
      right_eye();
      break;

    case 1:
      right_eye();
      left_eye();
      break;
    case 2:
      right_eye();
      blink();
      break;
    case 3:
      left_eye();
      blink();
      break;

    case 4:
      blink();
      delay(t);
      blink();  
      delay(t);   
      blink();
      delay(t);
      blink();
      break;

    default:
      blink();
      delay(t);
      blink();

      break;

  }
}

void loop()
{
  // reset the low power state
  int low_power = 0;

  //check the solar panel voltage
  int sensorValue = analogRead(A3);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float solarVoltage = sensorValue * (5.0 / 1023.0);
  long voltage = readVcc();
//  solarVoltage = 0;
  //check if its been 4.5 hours, low voltage or if its daytime
  if ( cycles < 10 || voltage < 3200 || solarVoltage >= 0.30) {
    //      reset the cycles if its daytime
    if ( solarVoltage >= 0.30) {
      cycles = 600;
    }
    //  daytime so turn off the lights and go to sleep
    digitalWrite(left, LOW);
    digitalWrite(right, LOW);
    digitalWrite(red, LOW);
    goToSleep ();
  } else {
    if (voltage < 3400) {
      low_power = 1;
      digitalWrite(red, HIGH);
      digitalWrite(left, LOW);
      digitalWrite(right, LOW);
    } else {
      digitalWrite(red, LOW);
    }

    if (low_power != 1) {
      switch (random(3)) {
        case 0:
          left_eye();
          break;

        case 1:
          right_eye();
          break;

        case 2:
          blink();
          break;

        default:
          combo();

      }
      delay(random(3000, 10000));

      cycles --;
    }

  }

}


void goToSleep ()
{

  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  static byte prevADCSRA = ADCSRA;
  ADCSRA = 0;
  power_all_disable ();  // power off ADC, Timer 0 and 1, serial interface
  noInterrupts ();       // timed sequence coming up
  resetWatchdog ();      // get watchdog ready
  sleep_enable ();       // ready to sleep
  interrupts ();         // interrupts are required now
  sleep_cpu ();          // sleep
  sleep_disable ();      // precaution
  power_all_enable ();   // power everything back on
  //  ADCSRA = 1;            // turn on ADC
  ADCSRA = prevADCSRA;
}  // end of goToSleep
