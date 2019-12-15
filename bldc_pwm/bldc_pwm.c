#define __AVR_ATmega328P__

#include <avr/io.h>
#include <util/delay.h>

#define PULSE_VALUE 12

void initTimers(void)
{
  // Timer 1 A,B, 16 bit timer
  // Fast PWM mode, counter max in ICR1, this can control the frequency of pwm
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  // Configure the frequency of timer1, prescale it by F_CPU / 64, since our CPU
  // run at 16MHZ, the frequency of timer is 1/4 MHZ, a count in timer will spend
  // 4 microsecond = 0.004 ms
  TCCR1B |= (1 << CS11 | 1 << CS10);

  // Set the counter max in ICR1, which is the TOP value for timer1, about 0.048ms
  //. The frequency is about 20KHz
  ICR1 = 12;

  // Create output on OC1A which is PB1 pin
  TCCR1A |= (1 << COM1A1);

  // Set the OC1A pin to output
  DDRB |= (1 << PORTB1);
}

int main(void)
{
  // The OC1A will first set on BOTTOM and clear at compare match to OCR1A value.
  // We set PULSE value to 8, wich is about 75% duty.
  OCR1A = PULSE_VALUE;

  // Init the timer1 for our BLDC motor
  initTimers();

  // ------ Event loop ------ //
  while (1)
  {
    OCR1A = PULSE_VALUE;
  }
  return 0;
}
