#include "timer1_servo.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
/* 
  A lot of the initialisation code is from the init.c file that Dmitry posted
  on the Moodle course page.
*/
#define F_CPU 16000000UL

/* 
  Constant for Timer1 50 Hz PWM (ICR mode)
  f_(OC1A) = f_(clk) / (2 * N * TOP), N is prescalar
  50Hz = 16MHz / (2 * 64 * TOP)
  TOP = 2500 
*/
#define PWM_TOP 2500  // From phase and frequency correct PWM frequency formula (datasheet p. 137)

/* 
  Again, these values are calculated from the PWM frequency formula on p.137
  Rearranging that formula, you get COUNT = pulse_duration * f_(clk) / (2 * 64)
*/
#define SERVO_MIN 85
#define SERVO_MIDDLE 188
#define SERVO_MAX 290

// ======= PWM0 and PWM1 control (16-bit timer1) ===================
// Perhaps use this later
// ISR for input capture interrupt
ISR(TIMER1_CAPT_vect) 
{

}

// en_IRQ enables input capture interrupt (might be useful later)
void servo_setup(uint8_t en_IRQ) 
{ 
  DDRB    |= (1 << PORTB1);
  TCCR1A  |= (1 << COM1A1) | (1 << COM1B1); // non-inv PWM on channels A and B
  TCCR1B  |= (1 << WGM13);  // PWM, Phase and Frequency Correct. TOP = ICR1.
  ICR1    = PWM_TOP; // 50Hz PWM
  OCR1A   = 188;  // For 0 degree angle
  OCR1B   = 0; 
  TCCR1B |= ((1 << CS11) | (1 << CS10)); // Timer prescaler of 64
  if (en_IRQ)
    TIMSK1 |= (1 << ICIE1); // enable Input Capture Interrupt. NOTE: the ISR MUST be defined!!! 
}

// DO NOT USE MAP IN FINAL VERSION. ATMEGA328P DOESN'T HAVE DIVISION HARDWARE
long servo_map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void set_servo_angle(uint8_t angle)
{
  uint16_t target;
  if(angle <= 90)
  {
    target = servo_map(angle, 0, 90, SERVO_MIN, SERVO_MIDDLE);
  }
  else
  {
    target = servo_map(angle, 91, 180, SERVO_MIDDLE, SERVO_MAX);
  }

  OCR1A = target;
}

void set_servo_pulse(uint16_t pulse)
{
  OCR1A = pulse;
}

void servo_test()
{
  for(int i = 0; i < 180; i++)
  {
    set_servo_angle(i);
    _delay_ms(15);
  }
  for(int i = 180; i > 0; i--)
  {
    set_servo_angle(i);
    _delay_ms(15);
  }
}