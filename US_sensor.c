#include "US_sensor.h"
#include "UART.h"

volatile uint8_t timer_2_overflow_count = 0;
volatile uint16_t echo_duration = 0; // Duration of the echo signal in timer ticks
volatile bool echo_complete = false; // Flag to indicate echo completion

ISR(INT0_vect)
{ 
  if (!echo_complete) // Rising edge
  { 
      TCNT2 = 0;            // Reset Timer2 count
      timer_2_overflow_count = 0;  // Reset overflow count
      TCCR2B |= (1 << CS21); // Start Timer2 with prescaler 8
      echo_complete = true; // Start measuring
  } 
  else  // Falling edge
  {
      TCCR2B = 0; // Stop Timer2 (clear prescaler bits)
      // f_clk = 2000000Hz = 2000000 ticks/sec
      // distance in us = 1/2000000 sec/tick * x ticks * 10^-6 = 1/2 us/tick * x ticks

      echo_duration = TCNT2 + (timer_2_overflow_count << 8); // TCNT2 + overflow count * 256
      echo_duration = echo_duration >> 1; // (TCNT2 + overflow count * 256) / 2

      echo_complete = false; // Reset for next measurement
  }
}

ISR(TIMER2_OVF_vect)
{
  timer_2_overflow_count++;  // Increment overflow counter every time the timer overflows
}

void US_init()
{
  DDRB |= (1 << TRIG_PIN); // Set trig pin as output

  // Set Timer/Counter2 for normal mode
  TCCR2A = 0;            // Normal mode
  TCCR2B = 0;            // Stop Timer2
  TCNT2 = 0;             // Reset Timer2
  TIMSK2 = (1 << TOIE2);  // Enable Timer2 overflow interrupt
  
  init_ext_interrupt(); // Initialise external interrupts on echo pin

  sei();  // Enable global interrupts
}

void init_ext_interrupt()
{
  EICRA |= (1 << ISC00); // Trigger INT0 on both edges
  EIMSK |= (1 << INT0);   // Enable INT0  
}

uint16_t read_distance_US()
{
  trigger_US_sensor();
  _delay_ms(20);  // Wait for measurement to complete
  uint16_t distance = echo_duration / 58;

  uart_txString("Distance is: ");
  uart_txU16(distance);
  uart_txString("cm\n");

  return distance;
}

void trigger_US_sensor()
{
  PORTB |= (1 << TRIG_PIN);   // Set trigger pin high
  _delay_us(10);              // Wait for 10 microseconds
  PORTB &= ~(1 << TRIG_PIN);  // Set trigger pin low
}