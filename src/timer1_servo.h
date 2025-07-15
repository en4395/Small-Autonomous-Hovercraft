#ifndef timer1_servo_h
#define timer1_servo_h

#include <inttypes.h>

void servo_setup(uint8_t en_IRQ);

long servo_map(long x, long in_min, long in_max, long out_min, long out_max);

void set_servo_angle(uint8_t angle);

void set_servo_pulse(uint16_t pulse);

void servo_test();

#endif