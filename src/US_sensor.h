#ifndef US_SENSOR_H
#define US_SENSOR_H

// US sensor pins corresponding to "P6" on ENCS board
#define ECHO_PIN PD2
#define TRIG_PIN PB3

#include <inttypes.h>
#include <avr/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

void US_init();

void init_ext_interrupt();

uint16_t read_distance_US();

void trigger_US_sensor();

#endif