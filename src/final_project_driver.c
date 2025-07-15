#include <avr/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "IMU.h"
#include "US_sensor.h"
#include "timer1_servo.h"

/* 
  Author: Ella Noyes
  Created: November 2024

  Ports used for this program:
    - IMU uses:
      - PC5 as SCL (serial clock pin)
      - PC4 as SDA (serial data pin)
      - These are on Port 7 on ENCS board (which are connected to a semiconductor voltage regulator,
        so the default TWI Vcc is 3.3V)
    - Servo uses:
      - PB1 as output
      - Timer/Counter1 configurations (with prescaler 64)
    - US sensor uses:
      - Port 6 on ENCS board for Vcc, Trig, Echo, and GND
      - Time/Counter2 (with prescaler 8) for measuring echo time
    - IR sensor uses 
      - Port 5 on ENCS board (PC0: ADC0)
    - Lift fan uses
      - PD5
      - Timer/Counter0 with OCR0B
    - Thrust fan uses
      - PD6
      - Timer/Counter0 with OCR0A
*/

#define GYRO_RANGE 250        // Gyro range will be set to ±GYRO_RANGE
#define YAW_COMPENSATION -80  // Compensates for overturning on smooth surfaces

#define IR_READING_MIN 45
#define IR_READING_MAX 70

#define US_READING_MIN 30
#define US_CLEAR_AHEAD 37
#define US_SLOWDOWN_DISTANCE 85

#define THRUST_FAN_MEDIUM 175
#define THRUST_FAN_SLOW 50

#define LIFT_FAN_SPEED 235
#define LIFT_FAN_SLOW 225

int servo_sweep_angles[2][7] = {{90, 60, 30, 0, -30, -60, -90}, 
                                {85, 119, 153, 188, 223, 257, 290}};

uint16_t servo_pulses[181] = {
85, 86, 87, 88, 89, 90, 91, 92, 94, 
95, 96, 97, 98, 99, 100, 102, 103, 104, 105, 
106, 107, 108, 110, 111, 112, 113, 114, 115, 116, 
118, 119, 120, 121, 122, 123, 124, 126, 127, 128, 
129, 130, 131, 132, 133, 135, 136, 137, 138, 139, 
140, 141, 143, 144, 145, 146, 147, 148, 149, 151, 
152, 153, 154, 155, 156, 157, 159, 160, 161, 162, 
163, 164, 165, 167, 168, 169, 170, 171, 172, 173, 
174, 176, 177, 178, 179, 180, 181, 182, 184, 185, 
186, 187, 188, 189, 190, 192, 193, 194, 195, 196, 
197, 198, 200, 201, 202, 203, 204, 205, 206, 208, 
209, 210, 211, 212, 213, 214, 215, 217, 218, 219, 
220, 221, 222, 223, 225, 226, 227, 228, 229, 230, 
231, 233, 234, 235, 236, 237, 238, 239, 241, 242, 
243, 244, 245, 246, 247, 249, 250, 251, 252, 253, 
254, 255, 256, 258, 259, 260, 261, 262, 263, 264, 
266, 267, 268, 269, 270, 271, 272, 274, 275, 276, 
277, 278, 279, 280, 282, 283, 284, 285, 286, 287, 
288, 290 
};

#define SERVO_PULSE_VALUES 1  // Index of pulse values in sweep angles array
#define ANGLE_VALUES 0        // Index of angle values in sweep angles array
#define LOOK_TIME 1000         // Look time is 800ms
#define SERVO_STRAIGHT 188
#define SERVO_RIGHT 290
#define SERVO_LEFT 85

volatile float gyro_x, gyro_y, gyro_z; // variables for gyro raw data
volatile float roll = 0, pitch = 0, yaw = 0;

volatile bool obstacle_detected = false;
volatile bool end_of_course = false;

// Some function prototypes
void init_driver();
void init_IR_sensor();
void read_vertical_IR();
uint16_t find_gaps();
void set_servo_to_yaw(float yaw);
float get_yaw_from_ticks(uint16_t ticks);
void fans_init();
void set_lift_fan_speed(uint8_t dutyCycle);
void set_thrust_fan_speed(uint8_t dutyCycle);
void reset_angles();
void straighten_servo(uint16_t current_pulse);


int main()
{
  init_driver();
  
  uint16_t counter = 50;
  uint16_t wall_distance, turn_pulse;
  float target_yaw, compensated_target_yaw;

  while(!end_of_course)
  {
    update_gyro_angles(0.02, &roll, &pitch, &yaw);

    print_angles();

    _delay_ms(20);
    set_servo_to_yaw(yaw);

    //read_vertical_IR(); // Check for bar

    //wall_distance = read_distance_US(); // Includes 20ms delay
    // if(wall_distance < US_SLOWDOWN_DISTANCE)
    // {
    //   set_thrust_fan_speed(THRUST_FAN_SLOW);
    //   set_lift_fan_speed(LIFT_FAN_SLOW);
    // }
    // else
    // {
    //   set_thrust_fan_speed(THRUST_FAN_MEDIUM);
    //   set_lift_fan_speed(LIFT_FAN_SPEED);
    // }

    // if(wall_distance < US_READING_MIN)
    // {
    //   set_thrust_fan_speed(0);  // Stop thrust fan
    //   set_lift_fan_speed(0);    // Stop lift fan

    //   turn_pulse = find_gaps();
    //   target_yaw = get_yaw_from_ticks(turn_pulse);

      
    //   set_thrust_fan_speed(THRUST_FAN_SLOW);  // Start thrust fan
    //   set_lift_fan_speed(LIFT_FAN_SPEED);     // Start lift fan 

    //   set_servo_pulse(turn_pulse);

    //   reset_angles();
 
    //   if(target_yaw < 0)
    //     compensated_target_yaw = target_yaw - YAW_COMPENSATION;
    //   else if(target_yaw > 0)
    //     compensated_target_yaw = target_yaw + YAW_COMPENSATION;
              
    //   while(fabs(yaw) < fabs(compensated_target_yaw))
    //   {
    //     _delay_ms(10);
    //     update_gyro_angles(0.01, &roll, &pitch, &yaw);
    //   } // Let it turn

    //   reset_angles();
    //   straighten_servo(turn_pulse);
    
    //   set_lift_fan_speed(LIFT_FAN_SPEED); // Restart lift fan
    // }
  }

  set_lift_fan_speed(0);
  set_thrust_fan_speed(0);

  return 0;
}

void init_driver()
{
  US_init();
  init_IR_sensor();
  servo_setup(0);
  imu_init(GYRO_RANGE);
  calibrate_imu();

  uart_init_9600();

  _delay_ms(1000);

  fans_init();

  _delay_ms(1000);

  sei();
}

void init_IR_sensor()
{
  ADMUX |= (1 << REFS0);  // Use AVCC with external capacitor at AREF pin
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1);  // Set prescaler to 64
  ADMUX |= (1 << ADLAR); // Set result as left-adjusted
  ADCSRA |= (1 << ADEN);  // Enable ADC
}

// Read distance using an IR sensor connected to ADC0
void read_vertical_IR()
{
  ADMUX = (ADMUX & 0xF8); // Select ADC0 as input channel 
  ADCSRA |= (1 << ADSC);  // Start the conversion
  while (ADCSRA & (1 << ADSC)); // Wait while ADC conversion is taking place

  uint8_t reading = ADCH;

  if(reading > IR_READING_MIN && reading < IR_READING_MAX)
  {
    end_of_course = true;
  }
}

uint16_t find_gaps()
{
  uint16_t servo_pulse;
  uint16_t distance_ahead;
  
  // ***********************************
  // LOOK RIGHT
  // ***********************************
  set_servo_pulse(SERVO_RIGHT);
  _delay_ms(LOOK_TIME); // Make sure the servo's steady
  distance_ahead = read_distance_US();
  if(distance_ahead > US_CLEAR_AHEAD)
  {
    servo_pulse = SERVO_RIGHT;
  }
  else
  {
    // ***********************************
    // LOOK LEFT
    // ***********************************
    set_servo_pulse(SERVO_LEFT);
    _delay_ms(LOOK_TIME); // Make sure the servo's steady
    distance_ahead = read_distance_US();
    if(distance_ahead > US_CLEAR_AHEAD)
    {
      servo_pulse = SERVO_LEFT;
    }
    else
    {
      // ***********************************
      // SWEEP
      // *********************************** 
      uint16_t max_distance_pulse = 0, max_distance_ahead = 0; 
      for(int i = 0; i < 7; i++)
      {
        set_servo_pulse(servo_sweep_angles[SERVO_PULSE_VALUES][i]);
        _delay_ms(LOOK_TIME); // Make sure the servo's steady
        distance_ahead = read_distance_US();

        if(distance_ahead > max_distance_ahead)
        {
          max_distance_ahead = distance_ahead;
          max_distance_pulse = servo_sweep_angles[SERVO_PULSE_VALUES][i];
        }
      }
      servo_pulse = max_distance_pulse;
    }
    
  }

  set_servo_pulse(188); // Straighten
  _delay_ms(LOOK_TIME); // Make sure the servo's steady

  return servo_pulse;
}

void set_servo_to_yaw(float yaw) 
{
  int int_yaw = (int)yaw;
  if(int_yaw > -91 && int_yaw < 91)
  {
    set_servo_pulse(servo_pulses[int_yaw + 90]);
  }
}

// Find angle that corresponds to the servo pulse in the sweep angles array
float get_yaw_from_ticks(uint16_t ticks)
{
  for(int i = 0; i < 7; i++)
  {
    if(servo_sweep_angles[SERVO_PULSE_VALUES][i] == ticks)
    {
      return(float)(servo_sweep_angles[ANGLE_VALUES][i]);
    }
  }

  return 0;
}

void fans_init()
{
  
  DDRD  |= (1 << PORTD6); // Set up PD6 (thrust fan) as output
  DDRD  |= (1 << PORTD5); // Set up PD5 (lift fan) as output

  TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); // Set to fast pwm, non-inverting mode
  TCCR0B  |= (1 << CS00); // Set clock prescalar to NO PRESCALAR

  OCR0A = THRUST_FAN_MEDIUM;
  OCR0B = LIFT_FAN_SPEED;
}

void set_lift_fan_speed(uint8_t dutyCycle)
{
  OCR0B = dutyCycle;
}

void set_thrust_fan_speed(uint8_t dutyCycle)
{
  OCR0A = dutyCycle;
}

void reset_angles()
{
  roll = 0;
  pitch = 0;
  yaw = 0;
}

// Gradually straighten servo from current pulse
void straighten_servo(uint16_t current_pulse)
{
  if(current_pulse < SERVO_STRAIGHT)
  {
    for(int i = current_pulse; i < SERVO_STRAIGHT; i += 1)
    {
      set_servo_pulse(i);
      _delay_ms(10);
    }
  }
  else
  {
    for(int i = current_pulse; i > SERVO_STRAIGHT; i -= 1)
    {
      set_servo_pulse(i);
      _delay_ms(10);
    }
  }

  set_servo_pulse(SERVO_STRAIGHT);
}

void print_angles()
{
  uart_txString("Yaw is: ");
  if(yaw < 0) {
    uart_txChar('-');
    uart_txU16((uint16_t)(-yaw));
  } else {
    uart_txU16((uint16_t)yaw);
  }
  uart_txString(" degrees\n");
}