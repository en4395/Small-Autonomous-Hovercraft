#include "IMU.h"
#include "TWI_290.h"
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <math.h>

const int CALIBRATION_SAMPLES = 300;

volatile float gyro_lsb_sensitivity;   // Determined by gyro configuration

volatile float gyro_x, gyro_y, gyro_z;
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;

void imu_init(uint16_t gyro_sensitivity) {
  // Initialise TWI
  TWSR = 0;                                           // no prescaler
  TWBR = (uint8_t)(((F_CPU / SCL_CLOCK) - 16) >> 1);  // TWI Bit Rate

  set_gyro_config(gyro_sensitivity);

  Write_Reg(MPU_ADDRESS, 0x6B, 0);  // PWR_MGMT_1 register set to 0 to wake up MPU
}

// I got the idea to calibrate from here: https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
// and from here: https://github.com/rfetick/MPU6050_light
void calibrate_imu() 
{
  // For gyro calibration
  float gyro_x = 0, gyro_y = 0, gyro_z = 0;
  float gx, gy, gz;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    // Increment gyro sums
    gyro_x += gx;
    gyro_y += gy;
    gyro_z += gz;

    _delay_ms(5);
  }

  // Update offset
  gyro_offset_x = gyro_x / CALIBRATION_SAMPLES;
  gyro_offset_y = gyro_y / CALIBRATION_SAMPLES;
  gyro_offset_z = gyro_z / CALIBRATION_SAMPLES;
}

uint8_t set_gyro_config(uint16_t range) 
{
  uint8_t status;

  switch (range) {
    case 250:
      status = Write_Reg(MPU_ADDRESS, GYRO_CONFIG, 0x00);  // Set range to ±250
      gyro_lsb_sensitivity = 131.0;
      break;
    case 500:
      status = Write_Reg(MPU_ADDRESS, GYRO_CONFIG, 0x08);  // Set range to ±500
      gyro_lsb_sensitivity = 65.5;
      break;
    case 1000:
      status = Write_Reg(MPU_ADDRESS, GYRO_CONFIG, 0x10);  // Set range to ±1000
      gyro_lsb_sensitivity = 32.8;
      break;
    case 2000:
      status = Write_Reg(MPU_ADDRESS, GYRO_CONFIG, 0x18);  // Set range to ±2000
      gyro_lsb_sensitivity = 16.4;
      break;
    default:
      status = 1;  // Invalid argument
  }

  return status;
}

void read_gyro(float *gx, float *gy, float *gz) 
{
  int16_t data_buffer[2];

  // Read the gyro x data
  Read_Reg_N(MPU_ADDRESS, 0x43, 2, data_buffer);  // 0x43 = GYRO_XOUT_H
  int16_t raw_data_x = (data_buffer[0] << 8) | (data_buffer[1] & 0xFF);

  // Read the gyro y data
  Read_Reg_N(MPU_ADDRESS, 0x45, 2, data_buffer);  // 0x45 = GYRO_YOUT_H
  int16_t raw_data_y = (data_buffer[0] << 8) | (data_buffer[1] & 0xFF);

  // Read the gyro z data
  Read_Reg_N(MPU_ADDRESS, 0x47, 2, data_buffer);  // 0x47 = GYRO_ZOUT_H
  int16_t raw_data_z = (data_buffer[0] << 8) | (data_buffer[1] & 0xFF);

  // Convert to degreees/sec using conversion factor
  *gx = raw_data_x / gyro_lsb_sensitivity - gyro_offset_x;
  *gy = raw_data_y / gyro_lsb_sensitivity - gyro_offset_y;
  *gz = raw_data_z / gyro_lsb_sensitivity - gyro_offset_z;
}

void update_gyro_angles(float dt, float *gyro_angle_x, float *gyro_angle_y, float *gyro_angle_z) 
{
  read_gyro(&gyro_x, &gyro_y, &gyro_z);

  *gyro_angle_x += gyro_x * dt;
  *gyro_angle_y += gyro_y * dt;
  *gyro_angle_z += (gyro_z + 1) * dt;
}