#ifndef IMU_h
#define IMU_h

#include <inttypes.h>

#define SCL_CLOCK 100000UL  // 100kHz clock frequency for IMU MPU-6050
#define MPU_ADDRESS 0x68    // MPU's default address (from MPU-6050 datasheet)
#define ACCEL_XOUT_H 0x3B   // First byte of the 6 bytes storing acceleration data
#define GYRO_XOUT_H 0x43    // First byte of the 6 bytes storing gyro data
#define TEMP_OUT_H 0x41     // First byte of the 2 bytes storing temperature data
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B 
#define F_CPU 16000000UL    // 16MHz clock frequency (for ATmega328p)

void imu_init(uint16_t gyro_sensitivity);

void calibrate_imu(void); // Take initial measurements and use their average as an offset for future readings

uint8_t set_gyro_config(uint16_t range);  // Set range to ±250 deg/sec, ±500 deg/sec, ±1000 deg/sec, or ±2000 deg/sec,

void read_gyro(float *gx, float *gy, float *gz);

void update_gyro_angles(float dt, float *gyro_angle_x, float *gyro_angle_y, float *gyro_angle_z); // Update last angles reading. dt should be in seconds

#endif