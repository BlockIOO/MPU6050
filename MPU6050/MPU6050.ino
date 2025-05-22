/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/
#include "imu.h"
#include "heartbeat.h"
// VCC 3.3v or 5v
// GND GND
// SDA A4
// SCL A5

imu mpu;

/*
void gyro_angle() {
  filterrpy();
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}

void printgyro_angle() {
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(yaw);
}
*/

void setup() {
  Serial.begin(19200);
  //Serial.println("Loading up");
  //Serial.println("Setting up");
  heartbeat_setup();
  //Serial.println("Setup Heartbeat");
  
  mpu.setup();
  
  mpu.acc.config(0<<3);
  mpu.gyro.config(0<<3);

  // Call this function if you need to get the IMU error values for your module
  //mpu.acc.calculate_error();
  mpu.gyro.calculate_error();

  delay(100);
  //Serial.println("setup all");
}

void loop() {
  mpu.acc.get();
  mpu.grav_rpangle();
  //mpu.grav_cosangle();
  mpu.gravq();

  //mpu.temp.get();

  mpu.gyro.get();
  mpu.gyro.filter();

  //mpu.delta_time();
  //mpu.integrate_angle();

  if (string_complete()) {
    // Print the values on the serial monitor
    mpu.acc.print();

    //mpu.printgrav_rpangle();

    //mpu.printgrav_cosangle();

    //Serial.print(" ");
    //mpu.q.print();

    //mpu.temp.print();

    Serial.print(" ");
    mpu.gyro.print();
    //mpu.gyro.getraw();

    Serial.println();
  }
  //delay(200);
}