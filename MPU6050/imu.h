#include <Wire.h>
//#include "acceleration.h"
#include "temperature.h"
//#include "gyroscope.h"
#include "quaternion.h"
#include "triple_axis.h"

class imu {
public:
  int ADDR = 0x68; // MPU6050 I2C address
  int ADDRacc = 0x3B; // MPU6050 acc address
  int ADDRaccCONFIG=0x1C;
  int ADDRaccSCALE=0x00;

  int ADDRtemp = 0x41; // MPU6050 temp address

  int ADDRgyro = 0x43; // MPU6050 gyro address
  int ADDRgyroCONFIG=0x1C;
  int ADDRgyroSCALE=0x00;

  double accAngleX, accAngleY, accAngleZ;
  double accAnglecos, accUX, accUY, accUZ;;

  double vx, vy, vz;
  double x, y, z;
  
  double gyroAngleX, gyroAngleY, gyroAngleZ;
  double roll, pitch, yaw;
  
  unsigned long currentTime, previousTime;
  //float currentTime, previousTime;
  float elapsedTime;

  //acceleration acc;
  triple_axis acc;
  temperature temp;
  //gyroscope gyro;
  triple_axis gyro;

  quaternion q;

  imu(int _ADDR=0x68, int _ADDRacc=0x3B, int _ADDRtemp=0x41, int _ADDRgyro=0x43, int _ADDRaccCONFIG=0x1C, int _ADDRgyroCONFIG=0x1B) : ADDR(_ADDR), ADDRacc(_ADDRacc), ADDRtemp(_ADDRtemp), ADDRgyro(_ADDRgyro), ADDRaccCONFIG(_ADDRaccCONFIG), ADDRgyroCONFIG(_ADDRgyroCONFIG) {
  }

  void setup(int _ADDR=0x68, int _ADDRacc=0x3B, int _ADDRtemp=0x41, int _ADDRgyro=0x43, int _ADDRaccCONFIG=0x1C, int _ADDRgyroCONFIG=0x1B) {
    ADDR = _ADDR;
    ADDRacc = _ADDRacc;
    ADDRtemp = _ADDRtemp;
    ADDRgyro = _ADDRgyro;
    ADDRaccCONFIG = _ADDRaccCONFIG;
    ADDRgyroCONFIG = _ADDRgyroCONFIG;
    Wire.begin();                      // Initialize comunication
    Wire.beginTransmission(ADDR);       // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);                  // Talk to the register 6B
    Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);        //end the transmission

    //acc =  acceleration(ADDR, ADDRacc, ADDRaccCONFIG);
    acc =  triple_axis(ADDR, ADDRacc, ADDRaccCONFIG, 16384.0);
    temp = temperature(ADDR, ADDRtemp);
    //gyro = gyroscope(ADDR, ADDRgyro, ADDRgyroCONFIG);
    gyro = triple_axis(ADDR, ADDRgyro, ADDRgyroCONFIG, 131.0);

    // Call this function if you need to get the IMU error values for your module

    //acc.calculate_acc_error();
    //gyro.calculate_gyro_error();
    //temp.calculate_temp_error();
    //Serial.println("setup imu");

    delay(20);
  }

  void printxyz() {
    acc.print();
  }

  void printtemp() {
    temp.print();
  }

  void printrpy() {
    gyro.print();
  }

  double deg2rad(double deg) {
    return PI*deg/180.0;
  }

  double rad2deg(double rad) {
    return 180.0*rad/PI;
  }

  void grav_rpangle() {
      // Calculating Roll and Pitch from the accelerometer data
    //AccErrorX = ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    //AccErrorY = ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    
    accAngleX = (atan(acc.y / sqrt(pow(acc.x, 2) + pow(acc.z, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    accAngleY = (atan(-1 * acc.x / sqrt(pow(acc.y, 2) + pow(acc.z, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
    if (acc.z < 0) {
      accAngleX = 180 + accAngleX;
      accAngleY = 180 - accAngleY;
    }
  }

  void printgrav_rpangle() {
    Serial.print(" ");
    Serial.print(accAngleX);
    Serial.print(" ");
    Serial.print(accAngleY);
  }

  void grav_cosangle() {
      // Calculating Roll and Pitch from the accelerometer data
    //accAnglecos = (acos(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    double mag = sqrt(pow(acc.x, 2) + pow(acc.y, 2) + pow(acc.z, 2));
    accUX = (acc.x / mag); // AccErrorY ~(-1.58)
    accUY = (acc.y / mag); // AccErrorY ~(-1.58)
    accUZ = (acc.z / mag); // AccErrorY ~(-1.58)
    accAnglecos = acos(accUZ/mag);// - acc.AccErrorZ; // AccErrorz ~(0.58) See the calculate_IMU_error()custom function for more details
    //quaternion q = cos2quat(accAnglecos, accUX, accUY, accUZ);
    q = cos2quat(accAnglecos, accUX, accUY, accUZ);
  }

  void printgrav_cosangle() {
    Serial.print(" ");
    Serial.print(accAnglecos);
    Serial.print(" ");
    Serial.print(accUX);
    Serial.print(" ");
    Serial.print(accUY);
    Serial.print(" ");
    Serial.print(accUZ);
  }
  
  void gravq() {
    if (acc.z < 0) {
      q = quaternion(-acc.y/sqrt(2.0*(1.0-acc.z)), sqrt((1.0-acc.z)/2.0), 0, acc.x/sqrt(2.0*(1.0-acc.z)));
    }
    else {
      q = quaternion(sqrt((1.0+acc.z)/2.0), -acc.y/sqrt(2.0*(1.0+acc.z)), acc.x/sqrt(2.0*(1.0+acc.z)), 0);
    }
  }

  void delta_time() {
    // === Read gyroscope data === //
    previousTime = currentTime;        // Previous time is stored before the actual time read
    currentTime = millis();            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000.0; // Divide by 1000 to get seconds
  }
  
  void integrate_vel() {
  }

  void integrate_pos() {
    x += vx;
    y += vy;
    z += vz;
  }

  void integrate_angle() {
    /*
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    yaw =  yaw + GyroZ * elapsedTime;

    // Complementary filter - combine acceleromter and gyro angle values
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
    */
    
    //q = q.qrotate(xyzdeg2quat(gyro.GyroX*elapsedTime, gyro.GyroY*elapsedTime, gyro.GyroZ*elapsedTime));
    q = q.qrotate(xyzdeg2quat(gyro.x*elapsedTime, gyro.y*elapsedTime, gyro.z*elapsedTime));
  }

  void printgyro_angle() {
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(yaw);
  }

  void get_callibration() {
    // Get measurements for all scales
    int c = 0;
    while (c < 3) {
      acc.config(c<<3);
      acc.getraw();
      acc.printraw();
      Serial.print(" ");
      c += 1;
    }
    acc.config(0);
    c = 0;
    while (c < 3) {
      gyro.config(c<<3);
      gyro.getraw();
      gyro.printraw();
      Serial.print(" ");
      c += 1;
    }
    gyro.config(0);
  }

};