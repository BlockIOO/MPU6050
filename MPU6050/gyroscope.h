class gyroscope {
public:
  int ADDR = 0x68; // MPU6050 I2C address
  int ADDRgyro = 0x43; // MPU6050 acc address
  int ADDRgyroCONFIG=0x1B;
  int ADDRgyroSCALE=0x00;
  double div = 131.0;

  double GyroX, GyroY, GyroZ;
  double GyroErrorX, GyroErrorY, GyroErrorZ;

  gyroscope(int _ADDR=0x68, int _ADDRgyro=0x43, int _ADDRgyroCONFIG=0x1B) : ADDR(_ADDR), ADDRgyro(_ADDRgyro), ADDRgyroCONFIG(_ADDRgyroCONFIG) {
  }

  void config(int _ADDRgyroSCALE=0x00) {
    ADDRgyroSCALE = _ADDRgyroSCALE;
    div = 131.0 / (1<<((_ADDRgyroSCALE>>3)&3));
    // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
    Wire.beginTransmission(ADDR);
    Wire.write(ADDRgyroCONFIG);                   // Talk to the GYRO_CONFIG register (1B hex)
    Wire.write(ADDRgyroSCALE);                   // Set the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);
  }

  void get() {
    Wire.beginTransmission(ADDR);
    Wire.write(ADDRgyro); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    GyroX = (Wire.read() << 8 | Wire.read()) / div; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    GyroY = (Wire.read() << 8 | Wire.read()) / div;
    GyroZ = (Wire.read() << 8 | Wire.read()) / div;
  }

  void filter() {
    GyroX = GyroX-GyroErrorX;
    GyroY = GyroY-GyroErrorY;
    GyroZ = GyroZ-GyroErrorZ;
  }

  void getraw() {
    Wire.beginTransmission(ADDR);
    Wire.write(ADDRgyro); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    Serial.println();
    Serial.println(Wire.read() << 8 | Wire.read()); // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    Serial.println(Wire.read() << 8 | Wire.read());
    Serial.println(Wire.read() << 8 | Wire.read());
  }

  void print() {
    Serial.print(" ");
    Serial.print(GyroX);
    Serial.print(" ");
    Serial.print(GyroY);
    Serial.print(" ");
    Serial.print(GyroZ);
  }

  void calculate_error() {
    // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
    int c = 0;

    // Read gyro values 200 times
    while (c < 200) {
      get();
      // Sum all readings
      GyroErrorX += (GyroX / div);
      GyroErrorY += (GyroY / div);
      GyroErrorZ += (GyroZ / div);
      //GyroErrorX = GyroErrorX + (GyroX / div);
      //GyroErrorY = GyroErrorY + (GyroY / div);
      //GyroErrorZ = GyroErrorZ + (GyroZ / div);
      c++;
    }
    //Divide the sum by 200 to get the error value
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
    // Print the error values on the Serial Monitor
    /*
    Serial.print("GyroErrorX: ");
    Serial.println(GyroErrorX);
    Serial.print("GyroErrorY: ");
    Serial.println(GyroErrorY);
    Serial.print("GyroErrorZ: ");
    Serial.println(GyroErrorZ);
    */
  }
};