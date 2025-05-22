class acceleration {
public:
  int ADDR = 0x68; // MPU6050 I2C address
  int ADDRacc = 0x3B; // MPU6050 acc address
  int ADDRaccCONFIG=0x1C;
  int ADDRaccSCALE=0x00;
  double div = 16384.0;

  double x, y, z;
  double AccErrorX, AccErrorY, AccErrorZ;

  acceleration(int _ADDR=0x68, int _ADDRacc=0x3B, int _ADDRaccCONFIG=0x1C, int _ADDRaccSCALE=0x00) : ADDR(_ADDR), ADDRacc(_ADDRacc), ADDRaccCONFIG(_ADDRaccCONFIG) {
  }

  void config(int _ADDRaccSCALE=0x00) {
    ADDRaccSCALE = _ADDRaccSCALE;
    //div = 16384.0 >> ((_ADDRaccSCALE>>3)&3);
    div = 16384.0 / (1<<((_ADDRaccSCALE>>3)&3));
    // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
    Wire.beginTransmission(ADDR);
    Wire.write(ADDRaccCONFIG);                  //Talk to the ACCEL_CONFIG register (1C hex)
    Wire.write(ADDRaccSCALE);                  //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(true);
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  }

  void get() {
    // === Read acceleromter data === //
    Wire.beginTransmission(ADDR);
    Wire.write(ADDRacc); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    
    //Because gravity is acting at 9.81m/s^2. Some axis will have a constant acceleration of roughly 1g. When at an obleak angle, the magnitude of all 3 should add up to 1g.
    //The acceleration due to gravity makes it think it's going up because the components are being pulled down. This means you have +1,-3 range of motion
    x = (Wire.read() << 8 | Wire.read()) / div; // X-axis value
    y = (Wire.read() << 8 | Wire.read()) / div; // Y-axis value
    z = (Wire.read() << 8 | Wire.read()) / div - 0.25; // Z-axis value
    //z = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  }

  void print() {
    Serial.print(x);
    Serial.print(" ");
    Serial.print(y);
    Serial.print(" ");
    Serial.print(z);
  }

  void filter() {
    x = x-AccErrorX;
    y = y-AccErrorY;
    z = z-AccErrorZ;
  }

  void calculate_error() {
    // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
    // Read accelerometer values 200 times
    int c = 0;
    while (c < 200) {
      get();
      // Sum all readings
      AccErrorX += ((atan((y) / sqrt(pow((x), 2) + pow((z), 2))) * 180 / PI));
      AccErrorY += ((atan(-1 * (x) / sqrt(pow((y), 2) + pow((z), 2))) * 180 / PI));
      //AccErrorX = AccErrorX + ((atan((y) / sqrt(pow((x), 2) + pow((z), 2))) * 180 / PI));
      //AccErrorY = AccErrorY + ((atan(-1 * (x) / sqrt(pow((y), 2) + pow((z), 2))) * 180 / PI));
      //AccErrorX = AccErrorX + ((atan2((AccY) , sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
      //AccErrorY = AccErrorY + ((atan2(-1 * (AccX) , sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
      c++;
    }
    //Divide the sum by 200 to get the error value
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    // Print the error values on the Serial Monitor
    /*
    Serial.print("AccErrorX: ");
    Serial.println(AccErrorX);
    Serial.print("AccErrorY: ");
    Serial.println(AccErrorY);
    */
  }
};