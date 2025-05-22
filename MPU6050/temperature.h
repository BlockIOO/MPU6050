class temperature {
public:
  int ADDR = 0x68; // MPU6050 I2C address
  int ADDRtemp = 0x41; // MPU6050 temp address

  float TempC;
  float TempErrorC;

  temperature(int _ADDR=0x68, int _ADDRtemp=0x41) : ADDR(_ADDR), ADDRtemp(_ADDRtemp) {
  }
  
  void get() {
    // === Read thermometer data === //
    Wire.beginTransmission(ADDR);
    Wire.write(ADDRtemp); // Start with register 0x41 (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR, 2, true); // Read 2 registers total, each axis value is stored in 2 registers
    
    TempC = (Wire.read() << 8 | Wire.read()) / 340.0 + 36.53;
  }

  void filter() {
    TempC = TempC-TempErrorC;
  }

  void print() {
    Serial.print(" ");
    Serial.print(TempC);
  }

  void calculate_temp_error() {
    // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
    int c = 0;

    // Read temperature values 200 times
    while (c < 200) {
      get();
      // Sum all readings
      //TempErrorC = TempErrorC + TempC;
      TempErrorC += + TempC;
      c++;
    }
    //Divide the sum by 200 to get the error value
    TempErrorC = TempErrorC / 200;
    // Print the error values on the Serial Monitor
    /*
    Serial.print("TempErrorC: ");
    Serial.println(TempErrorC);
    */
  }
};