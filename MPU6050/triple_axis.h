class triple_axis {
public:
  int ADDR = 0x68; // MPU6050 I2C address
  int ADDRmeas = 0x3B; // MPU6050 acc address
  int ADDRCONFIG=0x1C;
  int ADDRSCALE=0x00;
  double start_div = 16384.0;
  double div = start_div;

  double x, y, z;
  int rawx, rawy, rawz;
  double ErrorX, ErrorY, ErrorZ;

  triple_axis(int _ADDR=0x68, int _ADDRmeas=0x3B, int _ADDRCONFIG=0x1C, double _start_div = 16384.0) : ADDR(_ADDR), ADDRmeas(_ADDRmeas), ADDRCONFIG(_ADDRCONFIG), start_div(_start_div) {
  }

  void config(int _ADDRSCALE=0x00) {
    ADDRSCALE = _ADDRSCALE;
    //div = 16384.0 >> ((_ADDRaccSCALE>>3)&3);
    div = start_div / (1<<((_ADDRSCALE>>3)&3));
    // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
    Wire.beginTransmission(ADDR);
    Wire.write(ADDRCONFIG);                  //Talk to the ACCEL_CONFIG register (1C hex)
    Wire.write(ADDRSCALE);                  //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(true);
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  }

  void get() {
    // === Read acceleromter data === //
    Wire.beginTransmission(ADDR);
    Wire.write(ADDRmeas); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    
    //Because gravity is acting at 9.81m/s^2. Some axis will have a constant acceleration of roughly 1g. When at an obleak angle, the magnitude of all 3 should add up to 1g.
    //The acceleration due to gravity makes it think it's going up because the components are being pulled down. This means you have +1,-3 range of motion
    x = (Wire.read() << 8 | Wire.read()) / div; // X-axis value
    y = (Wire.read() << 8 | Wire.read()) / div; // Y-axis value
    z = (Wire.read() << 8 | Wire.read()) / div; // Z-axis value
    //z = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  }

  void print() {
    Serial.print(x);
    Serial.print(" ");
    Serial.print(y);
    Serial.print(" ");
    Serial.print(z);
  }

  void getraw() {
    Wire.beginTransmission(ADDR);
    Wire.write(ADDRmeas); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    Serial.println();
    //Serial.println(Wire.read() << 8 | Wire.read()); // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    //Serial.println(Wire.read() << 8 | Wire.read());
    //Serial.println(Wire.read() << 8 | Wire.read());
    rawx = Wire.read() << 8 | Wire.read(); // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    rawy = Wire.read() << 8 | Wire.read();
    rawz = Wire.read() << 8 | Wire.read(); 
  }

  void printraw() {
    Serial.print(rawx);
    Serial.print(" ");
    Serial.print(rawy);
    Serial.print(" ");
    Serial.print(rawz);
  }

  void filter() {
    x -= ErrorX;
    y -= ErrorY;
    z -= ErrorZ;
  }

  void calculate_error() {
    // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
    int c = 0;

    // Read gyro values 200 times
    while (c < 200) {
      get();
      // Sum all readings
      ErrorX += (x / div);
      ErrorY += (y / div);
      ErrorZ += (z / div);
      //GyroErrorX = GyroErrorX + (GyroX / div);
      //GyroErrorY = GyroErrorY + (GyroY / div);
      //GyroErrorZ = GyroErrorZ + (GyroZ / div);
      c++;
    }
    //Divide the sum by 200 to get the error value
    ErrorX = ErrorX / 200;
    ErrorY = ErrorY / 200;
    ErrorZ = ErrorZ / 200;
    // Print the error values on the Serial Monitor
    /*
    Serial.print("ErrorX: ");
    Serial.println(ErrorX);
    Serial.print("ErrorY: ");
    Serial.println(ErrorY);
    Serial.print("ErrorZ: ");
    Serial.println(ErrorZ);
    */
  }
};