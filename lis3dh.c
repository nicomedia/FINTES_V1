

#if 0

setup


  if( myIMU.begin(sampleRate, 1, 1, 1, accelRange) != 0 )
  {
    Serial.print("Failed to initialize IMU.\n");
  }
  else
  {
    Serial.print("IMU initialized.\n");
  }

//Detection threshold can be from 1 to 127 and depends on the Range
  //chosen above, change it and test accordingly to your application
  //Duration = timeDur x Seconds / sampleRate
  myIMU.intConf(INT_1, DET_MOVE, 13, 2);
  myIMU.intConf(INT_2, DET_STOP, 13, 10, 1);  // also change the polarity to active-low, this will change both Interrupts behavior

  uint8_t readData = 0;

  // Confirm configuration:
  myIMU.readRegister(&readData, LIS3DH_INT1_CFG);
  myIMU.readRegister(&readData, LIS3DH_INT2_CFG);

  // Get the ID:
  myIMU.readRegister(&readData, LIS3DH_WHO_AM_I);
  Serial.print("Who am I? 0x");
  Serial.println(readData, HEX);

  loop

  int16_t dataHighres = 0;

  if( myIMU.readRegisterInt16( &dataHighres, LIS3DH_OUT_X_L ) != 0 )
  {
    errorsAndWarnings++;
  }

  if( myIMU.readRegisterInt16( &dataHighres, LIS3DH_OUT_Z_L ) != 0 )
  {
    errorsAndWarnings++;
  }

  loraData.ACC_X = myIMU.axisAccel( X );
  loraData.ACC_Y = myIMU.axisAccel( Y );
  loraData.ACC_Z = myIMU.axisAccel( Z );

  Serial.println(loraData.ACC_X);
  #endif