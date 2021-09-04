

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



  #if 0

   pinMode(13, INPUT);
  //gpio_set_interrupt(13, GPIO_INTTYPE_EDGE_POS, int_signal_handler);



   myIMU.settings.adcEnabled = 0;
  //Note:  By also setting tempEnabled = 1, temperature data is available
  //instead of ADC3 in.  Temperature *differences* can be read at a rate of
  //1 degree C per unit of ADC3 data.
  myIMU.settings.tempEnabled = 1;
  myIMU.settings.accelSampleRate = 1;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  myIMU.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.xAccelEnabled = 1;
  myIMU.settings.yAccelEnabled = 1;
  myIMU.settings.zAccelEnabled = 1;

  //FIFO control settings
  myIMU.settings.fifoEnabled = 1;
  myIMU.settings.fifoThreshold = 30;  //Can be 0 to 32
  myIMU.settings.fifoMode = 3;  //FIFO mode.
  //fifoMode can be:
  //  0 (Bypass mode, FIFO off)
  //  1 (FIFO mode)
  //  3 (FIFO until full)
  //  4 (FIFO when trigger)
  
  //Call .begin() to configure the IMU (except for the fifo)
  myIMU.begin();

  if (bootCount != 0)
  {
      //float temp;  //This is to hold read data
    //uint16_t tempUnsigned;
    //
    while(( myIMU.fifoGetStatus() & 0x80 ) == 0) {};  //Wait for watermark
    
    //Now loop until FIFO is empty.
    //If having problems with the fifo not restarting after reading data, use the watermark
    //bits (b5 to b0) instead.
    //while(( myIMU.fifoGetStatus() & 0x1F ) > 2) //This checks that there is only a couple entries left
    while(( myIMU.fifoGetStatus() & 0x20 ) == 0) //This checks for the 'empty' flag
    {
      Serial.print(sampleNumber);
      Serial.print(",");
      Serial.print(myIMU.readFloatAccelX());
      Serial.print(",");
      Serial.print(myIMU.readFloatAccelY());
      Serial.print(",");
      Serial.print(myIMU.readFloatAccelZ());
      Serial.println();
      sampleNumber++;
    }
  }
   

  Serial.print("Configuring FIFO with no error checking...");
  myIMU.fifoBegin(); //Configure fifo
  Serial.print("Done!\n");
  
  Serial.print("Clearing out the FIFO...");
  myIMU.fifoClear();
  Serial.print("Done!\n");
  myIMU.fifoStartRec(); //cause fifo to start taking data (re-applies mode bits)
  myIMU.fifoInterruptEnable();


  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up for an external trigger.
  There are two types for ESP32, ext0 and ext1 .
  ext0 uses RTC_IO to wakeup thus requires RTC peripherals
  to be on while ext1 uses RTC Controller so doesnt need
  peripherals to be powered on.
  Note that using internal pullups/pulldowns also requires
  RTC peripherals to be turned on.
  */
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,1); //1 = High, 0 = Low

#if 0
  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");
#endif
  /*
  Next we decide what all peripherals to shut down/keep on
  By default, ESP32 will automatically power down the peripherals
  not needed by the wakeup source, but if you want to be a poweruser
  this is for you. Read in detail at the API docs
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  Left the line commented as an example of how to configure peripherals.
  The line below turns off all RTC peripherals in deep sleep.
  */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /*
  Now that we have setup a wake cause and if needed setup the
  peripherals state in deep sleep, we can now start going to
  deep sleep.
  In the case that no wake up sources were provided but deep
  sleep was started, it will sleep forever unless hardware
  reset occurs.
  */
  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  esp_deep_sleep_start();
  Serial.println("This will never be printed");


#endif

#if 0

  if ( digitalRead(13) == 1 )
  {
    //float temp;  //This is to hold read data
  //uint16_t tempUnsigned;
  //
  while(( myIMU.fifoGetStatus() & 0x80 ) == 0) {};  //Wait for watermark
  
  //Now loop until FIFO is empty.
  //If having problems with the fifo not restarting after reading data, use the watermark
  //bits (b5 to b0) instead.
  //while(( myIMU.fifoGetStatus() & 0x1F ) > 2) //This checks that there is only a couple entries left
  while(( myIMU.fifoGetStatus() & 0x20 ) == 0) //This checks for the 'empty' flag
  {
	  Serial.print(sampleNumber);
	  Serial.print(",");
	  Serial.print(myIMU.readFloatAccelX());
	  Serial.print(",");
	  Serial.print(myIMU.readFloatAccelY());
	  Serial.print(",");
	  Serial.print(myIMU.readFloatAccelZ());
	  Serial.println();
	  sampleNumber++;
  }
  }

  #endif