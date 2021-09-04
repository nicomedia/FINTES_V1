

#if 0
  uint8_t read_ok = 0;
  uint8_t radioCnt = 0;
  uint8_t radioValid = 0;
  uint8_t gpsOff =0 ;
while(1)
  {
    while (SerialGPS.available() >0) {
        gps.encode(SerialGPS.read());
        read_ok = 1;

        
    }

    if (read_ok)
    {
      loraData.gpsLat = gps.location.lat();
      loraData.gpsLong = gps.location.lng();

      Serial.println(loraData.gpsLat,6);
      Serial.println(loraData.gpsLong,6);
      if (loraData.gpsLat > 0 && loraData.gpsLong > 0)
      {
          radioValid++;
      }
    }
    if (radioValid > 5 && gpsOff==0)
    {
      uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
      SerialGPS.write(GPSoff, sizeof(GPSoff)/sizeof(uint8_t));
      Serial.println("GPS OFF");

      radioValid = 0;
      radioCnt = 0;
      gpsOff = 1;
    }

    if (gpsOff==1 && radioCnt > 4)
    {
      uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
      SerialGPS.write(GPSon, sizeof(GPSon)/sizeof(uint8_t));
      gpsOff = 0;
      Serial.println("GPS ON");
    }

    Serial.println("while");

    delay(5000);
    read_ok = 0;
    radioCnt++;
  }  

#endif