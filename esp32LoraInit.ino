#include "gps.h"
//#include "lis3dh.h"
#include "types.h"
#include "lora.h"


#include <WiFi.h>
#include <BluetoothSerial.h>
#include "driver/adc.h"
#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp_sleep.h>

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;



/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

BluetoothSerial SerialBT;
void disableWiFi(){
    adc_power_off();
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
    Serial.println("");
    Serial.println("WiFi disconnected!");
}
void disableBluetooth(){
    // Quite unusefully, no relevable power consumption
    btStop();
    Serial.println("");
    Serial.println("Bluetooth stop!");
}
 
void setModemSleep() {
    disableWiFi();
    disableBluetooth();
    setCpuFrequencyMhz(40);
    // Use this if 40Mhz is not supported
    // setCpuFrequencyMhz(80);
}

/*#include "SparkFunLIS3DH.h"
#include "Wire.h"*/
#include "lis3dh.h"
//LIS3DH myIMU(I2C_MODE, 0x19); //Alternate constructor for I2C

void setup()
{
    while(1)
    {
      pinMode(13, INPUT);
      setModemSleep();
      Serial.begin(115200); //Serial port of USB

      lis3DH_init();
      if (bootCount != 0)
      {
        lis3DH_Read();
      }
      lis3DH_Fifo_Setup();
      lis3DH_Interrupt_Enable();

      Serial.println("GPS ON");
      SerialGPS.begin(9600, SERIAL_8N1, RXPin, TXPin);
      uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
      SerialGPS.write(GPSon, sizeof(GPSon)/sizeof(uint8_t));

      uint32_t gpsReadCnt = 0;
      uint8_t read_ok = 0;
      while (1)
      {
        if (bootCount == 0)
        {
          if (gpsReadCnt > 300)
          {
            break;
          }
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
                Serial.println("must be valid");
                break;
            }
          }
        }
        else
        {
          if (gpsReadCnt > 100)
          {
            break;
          }
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
                Serial.println("must be valid");
                break;
            }
          }
        }
        delay(100);
        gpsReadCnt++;
      }
      Serial.println("GPS Off");
      uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
      SerialGPS.write(GPSoff, sizeof(GPSoff)/sizeof(uint8_t));
      SerialGPS.flush();


      if(bootCount == 0)
      {
        loraInit();
      }
      else
      {
        loraSend();
      }
      

      ++bootCount;
      Serial.println("Boot number: " + String(bootCount));
      //Print the wakeup reason for ESP32
      //print_wakeup_reason();

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

      /* Serial.println("Going to sleep now");
      delay(1000);
      Serial.flush(); */
      //esp_deep_sleep_start();
      esp_light_sleep_start();
      //Serial.println("This will never be printed");
    }

}

void loop()
{

}


