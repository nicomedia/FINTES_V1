#include "types.h"
#include "lora.h"
#include "lis3dh.h"
#include "gps.h"

#include <DNSServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <BluetoothSerial.h>
#include "driver/adc.h"
#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp_sleep.h>

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

#define INIT_TIME      30000

RTC_DATA_ATTR int bootCount = 0;

DNSServer dnsServer;

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

IPAddress apStaticIp = IPAddress(192,168,4,1);  /**< Acces point ip address */
IPAddress apStaticGw = IPAddress(192,168,4,1);  /**< Access point gateway */
IPAddress apStaticSn = IPAddress(255,255,255,0);    /**< Access point subnet */

int loopCounter = 0;

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>


AsyncWebServer server(80);

void setup()
{
  Serial.begin(115200); //Serial port of USB

  Serial.println("FINTES begin" + String(COW_ID));

  WiFi.mode(WIFI_AP);
  String accesPoint = "FINTES_COW_" + String(COW_ID);
  WiFi.softAP(accesPoint.c_str(), "fintes1234"); 

  Serial.println(accesPoint + " is opened");
  WiFi.softAPConfig(apStaticIp, apStaticGw, apStaticSn);

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  int initCount = 0;  
  Serial.println("GPS Started");
  SerialGPS.begin(9600, SERIAL_8N1, RXPin, TXPin);
  gpsOn();
  uint8_t read_ok = 0;
  while(initCount < INIT_TIME)
  {
    
    dnsServer.processNextRequest();
    initCount++;
    delay(10);

    if(initCount%100 == 0)
    {
      Serial.println(initCount);
    }


    while (SerialGPS.available() >0) {
      gps.encode(SerialGPS.read());
      read_ok = 1;       
    }
    if (read_ok)
    {
      if(initCount%100 == 0)
      {
        loraData.gpsLat = gps.location.lat();
        loraData.gpsLong = gps.location.lng();
      }
      

      Serial.println(loraData.gpsLat,6);
      Serial.println(loraData.gpsLong,6);
      if (loraData.gpsLat > 0 && loraData.gpsLong > 0)
      {
        Serial.println("must be valid");
        break;
      }
    }
  }

  Serial.println("Init Phase Done");


    while(1)
    {
      pinMode(13, INPUT);
      setModemSleep();
      Serial.begin(115200); //Serial port of USB

      if (bootCount == 0)
        lis3DH_init();  
      else
        lis3DH_Read();
      
      lis3DH_Fifo_Setup();
      if(bootCount == 0)
        lis3DH_Calibration();
      lis3DH_Interrupt_Enable();

      
      if (loopCounter%2 == 0)
      {
        SerialGPS.begin(9600, SERIAL_8N1, RXPin, TXPin);

        Serial.println("GPS ON");
        gpsOn();

        gpsLoop();
        
        Serial.println("GPS Off");
        gpsOff();


        if(bootCount == 0)
        {
          loraInit();
        }
        else
        {
          loraSend();
        }
      }
      
      if(bootCount >= 3)
      {
        loopCounter++;
      }
      

      ++bootCount;
      Serial.println("Boot number: " + String(bootCount));
      //Print the wakeup reason for ESP32
      //print_wakeup_reason();

      esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,1); //1 = High, 0 = Low
      esp_light_sleep_start();

    }

}

void loop()
{

}


