#include "TinyGPSPlus.h"
#include "HardwareSerial.h"

#define GPS_COLD_START  3000 /* 5 minutes */
#define GPS_WARM_START  200  /* 20 sencond */

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial SerialGPS(1);
extern RTC_DATA_ATTR int bootCount;


void gpsOn(void)
{
    uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
    SerialGPS.write(GPSon, sizeof(GPSon)/sizeof(uint8_t));
}

void gpsOff(void)
{
    uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
    SerialGPS.write(GPSoff, sizeof(GPSoff)/sizeof(uint8_t));
    SerialGPS.flush();
}

extern struct loraMsg loraData;
void gpsLoop(void)
{
    uint32_t gpsReadCnt = 0;
      uint8_t read_ok = 0;
      while (1)
      {

          if (gpsReadCnt > GPS_WARM_START)
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
        
        delay(100);
        gpsReadCnt++;
      }
}