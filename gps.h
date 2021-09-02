#include "TinyGPSPlus.h"
#include "HardwareSerial.h"

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial SerialGPS(1);