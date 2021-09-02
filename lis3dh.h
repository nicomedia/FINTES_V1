#include "lis3dh-motion-detection.h"
#include "Wire.h"

// Accelerometer provides different Power modes by changing output bit resolution
//#define LOW_POWER
#define NORMAL_MODE
//#define HIGH_RESOLUTION

// Enable Serial debbug on Serial UART to see registers wrote
#define LIS3DH_DEBUG Serial

uint16_t sampleRate = 1;  // HZ - Samples per second - 1, 10, 25, 50, 100, 200, 400, 1600, 5000
uint8_t accelRange = 2;   // Accelerometer range = 2, 4, 8, 16g

uint16_t errorsAndWarnings = 0;

LIS3DH myIMU(0x19); //Default address is 0x19.

