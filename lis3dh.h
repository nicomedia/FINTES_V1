
#if 0
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
#endif

#include "SparkFunLIS3DH.h"
#include "Wire.h"
LIS3DH myIMU(I2C_MODE, 0x19); //Alternate constructor for I2C

void lis3DH_init()
{
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
}

void lis3DH_Fifo_Setup()
{
  myIMU.fifoBegin(); //Configure fifo
  myIMU.fifoClear();
  myIMU.fifoStartRec(); //cause fifo to start taking data (re-applies mode bits)
}

void lis3DH_Interrupt_Enable()
{
    myIMU.fifoInterruptEnable();
}

extern struct loraMsg loraData;
void lis3DH_Read()
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
      /*Serial.print(myIMU.readFloatAccelX());
      Serial.print(",");
      Serial.print(myIMU.readFloatAccelY());
      Serial.print(",");
      Serial.print(myIMU.readFloatAccelZ());
      Serial.println();*/
      loraData.ACC_X = myIMU.readFloatAccelX();
      loraData.ACC_Y = myIMU.readFloatAccelY();
      loraData.ACC_Z = myIMU.readFloatAccelZ();
    }
    
}