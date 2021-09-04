
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

typedef struct {
	float CALIB_X;
	float CALIB_Y;
	float CALIB_Z;
} Calibration_t;

Calibration_t calibAcc;

typedef struct {
  int16_t AXIS_X;
  int16_t AXIS_Y;
  int16_t AXIS_Z;
} AxesRaw_t;

AxesRaw_t rawAcc[31];

extern RTC_DATA_ATTR int bootCount;

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

    memset(&rawAcc[0], 0, sizeof(AxesRaw_t) * 31);
    calibAcc.CALIB_X = 0;
    calibAcc.CALIB_Y = 0;
    calibAcc.CALIB_Z = 0; 
}

void lis3DH_Fifo_Setup()
{
  myIMU.fifoBegin(); //Configure fifo
  myIMU.fifoClear();
  myIMU.fifoStartRec(); //cause fifo to start taking data (re-applies mode bits)

}

void lis3DH_Calibration()
{
      while(( myIMU.fifoGetStatus() & 0x80 ) == 0) {};  //Wait for watermark

    int calibCnt = 0;
    while(( myIMU.fifoGetStatus() & 0x20 ) == 0) //This checks for the 'empty' flag
    {
      calibAcc.CALIB_X += myIMU.readFloatAccelX();
      calibAcc.CALIB_Y += myIMU.readFloatAccelY();
      calibAcc.CALIB_Z += myIMU.readFloatAccelZ();
      calibCnt++;
    }
    calibAcc.CALIB_X = calibAcc.CALIB_X / calibCnt;
    calibAcc.CALIB_Y = calibAcc.CALIB_Y / calibCnt;
    calibAcc.CALIB_Z = calibAcc.CALIB_Z / calibCnt;
    Serial.println(calibAcc.CALIB_X);
    Serial.println(calibAcc.CALIB_Y);
    Serial.println(calibAcc.CALIB_Z);
}

void lis3DH_Interrupt_Enable()
{
    myIMU.fifoInterruptEnable();
}

extern struct loraMsg loraData;

float accX = 0;
float accXOld = 0;
float VelocityX = 0;
float VelocityXOld = 0;
float PositionX = 0;
float PositionXOld = 0;
float TotalX = 0;

float accY = 0;
float accYOld = 0;
float VelocityY = 0;
float VelocityYOld = 0;
float PositionY = 0;
float PositionYOld = 0;
float TotalY = 0;

float accZ = 0;
float accZOld = 0;
float VelocityZ = 0;
float VelocityZOld = 0;
float PositionZ = 0;
float PositionZOld = 0;
float TotalZ = 0;
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
    if (bootCount == 1)
    {
        accXOld = calibAcc.CALIB_X;
        accYOld = calibAcc.CALIB_Y;
        accZOld = calibAcc.CALIB_Z;
    }
    
    int accCounter = 0;
    while(( myIMU.fifoGetStatus() & 0x20 ) == 0) //This checks for the 'empty' flag
    {
            accX = myIMU.readFloatAccelX();
            accX = accX - calibAcc.CALIB_X;
            if (abs(accX - accXOld) > 0.02)
            {
                VelocityX = VelocityXOld + abs(accX - accXOld) * 1;
                PositionX = PositionXOld + ((VelocityXOld + VelocityX) / 2);
                VelocityXOld = VelocityX;
                PositionXOld = PositionX;

                TotalX = TotalX + PositionX;
            }
            else
            {
                VelocityXOld = 0;
			    PositionXOld = 0;
            }
            
        


            accY = myIMU.readFloatAccelY();
            accY = accY - calibAcc.CALIB_Y;
            if (abs(accY - accYOld) > 0.02)
            {
                VelocityY = VelocityYOld + abs(accY - accYOld) * 1;
                PositionY = PositionYOld + ((VelocityYOld + VelocityY) / 2);
                VelocityYOld = VelocityY;
                PositionYOld = PositionY;

                TotalY = TotalY + PositionY;
            }
            else
            {
                VelocityYOld = 0;
			    PositionYOld = 0;
            }

            accZ = myIMU.readFloatAccelZ();
            accZ = accY - calibAcc.CALIB_Z;
            if (abs(accZ - accZOld) > 0.02)
            {
                VelocityZ = VelocityZOld + abs(accZ - accZOld) * 1;
                PositionZ = PositionZOld + ((VelocityZOld + VelocityZ) / 2);
                VelocityZOld = VelocityZ;
                PositionZOld = PositionZ;

                TotalZ = TotalZ + PositionZ;
            }
            else
            {
                VelocityZOld = 0;
			    PositionZOld = 0;
            }
            
        
         



      /*Serial.print(myIMU.readFloatAccelX());
      Serial.print(",");
      Serial.print(myIMU.readFloatAccelY());
      Serial.print(",");
      Serial.print(myIMU.readFloatAccelZ());
      Serial.println();*/
      /*loraData.ACC_X = myIMU.readFloatAccelX();
      loraData.ACC_Y = myIMU.readFloatAccelY();
      loraData.ACC_Z = myIMU.readFloatAccelZ();
      Serial.println(loraData.ACC_X);
      Serial.println(loraData.ACC_Y);
      Serial.println(loraData.ACC_Z);*/
    }
    accXOld = accX;
    accYOld = accY;
    accZOld = accZ;

    Serial.println(TotalX);
    Serial.println(TotalY);
    Serial.println(TotalZ);
}