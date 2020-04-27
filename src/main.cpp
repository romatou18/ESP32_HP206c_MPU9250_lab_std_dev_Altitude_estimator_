/*
 * Demo name   : HP20x_dev demo 
 * Usage       : I2C PRECISION BAROMETER AND ALTIMETER [HP206C hopeRF] 
 * Author      : Oliver Wang from Seeed Studio
 * Version     : V0.1
 * Change log  : Add kalman filter 2014/04/04
*/
#include <Arduino.h>
#include <HP20x_dev.h>
#include "Arduino.h"
#include "Wire.h" 
#include <KalmanFilter.h>
#include <MPU9250_Passthru.h>
unsigned char ret = 0;

/* Instance */
grove::KalmanFilter t_filter;    //temperature filter
grove::KalmanFilter p_filter;    //pressure filter
grove::KalmanFilter a_filter;    //altitude filter

HP20x_dev HP20x(0,21,22, 400000U);

// Number of readings from which standard deviations will be computed
constexpr int iterations = 1000;
static float history[iterations];
static float meanPressure;
static int idx;

long alti_offset = 0.0;



// Acelerometer anf Gyrometer helper methods and variables

// Sensor scale settings
const MPUIMU::Ascale_t ASCALE = MPUIMU::AFS_8G;
const MPUIMU::Gscale_t GSCALE = MPUIMU::GFS_2000DPS;
const MPU9250::Mscale_t MSCALE = MPU9250::MFS_16BITS;
const MPU9250::Mmode_t MMODE = MPU9250::M_100Hz;
// SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so
// SAMPLE_RATE_DIVISOR = 0 means 1 kHz sample rate for both accel and gyro, 4 means 200 Hz, etc.
const uint8_t SAMPLE_RATE_DIVISOR = 0;
// MPU9250 add-on board has interrupt on Butterfly pin 8
const uint8_t INTERRUPT_PIN = 8;
static const uint8_t LED_PIN = 13; // red led


// Use the MPU9250 in pass-through mode
static MPU9250_Passthru imu(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);
// flag for when new data is received
bool gotNewData = false;

// void getPressure(float* p)
float getPressure()
{
  float pi =0.0;
    char display[40];
    if(OK_HP20X_DEV == ret)
    { 
	  Serial.println(F("------------------\n"));
	  long Temper = HP20x.ReadTemperature();
	  Serial.println(F("Temper:"));
	  float t = Temper/100.0;
	  Serial.print(t);	  
	  Serial.println(F("C.\n"));
	  Serial.println(F("Filter:"));
	  Serial.print(t_filter.Filter(t));
	  Serial.println(F("C.\n"));
 
      long Pressure = HP20x.ReadPressure();
	  Serial.println(F("Pressure:"));
	  // *p = Pressure/100.0;
    pi = Pressure/100.0;
	  // Serial.print(*p);
    Serial.print(pi);

	  Serial.println(F("hPa.\n"));
	  Serial.println(F("Filter:"));
	  // Serial.print(p_filter.Filter(*p));
    Serial.print(p_filter.Filter(pi));

	  Serial.println(F("hPa\n"));
	  
	  long Altitude = HP20x.ReadAltitude();
	  Serial.println(F("Altitude:"));
	  float a = Altitude/100.0;
	  Serial.print(a);
	  Serial.println(F("m.\n"));
	  Serial.println(F("Filter:"));
	  Serial.print(a_filter.Filter(t));
	  Serial.println(F("m.\n"));
	  Serial.println(F("------------------\n"));
      delay(50);
    }
    return pi;
}

// Function to compute barometer standard deviations
static float getBarometerSigma(int numberOfIterations)
{
   // Initialize barometer
//    MS5637 barometer = MS5637();
//    barometer.begin();
   // here we will store all pressure readings

    float history[numberOfIterations];
    long double meanPressure = 0;
    for (uint16_t idx = 0; idx < numberOfIterations; idx++) 
    {
        // getPressure(&p);
        float p = getPressure();
        Serial.print(F("instant pressure = "));
        Serial.println(p, 2);
        history[idx] = p;
        // we will use pressureSum to compute the mean pressure
        meanPressure += history[idx];
    }
    meanPressure /= numberOfIterations;
    // Compute standard deviation
    long double numerator = 0;
    for (uint16_t idx = 0; idx < numberOfIterations; idx++) {
      numerator += pow(history[idx] - meanPressure, 2);
    }
    return sqrt(numerator / (numberOfIterations - 1));
}


static void interruptHandler()
{
    gotNewData = true;
}

void getGyrometerAndAccelerometer(float gyro[3], float accel[3])
{
    if (gotNewData) {

        gotNewData = false;

        // if (imu.checkNewAccelGyroData()) {
          
          float ax, ay, az, gx, gy, gz;
          imu.readAccelerometer(ay, ax, az);
          imu.readGyrometer(gy, gx, gz);
          gx = -gx;
          // Copy gyro values back out in rad/sec
          gyro[0] = gx * M_PI / 180.0f;
          gyro[1] = gy * M_PI / 180.0f;
          gyro[2] = gz * M_PI / 180.0f;
          // and acceleration values
          accel[0] = ax;
          accel[1] = ay;
          accel[2] = az;

          Serial.print(F("readGyro[z]"));
          Serial.println(gyro[2], 2);
          Serial.print(F("readAccel[z]"));
          Serial.println(accel[2], 2);
          
        // } // if (imu.checkNewAccelGyroData())

    } // if gotNewData

}

static void error(const char * errmsg) 
{
    Serial.println(errmsg);
    while (true) ;
}

void setup()
{  
  idx = 0;
  meanPressure = 0.0;
  Serial.begin(38400);        // start serial for output
  
  Serial.println("****HP20x_dev demo by seeed studio****\n");
  Serial.println("Calculation formula: H = [8.5(101325-P)]/100 \n");
  /* Power up,delay 150ms,until voltage is stable */
  delay(150);
  /* Reset HP20x_dev */
  HP20x.begin();
  delay(100);
  
  /* Determine HP20x_dev is available or not */
  ret = HP20x.isAvailable();
  if(OK_HP20X_DEV == ret)
  {
    Serial.println(F("HP20x_dev is available.\n"));    
  }
  else
  {
    Serial.println(F("HP20x_dev isn't available.\n"));
  }


    Wire.begin(); 
    Wire.setClock(400000); 

    delay(100);

    // Start the MPU9250
    switch (imu.begin()) {

        case MPUIMU::ERROR_IMU_ID:
            error("Bad IMU device ID");
        case MPUIMU::ERROR_MAG_ID:
            error("Bad magnetometer device ID");
        case MPUIMU::ERROR_SELFTEST:
            //error("Failed self-test");
            break;
        default:
            Serial.println("MPU9250 online!\n");
    }

    delay(100);

    // Set up the interrupt pin, it's set as active high, push-pull
    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);  // define interrupt for INTERRUPT_PIN output of MPU9250


    // Start with orange led on (active HIGH)
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); 

    // Comment out if using pre-measured, pre-stored offset magnetometer biases
    //Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    //imu.calibrateMagnetometer();


    digitalWrite(LED_PIN, LOW); // turn off led when using flash memory
  
}
 

void loop()
{
  // if(idx < iterations)
  // {
  //   float p = getPressure();
  //   Serial.print(F("instant pressure = "));
  //   Serial.println(p, 2);
  //   history[idx] = p;
  //   // we will use pressureSum to compute the mean pressure
  //   meanPressure += history[idx];
  //   ++idx;
  // } else {
  //   meanPressure /= iterations;
  //   // Compute standard deviation
  //   Serial.print(F("meanPressure = "));
  //   Serial.println(meanPressure, 2);
  //   float numerator = 0;
  //   for(uint16_t i = 0; i < iterations; i++)
  //   {
  //     numerator += pow(history[i] - meanPressure, 2);
  //   }

  //   double standard_deviation = sqrt(numerator / (iterations - 1));
  //   Serial.print(F("baro std dev = "));
  //   Serial.println(standard_deviation, 2);
  //   Serial.print(F("meanPressure = "));
  //   Serial.print(meanPressure, 2);
  //   Serial.println(F("meanPressure"));
  //   delay(5000);
  //   meanPressure = 0;
  //   idx = 0;
  // }

  // float readGyro[3];
  // float readAccel[3];
  // getGyrometerAndAccelerometer(readGyro, readAccel);
 
  Serial.println("Computing Barometer standard deviation");
    float baroSigma = 0;
    baroSigma = getBarometerSigma(iterations);
    Serial.print("Barometer standard deviation: ");
    Serial.println(baroSigma, 15);
  
}