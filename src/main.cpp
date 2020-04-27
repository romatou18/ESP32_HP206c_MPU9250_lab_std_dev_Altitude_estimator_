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
#include <altitude.h>


TaskHandle_t *Task1;

float pastTime = millis();
float currentTime = millis();

// Altitude estimator
constexpr long alti_offset = 23.0;

static AltitudeEstimator altitude = AltitudeEstimator(
  0.005, // sigma Accel
        0.001, // sigma Gyro
        0.10,   // sigma Baro
        0.5, // ca
        1.0);// accelThreshold

unsigned char ret = 0;

/* Instance */
grove::KalmanFilter t_filter;    //temperature filter
grove::KalmanFilter p_filter;    //pressure filter
grove::KalmanFilter a_filter;    //altitude filter

typedef struct {
  float t;
  float tf;
  float p;
  float pf;
  float a;
  float af;
} baro_reading_t;

HP20x_dev HP20x(0,21,22, 400000U);

// Number of readings from which standard deviations will be computed
constexpr int iterations = 1000;



// Acelerometer anf Gyrometer helper methods and variables

// Sensor scale settings
const MPUIMU::Ascale_t ASCALE = MPUIMU::AFS_8G;
const MPUIMU::Gscale_t GSCALE = MPUIMU::GFS_2000DPS;
const MPU9250::Mscale_t MSCALE = MPU9250::MFS_16BITS;
const MPU9250::Mmode_t MMODE = MPU9250::M_100Hz;
// SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so
// SAMPLE_RATE_DIVISOR = 0 means 1 kHz sample rate for both accel and gyro, 4 means 200 Hz, etc.
const uint8_t SAMPLE_RATE_DIVISOR = 4;
// MPU9250 add-on board has interrupt on Butterfly pin 8
const uint8_t INTERRUPT_PIN = 4;
static const uint8_t LED_PIN = 13; // red led


// Use the MPU9250 in pass-through mode
static MPU9250_Passthru imu(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);
// flag for when new data is received
bool gotNewData = false;


// helper variables and functions for obtaining baro data
static const uint8_t HISTORY_SIZE = 48;

static float   groundAltitude = 0;
static float   groundPressure = 0;
static float   pressureSum = 0;
static float   history[HISTORY_SIZE];
static uint8_t historyIdx = 0;
static uint32_t endCalibration = 120;


// Pressure in millibars to altitude in meters. We assume
// millibars are the units of the pressure readings from the sensor
static float millibarsToMeters(float mbar)
{
    // see: https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
    return (1.0f - powf(mbar / 1013.25f, 0.190295f)) * 44330.0f;
}
// Calibrate the baro by setting the average measured pressure as the
// ground pressure and the corresponding altitude as the ground altitude.
static void calibrate(float pressure)
{
    // Update pressure history
    history[historyIdx] = pressure;
    pressureSum += pressure;
    // cycle the index throught the history array
    uint8_t nextIndex = (historyIdx + 1) % HISTORY_SIZE;
    // Remove next reading from sum so that pressureSum is kept in sync
    pressureSum -= history[nextIndex];
    historyIdx = nextIndex;
    // groundPressure will stabilize at 8 times the average measured
    // pressure (when groundPressure/8 equals pressureSum/(HISTORY_SIZE-1))
    // This acts as a low pass filter and helps to reduce noise
    groundPressure -= groundPressure / 8;
    groundPressure += pressureSum / (HISTORY_SIZE - 1);
    groundAltitude = millibarsToMeters(groundPressure/8);
}
// void getPressure(float* p)
void getPressure(baro_reading_t& read)
{
  float pi =0.0;
  char display[40];
  if(OK_HP20X_DEV == ret)
  { 
	  Serial.println(F("------------------\n"));
	  long Temper = HP20x.ReadTemperature();
	  Serial.println(F("Temper:"));

	  float t = Temper/100.0;
    read.t = t;

	  Serial.print(t);	  
	  Serial.println(F("C.\n"));
	  Serial.println(F("Filter:"));

    auto tf = t_filter.Filter(t);
    read.tf = tf;

	  Serial.print(tf);
	  Serial.println(F("C.\n"));
 
    long Pressure = HP20x.ReadPressure();
	  Serial.println(F("Pressure:"));
	  // *p = Pressure/100.0;
    pi = Pressure/100.0;
	  // Serial.print(*p);

    Serial.print(pi);
    read.p = pi;

	  Serial.println(F("hPa.\n"));
	  Serial.println(F("Filter:"));
	  // Serial.print(p_filter.Filter(*p));

    auto pf = p_filter.Filter(pi);
    read.pf = pf;

    Serial.print(pf);
	  Serial.println(F("hPa\n"));
	  
	  long Altitude = HP20x.ReadAltitude();
	  Serial.println(F("Altitude:"));
	  float a = Altitude/100.0;
    read.a = a;
	  Serial.print(a);
	  Serial.println(F("m.\n"));
	  Serial.println(F("Filter:"));

    auto af = a_filter.Filter(t);
	  Serial.print(af);
    read.af = af;

	  Serial.println(F("m.\n"));
	  Serial.println(F("------------------\n"));
    delay(50);
    }
}

// Function to compute barometer standard deviations
static float getBarometerSigma(int numberOfIterations)
{
    float history[numberOfIterations];
    long double meanPressure = 0;
    for (uint16_t idx = 0; idx < numberOfIterations; idx++) 
    {
        // getPressure(&p);
        baro_reading_t r;
        getPressure(r);
        Serial.print(F("instant pressure = "));
        Serial.println(r.p, 2);
        history[idx] = r.p;
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

          // Serial.print(F("readGyro[z]"));
          // Serial.println(gyro[2], 2);
          // Serial.print(F("readAccel[z]"));
          // Serial.println(accel[2], 2);
          
        // } // if (imu.checkNewAccelGyroData())

    } // if gotNewData
    // delay(5);
}

// Function to compute accelerometer and gyrometer standard deviations
void getAccelAndGyroSigmas(double* sigmaAccel, double* sigmaGyro, uint16_t numberOfIterations)
{
    // here we will store each accel axis' readings
    long double accelHistoryX[numberOfIterations];
    long double accelHistoryY[numberOfIterations];
    long double accelHistoryZ[numberOfIterations];
    // here we will store each gyro axis' readings
    long double gyroHistoryX[numberOfIterations];
    long double gyroHistoryY[numberOfIterations];
    long double gyroHistoryZ[numberOfIterations];
    long double meanAccelX = 0;
    long double meanAccelY = 0;
    long double meanAccelZ = 0;
    long double meanGyroX = 0;
    long double meanGyroY = 0;
    long double meanGyroZ = 0;

    for (uint16_t index = 0; index < numberOfIterations; index++) {
        float readGyro[3];
        float readAccel[3];
        getGyrometerAndAccelerometer(readGyro, readAccel);
        // store gyro readings
        gyroHistoryX[index] = readGyro[0];
        gyroHistoryY[index] = readGyro[1];
        gyroHistoryZ[index] = readGyro[2];
        // store accel readings
        accelHistoryX[index] = readAccel[0];
        accelHistoryY[index] = readAccel[1];
        accelHistoryZ[index] = readAccel[2];
        // increase mean sums
        meanGyroX += readGyro[0];
        meanGyroY += readGyro[1];
        meanGyroZ += readGyro[2];

        meanAccelX += readAccel[0];
        meanAccelY += readAccel[1];
        meanAccelZ += readAccel[2];
    }
    // Compute means
    meanGyroX /= numberOfIterations;
    meanGyroY /= numberOfIterations;
    meanGyroZ /= numberOfIterations;

    meanAccelX /= numberOfIterations;
    meanAccelY /= numberOfIterations;
    meanAccelZ /= numberOfIterations;

    // Compute standard deviations
    long double numeratorGyroX = 0;
    long double numeratorGyroY = 0;
    long double numeratorGyroZ = 0;

    long double numeratorAccelX = 0;
    long double numeratorAccelY = 0;
    long double numeratorAccelZ = 0;

    for (uint16_t index = 0; index < numberOfIterations; index++) {
      numeratorGyroX += pow(gyroHistoryX[index] - meanGyroX, 2);
      numeratorGyroY += pow(gyroHistoryY[index] - meanGyroY, 2);
      numeratorGyroZ += pow(gyroHistoryZ[index] - meanGyroZ, 2);

      numeratorAccelX += pow(accelHistoryX[index] - meanAccelX, 2);
      numeratorAccelY += pow(accelHistoryY[index] - meanAccelY, 2);
      numeratorAccelZ += pow(accelHistoryZ[index] - meanAccelZ, 2);
    }
    // Now, to compute on single standard deviation value for each
    // sensor, Gyro and Accel, we will take the maximum of the standard
    // deviations of each axis.
    long double gyroSigmaX = sqrt(numeratorGyroX / (numberOfIterations - 1));
    long double gyroSigmaY = sqrt(numeratorGyroY / (numberOfIterations - 1));
    long double gyroSigmaZ = sqrt(numeratorGyroZ / (numberOfIterations - 1));

    long double accelSigmaX = sqrt(numeratorAccelX / (numberOfIterations - 1));
    long double accelSigmaY = sqrt(numeratorAccelY / (numberOfIterations - 1));
    long double accelSigmaZ = sqrt(numeratorAccelZ / (numberOfIterations - 1));

    long double tmp  = max(gyroSigmaX, gyroSigmaY);
    *sigmaGyro =  max(tmp, gyroSigmaZ);
    tmp  = max(accelSigmaX, accelSigmaY);
    *sigmaAccel = max(tmp, accelSigmaZ);

}


static void error(const char * errmsg) 
{
    Serial.println(errmsg);
    while (true) ;
}

void init_sensors()
{
  Wire.begin(); 
  Wire.setClock(400000); 

  delay(100);

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


void main_task(void*)
{
  // IMPORTANT : put all the Bus related, device related init in the task itself instead of the setup()
  // setup() is run by CPU 1, so if ever things need to be run by CPU0 then all that is not init in the task itself won't work!
  init_sensors();

  while(1)
  {
    // Serial.println("Computing Barometer standard deviation");
    //     float baroSigma = 0;
    //     baroSigma = getBarometerSigma(iterations);
    //     Serial.print("Barometer standard deviation: ");
    //     Serial.println(baroSigma, 15);
    //   delay(5000);

    Serial.println("Computing Accelerometer and Gyrometer standard deviations");
    double accelSigma;
    double gyroSigma;
    getAccelAndGyroSigmas(&accelSigma, &gyroSigma, iterations);
    Serial.print("Accelerometer standard deviation: ");
    Serial.println(accelSigma, 15);
    Serial.print("Gyrometer standard deviation: ");
    Serial.println(gyroSigma, 15);

    delay(5000);
    
  }
}

static void imuRead(float gyro[3], float accel[3])
{
    getGyrometerAndAccelerometer(gyro, accel);
}

void setup()
{  

  // Set all pressure history entries to 0
  for (uint8_t k = 0; k < HISTORY_SIZE; ++k) {
      history[k] = 0;
  }

  Serial.begin(38400);        // start serial for output
  
  Serial.println("****HP20x_dev demo by seeed studio****\n");
  Serial.println("Calculation formula: H = [8.5(101325-P)]/100 \n");
  /* Power up,delay 150ms,until voltage is stable */
  delay(150);
   
  init_sensors();

  // calibrate barometer
  uint32_t count = 0;
  while (count < endCalibration) 
  {
      baro_reading_t r;
      getPressure(r);
      calibrate(r.p);
      count++;
  }

  Serial.print("Ground pressure = ");
  Serial.println(groundPressure/8);

  Serial.print("Ground Altitude = ");
  Serial.println(groundAltitude);
  delay(4000);

  
  // xTaskCreatePinnedToCore(
  //                     main_task,   /* Task function. */
  //                     "Task1",     /* name of task. */
  //                     50000,       /* Stack size of task */
  //                     NULL,        /* parameter of the task */
  //                     2,           /* priority of the task */
  //                     Task1,      /* Task handle to keep track of created task */
  //                     1);          /* pin task to core 0 */   
}
 



void loop()
{
 currentTime = millis();
  if ((currentTime - pastTime) > 500)
  {
    // // Check for new data in the FIFO
    // if ( imu.fifoAvailable() )
    // {
    //   // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    //   if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    //   {
    //     // computeEulerAngles can be used -- after updating the
    //     // quaternion values -- to estimate roll, pitch, and yaw
    //     imu.computeEulerAngles();
    //     imu.update(UPDATE_COMPASS);
    //     printIMUData();
    //     // delay(500);
    //   }
    // }
    // get all necessary data

    baro_reading_t r; 
    getPressure(r);
    // float Altitude = HP20x.ReadAltitude();
    Serial.print("Ground Altitude: ");
    float alti = r.a - groundAltitude;
    Serial.println(alti);

    uint32_t timestamp = micros();

    float accelData[3];
    float gyroData[3];

    imuRead(gyroData, accelData);

    Serial.print(F("readGyro[z]"));
    Serial.println(gyroData[2], 2);
    Serial.print(F("readAccel[z]"));
    Serial.println(accelData[2], 2);

    altitude.estimate(accelData, gyroData, alti , timestamp);
    Serial.print(alti);
    Serial.print(",est Alti=");
    Serial.print(altitude.getAltitude());
    Serial.print(",velocity=");
    Serial.print(altitude.getVerticalVelocity());
    Serial.print(",vert acc=");
    Serial.println(altitude.getVerticalAcceleration());




    pastTime = currentTime;
  }
}