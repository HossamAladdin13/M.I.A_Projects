/*********************************************************************
 * M.I.A ROBOTICS TEAM
 * MPU6050 working with Kalman Filter 1D
 * Filtring measured YAW value from gyro
 * hello
 *********************************************************************/

// Kalman Filter Variables
double curr_estimate;
double prev_estimate;
double curr_uncertanity;
double prev_uncertanity;
double process_var;
double kalman_gain;
double sensor_varaiance;
double measured_value;

    /**** Preparing the IMU ****/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 1
#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // initialize serial communication
  Serial.begin(38400);
  while (!Serial);

  /**** Setup for MPU6050 ****/
  // join I2C bus
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock.

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // gyro offsets, scaled for min sensitivity
  mpu.setZGyroOffset(-4);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

  } else {
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
    Serial.println("device failed");
    while (true);
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // Set Kalman Filter Values
  set_values(68, 2, 0.002, 5);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  /**** Code for MPU6050 ****/
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    // display YAW angle in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }

  /**** Code for Kalman Filter ****/
  measured_value = (double)(ypr[0] * 180 / M_PI);     // take measured yaw value
  Measurement_Update(measured_value);                 // Apply Kalman Filter on the measured value(YAW)

  // Chose how to display results
  //Monitor_Results();
  //Plotter_Results();
}

// ================================================================
// ===                Kalman Filter Functions                   ===
// ================================================================

// Set X, P, Q, R
void set_values(double _curr_estimate, double _curr_uncertanity,
                double _process_var, double _sensor_varaiance) {
  curr_estimate = _curr_estimate;
  curr_uncertanity = _curr_uncertanity;
  process_var = _process_var;
  sensor_varaiance = _sensor_varaiance;
}

// State prediction equations
// Update prdections values after each measurementa update
void State_Prediction() {
  prev_estimate = curr_estimate;
  prev_uncertanity = curr_uncertanity + process_var;
}

// Measurment update equations
void Measurement_Update(double measured_value) {
  State_Prediction();

  // Calc kalman gain ---> Prediction Uncertainty / (Prediction Uncertainty + Sensor Varaiance)
  kalman_gain = prev_uncertanity / (prev_uncertanity + sensor_varaiance);
  // calc Current Estimate ---> Predicted state + Kalman Gain *Residual
  curr_estimate = prev_estimate + kalman_gain * (measured_value - prev_estimate);
  // Calc Uncertainty ---> (1 - Kalman Gain)*Prediction Uncertainty
  curr_uncertanity = (1 - kalman_gain) * prev_uncertanity;
}

// Display calculated values on Serial Monitor
void Monitor_Results() {
  Serial.print("Kalman Gain(K): ");
  Serial.println(kalman_gain);
  Serial.print("Current Estimate(X): ");
  Serial.println(curr_estimate);
  Serial.print("Uncertainty(P): ");
  Serial.println(curr_uncertanity);
  Serial.println("=====================================");
}

// Display calculated values on Serial Plotter
void Plotter_Results() {
  Serial.print(curr_estimate);
  Serial.print(',');
  Serial.println(measured_value);
  Serial.println();
}
