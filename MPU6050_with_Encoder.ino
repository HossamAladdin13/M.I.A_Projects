// M.I.A ROBOTICS TEAM
// IMU & Encoder task 
// Device measures the YAW angle from MPU6050..
// ..measures the SPEED of motor via Rotary Encoder
// Using STM32f103c8t6

    /**** Preparing the MPU6050 ****/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN PA0
#define LED_PIN PC13
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


    /**** Preparing the Motor & Encoder ****/

#define ENC_COUNT_REV 374       // Motor encoder output pulse per rotation (change as required)
#define ENC_IN PA1              // Encoder output to Interrupt pin 
#define PWM PA2                 // PWM connected to PWM pin

int speedcontrol = 0;           // Analog input
int interval = 1000;            // One-second interval for measurements
 
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
int rpm = 0;                    // Variable for RPM measuerment
 

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

volatile long encoderValue = 0;         // Pulse count from encoder
void updateEncoder()
{
  encoderValue++;       // Increment value for each pulse from encoder
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
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // gyro offsets, scaled for min sensitivity
    mpu.setZGyroOffset(-85);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050       
        mpu.CalibrateGyro(6);
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable STM32 interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

             /**** Setup for Motor & Encoder ****/   
    
    pinMode(ENC_IN, INPUT_PULLUP);          // Set encoder as input with internal pullup  
    pinMode(PWM, OUTPUT);                   // Set PWM and DIR connections as outputs  
    attachInterrupt(digitalPinToInterrupt(ENC_IN), updateEncoder, RISING);      // Attach interrupt  
    previousMillis = millis();             // Setup initial values for timer

    // to show result
    Serial.println("-YAW- \t\t\t\t -SPEED-");
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
             /**** Code for MPU6050 ****/     
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            // display YAW angle in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.println(ypr[0] * 180/M_PI);
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    
             /**** Code for Motor & Encoder ****/   
    // Write PWM to controller
    analogWrite(PWM, map(analogRead(speedcontrol), 0, 1023, 0, 255));
  
    // Update RPM value every second
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;
        
        // Calculate RPM
        rpm = (float)(encoderValue * 60 / ENC_COUNT_REV);
     
        // Only update display when there is a reading
        if (rpm > 0) {
            Serial.print("\t\t\t\t ");
            Serial.print(rpm);
            Serial.println(" RPM");
        }
        else Serial.println("");
        
        encoderValue = 0;
    }
}
