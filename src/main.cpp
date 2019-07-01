/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/* This driver reads quaternion data from the MPU6060 and sends
   Open Sound Control messages.

  GY-521  NodeMCU
  MPU6050 devkit 1.0
  board   Lolin         Description
  ======= ==========    ====================================================
  VCC     VU (5V USB)   Not available on all boards so use 3.3V if needed.
  GND     G             Ground
  SCL     D1 (GPIO05)   I2C clock
  SDA     D2 (GPIO04)   I2C data
  XDA     not connected
  XCL     not connected
  AD0     not connected
  INT     D8 (GPIO15)   Interrupt pin

*/

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <DNSServer.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Skip connecting to WiFi for faster debugging
const bool DEBUG_OFFLINE = false;

// Tonic note for cadence
int keynote = 72; // c2

// Threshold value for distinction between NoteOn/NoteOff
// Active angle = angle +- angleThreshold
const int angleThreshold = 45;

// Global loop variables
bool firstLoop = true;
char previousTriad[3];

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 5/3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the ESP8266 GPIO15
   pin.
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT_OSC" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT_OSC

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
#ifdef OUTPUT_READABLE_EULER
float euler[3];         // [psi, theta, phi]    Euler angle container
#endif
#ifdef OUTPUT_READABLE_YAWPITCHROLL
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#endif

#define INTERRUPT_PIN 15 // use pin 15 on ESP8266

const char DEVICE_NAME[] = "mpu6050";

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(192, 168, 1, 101);     // remote IP to receive OSC
const unsigned int outPort = 9999;          // remote port to receive OSC

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void mpu_setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
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

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(42);
  mpu.setYGyroOffset(-60);
  mpu.setZGyroOffset(51);
  mpu.setZAccelOffset(801); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("\nOrientation Sensor OSC output")); Serial.println();

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  if (!DEBUG_OFFLINE) {
    WiFiManager wifiManager;
  //reset saved settings
  //wifiManager.resetSettings();

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
    wifiManager.autoConnect(DEVICE_NAME);

    Serial.print(F("WiFi connected! IP address: "));
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("Offline debug mode. Not going to try to establish WiFi connection.");
  }

  mpu_setup();
}

/*
  Acquires MPU sensor data.
  Returns, whether new sensor data could be obtained (true)
  or not (false).
*/
bool mpu_loop()
{
  bool newDataObtained;
  // if programming failed, don't try to do anything
  if (!dmpReady) return false;

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize) return false;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    
    return false;

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_TEAPOT_OSC
#ifndef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
#endif
    // Send OSC message
    OSCMessage msg("/imuquat");
    msg.add((float)q.w);
    msg.add((float)q.x);
    msg.add((float)q.y);
    msg.add((float)q.z);

    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();

    msg.empty();
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print("ypr\t");
    //Serial.print(ypr[0] * 180/M_PI);
    //Serial.print("\t");
    //Serial.print(ypr[1] * 180/M_PI);
    //Serial.print("\t");
    //Serial.println(ypr[2] * 180/M_PI);
    newDataObtained = true;
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

    return newDataObtained;
  }
  return false;
}

/* 
  Convert YPR data into angles
  between -180° and +180°.

  For passing arrays, see: https://stackoverflow.com/questions/11656532/returning-an-array-using-c
*/
float *calcAngles(float *yprData) {
  static float yprAngles[3]; // do not destroy array after return statement
  for (int i = 0; i <= 3; i++)
    yprAngles[i] = yprData[i] * 180/M_PI;

  return yprAngles; // return pointer to angle data
}

/*
  Detect triad of the cadence from yaw/pitch/roll angle.
*/
char *detectTriad(float *angles) {
  static char triad[3];
  // 1. T: Tonic (Tonika, Dur)
  // YPR x == 90, y == 90
  if ((angles[0] >= 90.0-angleThreshold && angles[0] < 90.0+angleThreshold) && (angles[1] >= 90.0-angleThreshold && angles[1] < 90.0+angleThreshold)) { // && (angles[2] >= 0.0-angleThreshold && angles[2] < 0.0+angleThreshold)) {
    strcpy(triad, "T");
    return triad;
  }

  // 2. Sp: Supertonic (Subdominantparallele, Moll)
  // x == +-180, y == 0
  else if (((angles[0] >= 180.0-angleThreshold && angles[0] <= 180.0) || (angles[0] < -180.0+angleThreshold && angles[0] >= -180.0)) && (angles[1] >= 0.0-angleThreshold && angles[1] < 0.0+angleThreshold)) {
    strcpy(triad, "Sp");
    return triad;
  }

  // 3. Dp: Mediant (Dominantparallele, Moll)
  // x == 0, y == 0
  else if ((angles[0] >= 0.0-angleThreshold && angles[0] < 0.0+angleThreshold) && (angles[1] >= 0.0-angleThreshold && angles[1] < 0.0+angleThreshold)) {
    strcpy(triad, "Dp");
    return triad;
  }

  // 4. S: Subdominant (Subdominante, Dur)
  // x == +-180, y == +-180
   else if (((angles[0] >= 180.0-angleThreshold && angles[0] <= 180.0) || (angles[0] < -180.0+angleThreshold && angles[0] >= -180.0)) && ((angles[1] >= 180.0-angleThreshold && angles[1] <= 180.0) || (angles[1] < -180.0+angleThreshold && angles[1] >= -180.0))) {
    strcpy(triad, "S");
    return triad;
  }

  // 5. D: Dominant (Dominante, Dur)
  // x == 0, y == +-180
  else if ((angles[0] >= 0.0-angleThreshold && angles[0] < 0.0+angleThreshold) && ((angles[1] >= 180.0-angleThreshold && angles[1] <= 180.0) || (angles[1] < -180.0+angleThreshold && angles[1] >= -180.0))) {
    strcpy(triad, "D");
    return triad;
  }

  // 6. Tp: Submediant (Tonikaparallele, Moll)
  // YPR x == 90, y == 0
  else if ((angles[0] >= 90.0-angleThreshold && angles[0] < 90.0+angleThreshold) && (angles[1] >= 0.0-angleThreshold && angles[1] < 0.0+angleThreshold)){ // && (angles[2] >= 0.0-angleThreshold && angles[2] < 0.0+angleThreshold)) {
    strcpy(triad, "Tp");
    return triad;
  }
  else
    strcpy(triad, "0");
    return triad;
}

void sendRenoiseNoteOn(int note, IPAddress outIp, int outPort) {
  OSCMessage msg("/renoise/trigger/note_on");
  msg.add((int)1).add((int)2).add(note).add((int)127);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void sendRenoiseNoteOff(int note, IPAddress outIp, int outPort) {  
  OSCMessage msg("/renoise/trigger/note_off");
  msg.add((int)1).add((int)2).add(note);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}  

/*
  Send triad as OSC messages to ReNoise.
*/
void sendTriad(char *triad, bool noteOn) {
  int first, third, fifth;
  // 1. T: Tonic (Tonika, Dur)
  if (strcmp(triad, "T") == 0) {
    first = keynote + 0;
    third = keynote + 4;
    fifth = keynote + 7;
  }

   // 2. Sp: Supertonic (Subdominantparallele, Moll)
  else if (strcmp(triad, "Sp") == 0) {
    first = keynote + 2;
    third = keynote + 5;
    fifth = keynote + 9;
  }

  // 3. Dp: Mediant (Dominantparallele, Moll)
  else if (strcmp(triad, "Dp") == 0) {
    first = keynote + 4;
    third = keynote + 7;
    fifth = keynote - 1;
  }

  // 4. S: Subdominant (Subdominante, Dur)
  else if (strcmp(triad, "S") == 0) {
    first = keynote + 5;
    third = keynote + 9;
    fifth = keynote + 0;
  }

  // 5. D: Dominant (Dominante, Dur)
  else if (strcmp(triad, "D") == 0) {
    first = keynote + 7;
    third = keynote - 1;
    fifth = keynote + 2;
  }

  // 6. Tp: Submediant (Tonikaparallele, Moll)
  else if (strcmp(triad, "Tp") == 0) {
    first = keynote + 9;
    third = keynote + 0;
    fifth = keynote + 4;
  }

  // Send OSC messages
  if (noteOn) {
    sendRenoiseNoteOn(first, outIp, outPort);
    sendRenoiseNoteOn(third, outIp, outPort);
    sendRenoiseNoteOn(fifth, outIp, outPort);
  }
  else {
    sendRenoiseNoteOff(first, outIp, outPort);
    sendRenoiseNoteOff(third, outIp, outPort);
    sendRenoiseNoteOff(fifth, outIp, outPort);
  }
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/

void loop(void) {

  // Connect to WiFi network
  if (WiFi.status() != WL_CONNECTED && !DEBUG_OFFLINE) {
    Serial.println();
    Serial.println("*** Disconnected from AP so rebooting ***");
    Serial.println();
    ESP.reset();
  }

  // Check for new MPU data
  while (mpu_loop() == false) {
    //Serial.println("Waiting for new MPU data...");
  }

  // Analyze MPU raw data
  float *mpuAngles = calcAngles(ypr);
  char *triad = detectTriad(mpuAngles);

  // Print raw data and cadence results
  Serial.print("\n\rypr\t");
  for (int i = 0; i <= 2; i++) {
    Serial.print(mpuAngles[i]);
    Serial.print("\t");
  }
  Serial.print(triad);

// Send only triad if it has changed
  if (strcmp(triad, previousTriad) != 0) { // check whether triad has changed
    if (!firstLoop) {
      sendTriad(previousTriad, false); // mute previous triad
      Serial.print("\t");
      Serial.print("mute");
    }
    else {
      Serial.print("\t");
    }
    if (strcmp(triad, "0") != 0) { // play new triad except for "0" triad (= not defined)
      sendTriad(triad, true);
      Serial.print("\t");
      Serial.print("play");
    }
  }

  // Save triad for next loop iteration
  strcpy(previousTriad, triad);

  //delay(100); // Too long delays cause "FIFO overflow" in mpu_loop() function
  firstLoop = false;
}
