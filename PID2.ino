#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
MPU6050 mpu;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[] = {0, 0, 0};         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float lastypr[] = {1, 1, 1};

//PID//
double Output[] = {0, 0, 0, 0}, Setpoint[] = {0, 0, 0};
double lastErr[] = {0, 0, 0};
double kp[] = {1, 1, 1}, ki[] = {0, 0, 0}, kd[] = {0, 0, 0};
double now = 0 ;
double startTime = 0;
double lastTime[] = {0, 0, 0};
///////

Servo north;
Servo east;
Servo south;
Servo west;

char view = 0;
double lastMessage = 0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {

  north.attach(3);
  east.attach(4);
  south.attach(5);
  west.attach(6);

  north.write(90);
  east.write(90);
  south.write(90);
  west.write(90);

  pinMode(A1, OUTPUT);
  //pinMode(A2,INPUT);
  //pinMode(A3,INPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

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

  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;
  startTime = millis() / 1000;
  while (ypr[0] != lastypr[0] || ypr[1] != lastypr[1] || ypr[2] != lastypr[2]) {
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      lastypr[0] = ypr[0];
      lastypr[1] = ypr[1];
      lastypr[2] = ypr[2];
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
      Serial.print("lastypr\t");
      Serial.print(lastypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(lastypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(lastypr[2] * 180 / M_PI);
      Serial.print("\t");
      Serial.println((double)millis() / 1000 - startTime);

    }
  }
  Serial.print("Calibrated in: ");
  Serial.print((double)millis() / 1000 - startTime);
  Serial.println(" Seconds!");
  Serial.println("Ready to Fly!");
  delay(2000);
  startTime = millis() / 1000;
}
void loop() {
  imu();
  controls();
  yaw();
  pitch();
  roll();
  analogWrite(1, Output[3]);
  north.write(90 + Output[2] + Output[0]);
  east.write(90 + Output[1] + Output[0]);
  south.write(90 - Output[1] + Output[0]);
  west.write(90 - Output[2] + Output[0]);

  if (Serial.available()) {
    view = Serial.read();
    Serial.print("\n\n\n\n\nInputs:\n1 Displays ypr\n2 Displays Pitch\n3 Displays Yaw\n4 Displays Roll\n5 Displays Servo Positions\n6 Displays Joy Stick\nAll Others Display Nothing\n\n\n\n");
  }
  if (view == '5') {
    Serial.print("NESW:\t");
    Serial.print(north.read());
    Serial.print("\t");
    Serial.print(east.read());
    Serial.print("\t");
    Serial.print(south.read());
    Serial.print("\t");
    Serial.println(west.read());
  }
  if (view == 'x' || (ypr[1] * 180 / M_PI) > 45 || (ypr[1] * 180 / M_PI) < -45 || (ypr[2] * 180 / M_PI) > 45 || (ypr[2] * 180 / M_PI) < -45) {
    //terminate();
  }
}

void imu() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

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

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    if (view == '1') {
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
    }
  }
}

void yaw() {
  now = (double)millis() / 1000 - startTime;
  double deltaTime = (double)(now - lastTime[0]); //time since last calculation

  double error = Setpoint[0] - ypr[0] * 180 / M_PI;
  double errSum += (error * deltaTime);
  double dErr = (error - lastErr[0]) / deltaTime;
  Output[0] = kp[0] * error + ki[0] * errSum + kd[0] * dErr;
  if (Output[0] > 90) {
    Output[0] = 90;
  }
  else if (Output[0] < -90) {
    Output[0] = -90;
  }
  lastErr[0] = error;
  lastTime[0] = now;
  if (view == '2') {
    Serial.print("Yaw-Error,PID,Time: ");
    Serial.print("\t");
    Serial.print(error);
    Serial.print("\t");
    Serial.print(kp[0] * error);
    Serial.print("\t");
    Serial.print(ki[0] * errSum);
    Serial.print("\t");
    Serial.println(kd[0] * dErr);
  }
}

void pitch() {
  now = (double)millis() / 1000 - startTime;
  double deltaTime = (double)(now - lastTime[1]); //time since last calculation

  double error = Setpoint[1] - ypr[1] * 180 / M_PI;
  double errSum += (error * deltaTime);
  double dErr = (error - lastErr[1]) / deltaTime;
  Output[1] = kp[1] * error + ki[1] * errSum + kd[1] * dErr;
  if (Output[1] > 90) {
    Output[1] = 90;
  }
  else if (Output[1] < -90) {
    Output[1] = -90;
  }
  lastErr[1] = error;
  lastTime[1] = now;
  if (view == '3') {
    Serial.print("Pitch-Error,PID,Time: ");
    Serial.print("\t");
    Serial.print(error);
    Serial.print("\t");
    Serial.print(kp[1] * error);
    Serial.print("\t");
    Serial.print(ki[1] * errSum);
    Serial.print("\t");
    Serial.println(kd[1] * dErr);
  }
}

void roll() {
  now = (double)millis() / 1000 - startTime;
  double deltaTime = (double)(now - lastTime[2]); //time since last calculation

  double error = Setpoint[2] - ypr[2] * 180 / M_PI;
  double errSum += (error * deltaTime);
  double dErr = (error - lastErr[2]) / deltaTime;
  Output[2] = kp[2] * error + ki[2] * errSum + kd[2] * dErr;
  if (Output[2] > 90) {
    Output[2] = 90;
  }
  else if (Output[2] < -90) {
    Output[2] = -90;
  }
  lastErr[2] = error;
  lastTime[2] = now;
  if (view == '4') {
    Serial.print("Roll-Error,PID,Time: ");
    Serial.print("\t");
    Serial.print(error);
    Serial.print("\t");
    Serial.print(kp[2] * error);
    Serial.print("\t");
    Serial.print(ki[2] * errSum);
    Serial.print("\t");
    Serial.println(kd[2] * dErr);
  }
}

void controls() {
  Setpoint[0] += (analogRead(4) / 511.5 - 1);
  Setpoint[1] = 45 * (analogRead(2) / 511.5 - 1);
  Setpoint[2] = 45 * (analogRead(3) / 511.5 - 1);

  if (view == '6') {
    Serial.print("XYZ:\t");
    Serial.print(analogRead(2) / 511.5 - 1);
    Serial.print("\t");
    Serial.print(analogRead(3) / 511.5 - 1);
    Serial.print("\t");
    Serial.println(analogRead(4) / 511.5 - 1);
  }
}

void terminate() {
  Serial.print("\n\n\n\n\n\n\n\n\n\nABORTED\n\n\n\n\n\n\n\n\n\n");
  north.write(90);
  east.write(90);
  south.write(90);
  west.write(90);
  analogWrite(1, 0);
  while (true);
}
