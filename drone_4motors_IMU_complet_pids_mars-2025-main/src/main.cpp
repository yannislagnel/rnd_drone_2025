#include <Arduino.h>
#include <Servo.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define GYRO_INT 34

//=============================================================================
// ===                      define motors & servos                         ====
//=============================================================================
// ESR FL
#define PIN_FL 37
// ESR RL
#define PIN_RL 14
// ESR RR
#define PIN_RR 24
// ESR FR
#define PIN_FR 25

Servo ESC_FL;
Servo ESC_RL;
Servo ESC_RR;
Servo ESC_FR;

int16_t PWM_FL = 1000;
int16_t PWM_RL = 1000;
int16_t PWM_RR = 1000;
int16_t PWM_FR = 1000;

//*************************** IMU MPU6050 *******************************
// with the DMP using I2Cdev lib
MPU6050 mpu;                    // MPU6050 mpu(0x69); // <-- use for AD0 high
#define INTERRUPT_PIN GYRO_INT  // use pin 34 for teensy could be any digital
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success,
                      // !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer
// orientation/motion vars
Quaternion q;    // [w, x, y, z]         quaternion container
VectorInt16 aa;  // [x, y, z]            accel sensor measurements
VectorInt16
    aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
    aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float
    ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===                PIDs for pitch & roll                    ====
// ================================================================
double throttle = 1300;  // initial value of throttle to the motors
float pitch = 0;
float roll = 0;
float yaw = 0;

// yaw manuellement
float p_desired_angle = 0.0;  // This is the angle in which we whant pitch
float r_desired_angle = 0.0;  // for roll
float y_desired_angle = 0.0;  // for yaw
// ************************** Pitch PID ***************************
double p_kp = 7.50000;  // 3.55
double p_ki = 0.004400;  // 0.003
double p_kd = 2.5000;    // 2.05
float P_PID, p_error, p_previous_error;
float p_pid_p = 0.0;
float p_pid_i = 0.0;
float p_pid_d = 0;

// ************************** Roll PID ***************************
double r_kp = 5.5000;    // 3.55
double r_ki = 0.004800;  // 0.003
double r_kd = 1.330;     // 2.05

float R_PID, r_error, r_previous_error;
float r_pid_p = 0.0;
float r_pid_i = 0.0;
float r_pid_d = 0;

// ************************** yaw PID ***************************
double y_kp = 10.0000;   // 3.55
double y_ki = 0.007500;  // 0.003
double y_kd = 2.000;     // 2.05

float Y_PID, y_error, y_previous_error;
float y_pid_p = 0.0;
float y_pid_i = 0.0;
float y_pid_d = 0;
// ==================================================================

char c = 0;
float elapsedTime, Time_act, timePrev;
bool urgence_stop = false;
// ================================================================
// ===               fonction declaration                      ====
// ================================================================
// --- pids --------
float pid_pitch(void);
float pid_roll(void);
float pid_yaw(void);

// --- ESC --------
void init_esc_cal(void);
void init_esc(void);
// --- INTERRUPT MPU6050 DETECTION ROUTINE ----
volatile bool mpuInterrupt =
    false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }
void setup_mpu6050(void);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Serial.begin(115200);
  // --------- init the ESCs, servos -------------------------------
  Serial.println("init ESC and servos...");
  init_esc();  // sans calibration en premier et on met le Throtle a 1000 (zero)
               // 4 moteurs stop
  PWM_FL = PWM_RL = PWM_RR = PWM_FR = 1000;
  ESC_FL.writeMicroseconds(PWM_FL);
  ESC_RL.writeMicroseconds(PWM_RL);
  ESC_RR.writeMicroseconds(PWM_RR);
  ESC_FR.writeMicroseconds(PWM_FR);

  delay(500);

  // ********* setup MPU6050 ****************************************
  Serial.println("init MPU6050 ...");
  ypr[0] = 0.0;
  ypr[1] = 0.0;
  ypr[2] = 0.0;
  setup_mpu6050();  // init le MPU6050

  //********** Attente d'une touche pour continuer ********
  // on attend qu'un carractere soit rentré sur le port serie pour continuer
  Serial.println("Tapez une touche pour continuer");
  while (!Serial.available()) {
    delay(1);
  }
  c = Serial.read();
  c = 0;
  throttle = 1000;

  Serial.println("\nSetup ended .....");
  Time_act = millis();  // Start counting time in milliseconds
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // ------- on peut augmenter ou diminuer Throttle par le port serie en
  // input T pour augmenter et t pour diminuer on a un char en input
  // serie?
  c = 0;
  if (Serial.available()) {
    delay(2);  // delay to allow byte to arrive in input buffer
    c = Serial.read();

    // urgence stop 
    if (c == 'S') {  // urgence stop if S sent
      urgence_stop = true;
    }
    if (c == 's') {  // urgence set to false 's'
      urgence_stop = false;
    }

    //  for constants pitch
    if (c == 'P') {
      p_kp = p_kp + 0.1;  // par pas de 0.01 (empirique)
    }
    if (c == 'p') {
      p_kp = p_kp - 0.1;
    }

    if (c == 'I') {
      p_ki = p_ki + 0.0001;  // par pas de 0.002
      p_pid_i = 0;
    }
    if (c == 'i') {
      p_ki = p_ki - 0.0001;
      p_pid_i = 0;
    }

    if (c == 'D') {
      p_kd = p_kd + 0.1;  // par pas de 0.001
    }
    if (c == 'd') {
      p_kd = p_kd - 0.1;
    }

    //  for constants roll Q/q pour kp, W/w pour ki & E/e pour kd
    if (c == 'Q') {
      r_kp = r_kp + 0.1;  // par pas de 0.01 (empirique)
    }
    if (c == 'q') {
      r_kp = r_kp - 0.1;
    }

    if (c == 'W') {
      r_ki = r_ki + 0.0001;  // par pas de 0.002
      r_pid_i = 0;
    }
    if (c == 'w') {
      r_ki = r_ki - 0.0001;
      r_pid_i = 0;
    }

    if (c == 'E') {
      r_kd = r_kd + 0.1;  // par pas de 0.001
    }
    if (c == 'e') {
      r_kd = r_kd - 0.1;
    }

    // for constants yaw Y/y pour kp, Z/z pour ki, X/x pour kd

    if (c == 'Y') {
      y_kp = y_kp + 0.1;  // par pas de 0.01 (empirique)
    }
    if (c == 'y') {
      y_kp = y_kp - 0.1;
    }

    if (c == 'Z') {
      y_ki = y_ki + 0.0001;  // par pas de 0.002
      y_pid_i = 0;
    }
    if (c == 'z') {
      y_ki = y_ki - 0.0001;
      y_pid_i = 0;
    }

    if (c == 'X') {
      y_kd = y_kd + 0.1;  // par pas de 0.001
    }
    if (c == 'x') {
      y_kd = y_kd - 0.1;
    }

    if (c == 'T') {
      throttle = throttle + 10;  // par pas de 10
      if (throttle > 2000) {
        throttle = 2000;
      }
    }
    if (c == 't') {
      throttle = throttle - 10;
      if (throttle < 1000) {
        throttle = 1000;
      }
    }
    while (Serial.available() >
           0) {  // on fini de lire le reste et vider le buffer
      c = Serial.read();
    }
  }

  timePrev =
      Time_act;  // the previous time is stored before the actual time read
  Time_act = millis();  // actual time read
  elapsedTime = (Time_act - timePrev) / 1000;

  //************  read MPU6050  *********************
  while (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    delayMicroseconds(100);
  }
  // if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
  //  display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // ATTENTION - ou + 180 doivent être ajustées en fonction du sens
  ypr[0] = ypr[0] * +180.0 / M_PI;  // pour convertir en angles
  ypr[1] = ypr[1] * 180.0 / M_PI;   // negatif =>down
  ypr[2] = ypr[2] * -180.0 / M_PI;  // pour avoir un roll a droite => negatif
  //}

  // **************** set motors speed *****************
  // only with proportionnal and 0 roll and 0 pitch
  // to test we say all 0
  pitch = pid_pitch();
  roll = pid_roll();
  yaw = pid_yaw();  // or: ypr[0];
  yaw = 0.00;
  roll = 0.0;
  // see the doc mpu_capteurs.docx page 6
  PWM_FR = int16_t(throttle - roll - pitch - yaw);  // motor 1
  PWM_RR = int16_t(throttle - roll + pitch + yaw);  // motor 2
  PWM_RL = int16_t(throttle + roll + pitch - yaw);  // motor 3
  PWM_FL = int16_t(throttle + roll - pitch + yaw);  // motor 4

  if (PWM_FR < 1000) {
    PWM_FR = 1000;
  }
  if (PWM_FR > 2000) {
    PWM_FR = 2000;
  }

  if (PWM_RR < 1000) {
    PWM_RR = 1000;
  }
  if (PWM_RR > 2000) {
    PWM_RR = 2000;
  }
  if (PWM_RL < 1000) {
    PWM_RL = 1000;
  }
  if (PWM_RL > 2000) {
    PWM_RL = 2000;
  }

  if (PWM_FL < 1000) {
    PWM_FL = 1000;
  }
  if (PWM_FL > 2000) {
    PWM_FL = 2000;
  }

  if (urgence_stop == true) {
    PWM_FL = PWM_RL = PWM_RR = PWM_FR = 1000;
  }

  ESC_FL.writeMicroseconds(PWM_FL);
  ESC_RL.writeMicroseconds(PWM_RL);
  ESC_RR.writeMicroseconds(PWM_RR);
  ESC_FR.writeMicroseconds(PWM_FR);

  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  // display real acceleration, adjusted to remove gravity
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

  Serial.print(">");
  // for pitch
  Serial.print(ypr[1]);  // 1
  Serial.print(",");
  Serial.print(p_desired_angle);  // 2
  Serial.print(",");
  Serial.print(p_error);  // 3
  Serial.print(",");
  Serial.print(P_PID);  // 4
  Serial.print(",");
  // for roll
  Serial.print(ypr[2]);  // 5
  Serial.print(",");
  Serial.print(r_desired_angle);  // 6
  Serial.print(",");
  Serial.print(r_error);  // 7
  Serial.print(",");
  Serial.print(R_PID);  // 8
  Serial.print(",");
  // motors PWM
  Serial.print(PWM_FL);  // 9
  Serial.print(",");
  Serial.print(PWM_RL);  // 10
  Serial.print(",");
  Serial.print(PWM_RR);  // 11
  Serial.print(",");
  Serial.print(PWM_FR);  // 12
  Serial.print(",");
  // time
  Serial.print(elapsedTime * 1000.0, 4);  // 13
  Serial.print(",");
  // constants pitch
  Serial.print(p_kp, 5);  // 14
  Serial.print(",");
  Serial.print(p_ki, 5);  // 15
  Serial.print(",");
  Serial.print(p_kd, 5);  // 16
  Serial.print(",");
  // constants roll
  Serial.print(r_kp, 5);  // 17
  Serial.print(",");
  Serial.print(r_ki, 5);  // 18
  Serial.print(",");
  Serial.print(r_kd, 5);  // 19
  Serial.print(",");
  // constants yaw
  Serial.print(y_kp, 5);  // 20
  Serial.print(",");
  Serial.print(y_ki, 5);  // 21
  Serial.print(",");
  Serial.print(y_kd, 5);  // 22
  Serial.print(",");
  Serial.print(throttle);  // 23
  Serial.print(",");
  // for yaw
  Serial.print(ypr[0]);  // 24
  Serial.print(",");
  Serial.print(y_desired_angle);  // 25
  Serial.print(",");
  Serial.print(y_error);  // 26
  Serial.print(",");
  Serial.println(Y_PID);  // 27

  p_previous_error = p_error;  // Remember to store the previous error.
  r_previous_error = r_error;  // Remember to store the previous error.
  y_previous_error = y_error;  // Remember to store the previous error.
  delay(10);
}

float pid_roll(void) {
  /*First calculate the error between the desired angle and
   *the real measured angle*/
  r_error = ypr[2] - r_desired_angle;

  /*Next the proportional value of the PID is just a proportional constant
   *multiplied by the error*/
  r_pid_p = r_kp * r_error;

  /*The integral part should only act if we are close to the
  desired position but we want to fine tune the error. That's
  why I've made a if operation for an error between -2 and 2 degree.
  To integrate we just sum the previous integral value with the
  error multiplied by  the integral constant. This will integrate (increase)
  the value each loop till we reach the 0 point*/
  // if (abs(error) < 10) {
  //     pid_i = pid_i + (ki * error);
  // }
  r_pid_i = r_pid_i + (r_ki * r_error);

  /*The last part is the derivate. The derivate acts upon the speed of the
  error. As we know the speed is the amount of error that produced in a certain
  amount of time divided by that time. For taht we will use a variable called
  previous_error. We substract that value from the actual error and divide all
  by the elapsed time. Finnaly we multiply the result by the derivate constant*/

  r_pid_d = r_kd * ((r_error - r_previous_error) / elapsedTime);

  /*The final PID values is the sum of each of this 3 parts*/
  R_PID = r_pid_p + r_pid_i + r_pid_d;

  /*We know taht the min value of PWM signal is 1000us and the max is 2000. So
  that tells us that the PID value can/s oscilate more than -1000 and 1000
  because when we have a value of 2000us the maximum value taht we could
  sybstract is 1000 and when we have a value of 1000us for the PWM sihnal, the
  maximum value that we could add is 1000 to reach the maximum 2000us*/
  if (R_PID < -1000) {
    R_PID = -1000;
  }
  if (R_PID > 1000) {
    R_PID = 1000;
  }
  return R_PID;
}

float pid_pitch(void) {
  /*First calculate the error between the desired angle and
   *the real measured angle*/
  p_error = ypr[1] - p_desired_angle;

  /*Next the proportional value of the PID is just a proportional constant
   *multiplied by the error*/
  p_pid_p = p_kp * p_error;

  /*The integral part should only act if we are close to the
  desired position but we want to fine tune the error. That's
  why I've made a if operation for an error between -2 and 2 degree.
  To integrate we just sum the previous integral value with the
  error multiplied by  the integral constant. This will integrate (increase)
  the value each loop till we reach the 0 point*/
  // if (abs(error) < 10) {
  //     pid_i = pid_i + (ki * error);
  // }
  p_pid_i = p_pid_i + (p_ki * p_error);

  /*The last part is the derivate. The derivate acts upon the speed of the
  error. As we know the speed is the amount of error that produced in a certain
  amount of time divided by that time. For taht we will use a variable called
  previous_error. We substract that value from the actual error and divide all
  by the elapsed time. Finnaly we multiply the result by the derivate constant*/

  p_pid_d = p_kd * ((p_error - p_previous_error) / elapsedTime);

  /*The final PID values is the sum of each of this 3 parts*/
  P_PID = p_pid_p + p_pid_i + p_pid_d;

  /*We know taht the min value of PWM signal is 1000us and the max is 2000. So
  that tells us that the PID value can/s oscilate more than -1000 and 1000
  because when we have a value of 2000us the maximum value taht we could
  sybstract is 1000 and when we have a value of 1000us for the PWM sihnal, the
  maximum value that we could add is 1000 to reach the maximum 2000us*/
  if (P_PID < -1000) {
    P_PID = -1000;
  }
  if (P_PID > 1000) {
    P_PID = 1000;
  }
  return P_PID;
}

float pid_yaw(void) {
  /*First calculate the error between the desired angle and
   *the real measured angle*/
  y_error = ypr[0] - y_desired_angle;

  /*Next the proportional value of the PID is just a proportional constant
   *multiplied by the error*/
  y_pid_p = y_kp * y_error;

  /*The integral part should only act if we are close to the
  desired position but we want to fine tune the error. That's
  why I've made a if operation for an error between -2 and 2 degree.
  To integrate we just sum the previous integral value with the
  error multiplied by  the integral constant. This will integrate (increase)
  the value each loop till we reach the 0 point*/
  // if (abs(error) < 10) {
  //     pid_i = pid_i + (ki * error);
  // }
  y_pid_i = y_pid_i + (y_ki * y_error);

  /*The last part is the derivate. The derivate acts upon the speed of the
  error. As we know the speed is the amount of error that produced in a certain
  amount of time divided by that time. For taht we will use a variable called
  previous_error. We substract that value from the actual error and divide all
  by the elapsed time. Finnaly we multiply the result by the derivate constant*/

  y_pid_d = y_kd * ((y_error - y_previous_error) / elapsedTime);

  /*The final PID values is the sum of each of this 3 parts*/
  Y_PID = y_pid_p + y_pid_i + y_pid_d;

  /*We know taht the min value of PWM signal is 1000us and the max is 2000. So
  that tells us that the PID value can/s oscilate more than -1000 and 1000
  because when we have a value of 2000us the maximum value taht we could
  sybstract is 1000 and when we have a value of 1000us for the PWM sihnal, the
  maximum value that we could add is 1000 to reach the maximum 2000us*/
  if (Y_PID < -1000) {
    Y_PID = -1000;
  }
  if (Y_PID > 1000) {
    Y_PID = 1000;
  }
  return Y_PID;
}

void init_esc_cal(void) {
  /* while (1)
  {
    throtle = analogRead(A3);
    Serial.println(throtle);
    delay(250);
  } */
  // Servo for the ESC
  ESC_FL.attach(
      PIN_FL, 1000,
      2000);  // FL range 1000 usec =full stop to 2000 usec =>full speed

  ESC_RL.attach(
      PIN_RL, 1000,
      2000);  // RL range 1000 usec =full stop to 2000 usec =>full speed

  ESC_RR.attach(
      PIN_RR, 1000,
      2000);  // RR range 1000 usec =full stop to 2000 usec =>full speed

  ESC_FR.attach(
      PIN_FR, 1000,
      2000);  // FR range 1000 usec =full stop to 2000 usec =>full speed

  PWM_FL = PWM_RL = PWM_RR = PWM_FR = 2000;
  ESC_FL.writeMicroseconds(PWM_FL);
  ESC_RL.writeMicroseconds(PWM_RL);
  ESC_RR.writeMicroseconds(PWM_RR);
  ESC_FR.writeMicroseconds(PWM_FR);

  delay(100);
  Serial.println(
      " mettre la battery et attendre les bips et ensuite appuez sur une "
      "touche");
  // int ch = Serial.read();  // get the first char.
  /*  while (Serial.available()) {
     Serial.read();
   }
   while (!Serial.available()) {
   } */
  // while (Serial.read() != -1);
  delay(4000);

  Serial.println(" mise a 0 et attente de 5 sec pour un autre bip");
  PWM_FL = PWM_RL = PWM_RR = PWM_FR = 1000;
  ESC_FL.writeMicroseconds(PWM_FL);
  ESC_RL.writeMicroseconds(PWM_RL);
  ESC_RR.writeMicroseconds(PWM_RR);
  ESC_FR.writeMicroseconds(PWM_FR);
  delay(5000);

  Serial.println("Setup et calibration finis ");
}

void init_esc(void) {
  // Servo for the ESC
  ESC_FL.attach(
      PIN_FL, 1000,
      2000);  // FL range 1000 usec =full stop to 2000 usec =>full speed
  ESC_RL.attach(
      PIN_RL, 1000,
      2000);  // RL range 1000 usec =full stop to 2000 usec =>full speed
  ESC_RR.attach(
      PIN_RR, 1000,
      2000);  // RR range 1000 usec =full stop to 2000 usec =>full speed
  ESC_FR.attach(
      PIN_FR, 1000,
      2000);  // FR range 1000 usec =full stop to 2000 usec =>full speed

  PWM_FL = PWM_RL = PWM_RR = PWM_FR = 1000;
  ESC_FL.writeMicroseconds(PWM_FL);
  ESC_RL.writeMicroseconds(PWM_RL);
  ESC_RR.writeMicroseconds(PWM_RR);
  ESC_FR.writeMicroseconds(PWM_FR);

  delay(5000);
}

void setup_mpu6050(void) {
  // ************************ Setup  MPU6050
  // **************************************************
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock.
  // ----- initialize device -------------------------
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                      : F("MPU6050 connection failed"));
  // ------ load and configure the DMP --------------
  delay(100);
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.print(
        F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use
    // it
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
