// ESC motor for arduino:
// https://arduino.blaisepascal.fr/controler-un-moteur-brushless/

// hx711:
// BON ok:
// https://randomnerdtutorials.com/arduino-load-cell-hx711/
// https://www.instructables.com/How-to-Interface-HX711-Balance-Module-With-Load-Ce/
// autre:
// https://arduino-france.site/capteur-de-poids/

#include <Arduino.h>
#include <Servo.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/*********************************************************
Algorithme : Contrôle PID pour stabilisation avec MPU6050 et moteurs
Cet algorithme contrôle un système de stabilisation basé sur un MPU6050
avec utilistion seulement de accéléromètre et deux moteurs dubanc de test.
On utilise un régulateur PID pour maintenir un angle cible (desired_angle)
en ajustant les commandes des moteurs lié à l'écart entre l'angle mesuré et
l'angle désiré.

MPU6050 : Données d'accélération (sur les axes x, z).
Port série : Commandes utilisateur pour ajuster les constantes PID (kp, ki, kd).
Temps (millis) : Pour le calcul du temps écoulé entre les cycles.

Commandes moteurs (PWM) :
Valeurs ajustées entre 1000 (arrêt) et 2000 (vitesse maximale) pour les moteurs
gauche et droit. Port série pour la visualisation sur BetterSerialplot ou
SerialStudio : Angle mesuré (angle_degres) PID calculé (pid) Erreur entre
l'angle mesuré et l'angle cible (error) PWM appliqués aux moteurs (pwmL, pwmR)
Paramètres PID (kp, ki, kd)
et temps ecoulé pour un tour de boucle (elapsedTime)
**********************************************************/

// I2Cdev and MPU6050 must be installed as libraries, or else the.cpp /.h files
// for both classes must be in the include path of your project
// #include <Adafruit_MPU6050.h> //https://github.com/adafruit/Adafruit_MPU6050
// #include <Adafruit_Sensor.h> //https://github.com/adafruit/Adafruit_Sensor
// #include <Wire.h> //https://www.arduino.cc/en/reference/wire

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

// ESR connecte au ESC FL de la carte
// ESR FL
#define PIN_FL 37
// ESR FR
#define PIN_FR 25

// pin pour l'interrupt du MPU6050
#define GYRO_INT 34

Servo right_prop;
Servo left_prop;

// Pins pour les moteurs PWM
#define PIN_MOT_L PIN_FL
#define PIN_MOT_R PIN_FR

/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, Time_act, timePrev;
int i;
float rad_to_deg = 180 / 3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0.0;
float pid_i = 0.0;
float pid_d = 0;
/////////////////PID CONSTANTS/////////////////
// double kp = 3.5;
// double ki = 0.0005;
// double kd = 0.0501;

// double kp = 3.70;
// double ki = 0.00160;
// double kd = 0.50;

// double kp = 3.70;    // 3.55
// double ki = 0.0016;  // 0.003
// double kd = 0.60;    // 2.05

// double kp = 1.20;     // 3.55
// double ki = 0.00080;  // 0.003
// double kd = 0.210;    // 2.05
/////////////////////////////BEST_PID//////////////////////////////
//double kp = 1.40;     //
//double ki = 0.00080;  //
//double kd = 0.6200;   //
///////////////////////////////////////////////

/* double kp = 1.80;     // 3.55
double ki = 0.00080;  // 0.003
double kd = 0.620;    // 2.05 */


double kp = 1.430;     // 3.55
double ki = 0.000800;  // 0.003
double kd = 0.620;    // 2.05

double throttle = 1300;     // initial value of throttle to the motors
float desired_angle = 0.0;  // This is the angle in which we whant the
                            // balance to stay steady
char c = 0;

// --------- variables pour le filtre moyenne lissée
float angle_degres = 0.0;

float angle_degres_1 = 0;
float angle_degres_2 = 0;
float angle_degres_3 = 0;
float angle_degres_4 = 0;
float angle_degres_5 = 0;
float angle_degres_6 = 0;
float angle_degres_7 = 0;
float angle_degres_8 = 0;
float angle_degres_9 = 0;
float angle_degres_10 = 0;
float angle_degres_filtre = 0;

// ================================================================
// ===                      IMU MPU6050                        ====
// ================================================================
/// with the DMP using I2Cdev lib
// default I2C address is 0x68 AD0=0 by default
MPU6050 mpu(0x68);              // MPU6050 mpu(0x69); // <-- use for AD0 high
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

// --------------   fonction declaration
void setup_mpu6050(void);
// ===  INTERRUPT MPU6050 DETECTION ROUTINE
volatile bool mpuInterrupt =
    false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

// ================================================================
// ===                     SETUP                               ===
// ================================================================

void setup() {
  Serial.begin(115200);

  right_prop.attach(PIN_MOT_R, 1000, 2000);  // attatch the right motor to pin 3
  left_prop.attach(PIN_MOT_L, 1000, 2000);   // attatch the left motor to pin 5

  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/
  left_prop.writeMicroseconds(1000);
  right_prop.writeMicroseconds(1000);
  delay(1000);
  Serial.println("\n-----------------------------------");
  Serial.println("Starting Setup balancoire v1");
  Serial.println("-----------------------------------\n");
  // ********* setup MPU6050 ****************************************
  Serial.println("init MPU6050 ...");
  ypr[0] = 0.0;
  ypr[1] = 0.0;
  ypr[2] = 0.0;
  setup_mpu6050();  // init le MPU6050

  // on attend qu'un carractere soit rentré sur le port serie pour continuer
  Serial.println("Tapez une touche pour continuer");
  while (!Serial.available()) {
    delay(1);
  }
  c = Serial.read();
  c = 0;
  Time_act = millis();  // Start counting time in milliseconds

  // delay(7000); /*Give some delay, 7s, to have time to connect
  //  *the propellers and let everything start up*/

}  // end of setup void

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  /////////////////////////////I M U/////////////////////////////////////

  // ------- on peut augmenter ou diminuer kP, kI et kD par le port serie en
  // input P, I et D pour augmenter et p,i,d poir diminuer on a un char en input
  // serie?
  c = 0;
  if (Serial.available()) {
    delay(2);  // delay to allow byte to arrive in input buffer
    c = Serial.read();
    /* pid = 0;
    previous_pid = 0;
    previous_error = 0;
    previous_Time = 1;  */

    if (c == 'P') {
      kp = kp + 0.1;  // par pas de 0.01 (empirique)
    }
    if (c == 'p') {
      kp = kp - 0.1;
    }

    if (c == 'I') {
      ki = ki + 0.0001;  // par pas de 0.002
      pid_i = 0;
    }
    if (c == 'i') {
      ki = ki - 0.0001;
      pid_i = 0;
    }

    if (c == 'D') {
      kd = kd + 0.1;  // par pas de 0.001
    }
    if (c == 'd') {
      kd = kd - 0.1;
    }

    if (c == 'T') {
      throttle = throttle + 5;  // par pas de 0.001
      if (throttle > 2000) {
        throttle = 2000;
      }
    }
    if (c == 't') {
      throttle = throttle - 5;
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

  /*The tiemStep is the time that elapsed since the previous loop.
   * This is the value that we will use in the formulas as "elapsedTime"
   * in seconds. We work in ms so we haveto divide the value by 1000
   to obtain seconds*/

  /*Reed the values that the accelerometre gives.
   * We know that the slave adress for this IMU is 0x68 in
   * hexadecimal. For that in the RequestFrom and the
   * begin functions we have to put this value.*/

  // ********************  read MPU6050  **********************************
  while (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
  }  // wit for new reads it returns 0 or 1
  // if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  ypr[0] = ypr[0] * -180.0 / M_PI;  // pour convertir en angles
  ypr[1] = ypr[1] * 180.0 / M_PI;   // negatif =>down
  ypr[2] =
      ypr[2] * 180.0 / M_PI;  // ori -180 pour avoir un roll a droite => negatif
  angle_degres =
      ypr[2];  // A vérifier mais d'après la photo Ca doit etre le pitch

  error = angle_degres - desired_angle;
  /* angle_degres_filtre =
      (angle_degres + angle_degres_1 + angle_degres_2 + angle_degres_3 +
       angle_degres_4 + angle_degres_5 + angle_degres_6 + angle_degres_7 +
       angle_degres_8 + angle_degres_9 + angle_degres_10) /
      11;
  // erreur estl'angle mesuré - angle voulu
  error = angle_degres_filtre - desired_angle;
  angle_degres_10 = angle_degres_9;
  angle_degres_9 = angle_degres_8;
  angle_degres_8 = angle_degres_7;
  angle_degres_7 = angle_degres_6;
  angle_degres_6 = angle_degres_5;
  angle_degres_5 = angle_degres_4;
  angle_degres_4 = angle_degres_3;
  angle_degres_3 = angle_degres_2;
  angle_degres_2 = angle_degres_1;
  angle_degres_1 = angle_degres; */

  /*First calculate the error between the desired angle and
   *the real measured angle*/
  // error = Total_angle[0] - desired_angle;

  /*Next the proportional value of the PID is just a proportional constant
   *multiplied by the error*/

  pid_p = kp * error;

  /*The integral part should only act if we are close to the
  desired position but we want to fine tune the error. That's
  why I've made a if operation for an error between -2 and 2 degree.
  To integrate we just sum the previous integral value with the
  error multiplied by  the integral constant. This will integrate (increase)
  the value each loop till we reach the 0 point*/

  // if (abs(error) < 10) {
  //     pid_i = pid_i + (ki * error);
  // }

  pid_i = pid_i + (ki * error);

  /*The last part is the derivate. The derivate acts upon the speed of the
  error. As we know the speed is the amount of error that produced in a certain
  amount of time divided by that time. For taht we will use a variable called
  previous_error. We substract that value from the actual error and divide all
  by the elapsed time. Finnaly we multiply the result by the derivate constant*/

  pid_d = kd * ((error - previous_error) / elapsedTime);

  /*The final PID values is the sum of each of this 3 parts*/
  PID = pid_p + pid_i + pid_d;

  /*We know taht the min value of PWM signal is 1000us and the max is 2000. So
  that tells us that the PID value can/s oscilate more than -1000 and 1000
  because when we have a value of 2000us the maximum value taht we could
  sybstract is 1000 and when we have a value of 1000us for the PWM sihnal, the
  maximum value that we could add is 1000 to reach the maximum 2000us*/
  if (PID < -1000) {
    PID = -1000;
  }
  if (PID > 1000) {
    PID = 1000;
  }

  /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID
   * value*/
  pwmLeft = throttle - PID;     //-35
  pwmRight = (throttle + PID);  // / 0.8207 * 0.7838;
 /*  if (pwmRight >= 1200) {
    pwmRight = pwmRight * 0.98;
  } */
  /*Once again we map the PWM values to be sure that we won't pass the min
  and max values. Yes, we've already maped the PID values. But for example, for
  throttle value of 1300, if we sum the max PID value we would have 2300us and
  that will mess up the ESC.*/
  // Right
  if (pwmRight < 1000) {
    pwmRight = 1000;
  }
  if (pwmRight > 2000) {
    pwmRight = 2000;
  }
  // Left
  if (pwmLeft < 1000) {
    pwmLeft = 1000;
  }
  if (pwmLeft > 2000) {
    pwmLeft = 2000;
  }

  /*Finnaly using the servo function we create the PWM pulses with the
  calculated width for each pulse*/
  left_prop.writeMicroseconds(pwmLeft);
  right_prop.writeMicroseconds(pwmRight);

  Serial.print(">");
  // Serial.print(Total_angle[0]);  // 1
  Serial.print(angle_degres);
  Serial.print(",");
  Serial.print(desired_angle);  // 2
  Serial.print(",");
  Serial.print(error);  // 3
  Serial.print(",");
  Serial.print(PID);  // 4
  Serial.print(",");
  Serial.print(pwmLeft);  // 5
  Serial.print(",");
  Serial.print(pwmRight);  // 6
  Serial.print(",");
  Serial.print(elapsedTime * 1000.0, 4);  // 7
  Serial.print(",");
  Serial.print(kp, 5);  // 8
  Serial.print(",");
  Serial.print(ki, 5);  // 9
  Serial.print(",");
  Serial.print(kd, 5);  // 10
  Serial.print(",");
  Serial.println(throttle);  // 11

  previous_error = error;  // Remember to store the previous error.

}  // end of loop void

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
  // mpu.setFullScaleAccelRange(2);
  //  supply your own gyro offsets here, scaled for min sensitivity
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
