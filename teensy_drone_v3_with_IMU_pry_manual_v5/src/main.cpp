#include <Arduino.h>
#include <Servo.h>

#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>

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

#define PIN_SERVO_FL 36
#define PIN_SERVO_RL 15
#define PIN_SERVO_RR 9
#define PIN_SERVO_FR 28

#define PIN_SERVO_AL 33
#define PIN_SERVO_AR 29
#define PIN_SERVO_D 6
#define PIN_SERVO_P 5

Servo ESC_FL;
Servo ESC_RL;
Servo ESC_RR;
Servo ESC_FR;

int PWM_FL = 1000;
int PWM_RL = 1000;
int PWM_RR = 1000;
int PWM_FR = 1000;

Servo servo_FL;
Servo servo_RL;
Servo servo_RR;
Servo servo_FR;

int servo_pwm_FL = 0;
int servo_pwm_RL = 0;
int servo_pwm_RR = 0;
int servo_pwm_FR = 0;

Servo servo_AL;
Servo servo_AR;
Servo servo_D;
Servo servo_P;

int pwm_AL = 0;
int pwm_AR = 0;
int pwm_D = 0;
int pwm_P = 0;

int throtle = 1000;
int pitch = 0;
int roll = 0;
int yaw = 0;

// doit etre le meme que pour le controleur
#define PRY_MAX 15
#define PRY_MIN -15

int counter_recieved = 0;
int counter_loop = 0;
bool faute_recieved = false;

// ================================================================
// ===            NRF24L01 Variable declaration                ====
// ================================================================
/*
connection Teensy 4 a NRF24L01
GND  -> GND
VCC  -> 3.3V (mettre une capa de 10uF)
CE   -> 31 select RX/TX mode (non lie a SPI donc peut etre changee)
CSN  -> 10 chip select SPI
SCK  -> 13
MOSI -> 11
MISO -> 12
IRQ  -> 32  (non lie a SPI donc peut etre changee)
*/
RF24 radio(31, 10); // CE, CSN
const byte address[6] = "00001"; // must be the same on both NRF
volatile bool messageAvailable = false; // will be true if we recieved something

// structure data to send or recieve

struct Data {
    int throtle;
    int roll;
    int pitch;
    int yaw;
    int px_pitch;
    int px_roll;
    int px_yaw;
};
// Data dataToSend = { 0, 0, 0, 0 };
// Data dataReceived = { 0, 0, 0, 0 };

Data dataToSend = { 0, 0, 0, 0, 0, 0, 0 };
Data dataReceived = { 0, 0, 0, 0, 0, 0, 0 };

//*********************** FIN NRF ********************************

//*************************** IMU MPU6050 *******************************
// with the DMP using I2Cdev lib
MPU6050 mpu; // MPU6050 mpu(0x69); // <-- use for AD0 high
#define INTERRUPT_PIN GYRO_INT // use pin 34 for teensy could be any digital
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q; // [w, x, y, z]         quaternion container
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16 aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3]; // [psi, theta, phi]    Euler angle container
float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===              PID Variable declaration                   ====
// ================================================================
//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, roll_error;
float roll_kp = 1.25;

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error;
float pitch_kp = 1.25;

//////////////////////////////PID FOR YAW//////////////////////////
float yaw_PID, yaw_error;
float yaw_kp = 1.0;

// ================================================================
// ===               fonction declaration                      ====
// ================================================================
void init_esc_cal(void);
void init_esc(void);
void init_servos_esc(void);
void init_servos_avion(void);
void NRF24L01_IRQ(void);
void setup_mpu6050(void);

// ===  INTERRUPT MPU6050 DETECTION ROUTINE
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
    Serial.begin(115200);
    // --------- init the ESCs, servos -------------------------------
    Serial.println("init ESC and servos...");
    init_esc(); // sans calibration en premier et on met le Throtle a 1000 (zero) 4 moteurs stop
    // init_esc_cal(); // avec calibration des ESC
    PWM_FL = PWM_RL = PWM_RR = PWM_FR = 1000;
    ESC_FL.writeMicroseconds(PWM_FL);
    ESC_RL.writeMicroseconds(PWM_RL);
    ESC_RR.writeMicroseconds(PWM_RR);
    ESC_FR.writeMicroseconds(PWM_FR);
    init_servos_esc(); // les 4 servos pour les 4 moteurs
    delay(500);
    // on ajuste pour etre perpendiculaires
    servo_pwm_FL = 90;
    servo_pwm_FR = 90;
    servo_pwm_RL = 85;
    servo_pwm_RR = 85;
    servo_FL.write(servo_pwm_FL);
    servo_RL.write(servo_pwm_RL);
    servo_RR.write(servo_pwm_RR);
    servo_FR.write(servo_pwm_FR);
    delay(500);
    init_servos_avion(); // pour les servos mode avion
    delay(1000);

    // ********* setup MPU6050 non utilise pour le vol ****************************************
    Serial.println("init MPU6050 ...");
    ypr[0] = 0.0;
    ypr[1] = 0.0;
    ypr[2] = 0.0;
    setup_mpu6050(); // init le MPU6050

    //********************* SET UP NRF **********************************************
    // set interrupt
    pinMode(32, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(32), NRF24L01_IRQ, FALLING);
    Serial.println("init NRF24L01 ...");
    Serial.println("checking SPI pins....");

    radio.begin();
    delay(1);
    radio.powerUp();
    delay(1);
    radio.setChannel(52); // we define the optimal channel
    delay(1);
    radio.setPALevel(RF24_PA_HIGH); // set the TX power output
    delay(1);
    radio.setDataRate(RF24_1MBPS); // set kbits/second : the lowest => max distance
    delay(1);
    radio.maskIRQ(1, 1, 0); // enable only IRQ on RX event

    radio.openWritingPipe(address);
    radio.openReadingPipe(1, address);

    radio.startListening();
    Serial.println(radio.available());
    Serial.println(radio.getPALevel());
    Serial.println(radio.isChipConnected());
    radio.printDetails();
    //********************* FIN SET UP NRF ****************

    //********** Attente de reception throtle=1000 ********
    // pour la securite on ne commence pas tant que on ne recoit pas
    // 1000 pour le throtle
    delay(5000);
    Serial.println("\nOn attend de recevoir 1000 de throtle de la telecontrole...");
    throtle = 0;
    while (throtle != 1000) {
        if (messageAvailable) { // if we recieved something
            messageAvailable = false;
            if (radio.available()) { // we check if data recieved are available
                radio.read(&dataReceived, sizeof(Data)); // get the data
                throtle = dataReceived.throtle;
                Serial.print("setup throtle: ");
                Serial.println(throtle);
            }
        }
        delay(10);
    }

    pitch_kp = (float)dataReceived.px_pitch / 1000.00; // 100 to 2500 =>0.1 to 2.5
    roll_kp = (float)dataReceived.px_roll / 1000.00;
    yaw_kp = (float)dataReceived.px_yaw / 1000.00;

    delay(5000); // Attente pour s'eloigner!!!
    Serial.println("\nEND du Setup .....");
    pwm_AL = 25;
    servo_pwm_FL == 90;
}

int compteur_ailreons = 0;
int compteur_esc = 0;

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    //************************ NRF *************************
    if (messageAvailable) { // if we recieved something
        messageAvailable = false;
        if (radio.available()) { // we check if data recieved are available
            counter_recieved++;
            radio.read(&dataReceived, sizeof(Data)); // get the data
            throtle = dataReceived.throtle;
            pitch = dataReceived.pitch;
            roll = dataReceived.roll;
            yaw = dataReceived.yaw;

            pitch_kp = (float)dataReceived.px_pitch / 1000.00; // 100 to 2500 =>0.1 to 2.5
            roll_kp = (float)dataReceived.px_roll / 1000.00;
            yaw_kp = (float)dataReceived.px_yaw / 1000.00;
        }
    }
    //************************ FIN NRF *********************

    //************  read MPU6050  *********************
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        ypr[0] = ypr[0] * -180.0 / M_PI; // pour convertir en angles
        ypr[1] = ypr[1] * 180.0 / M_PI; // negatif =>down
        ypr[2] = ypr[2] * -180.0 / M_PI; // pour avoir un roll a droite => negatif

/* 
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
 */
    }

    // **************** set motors speed *****************
    // only with proportionnal and 0 roll and 0 pitch
    // to test we say all 0
    // pitch = 0;
    // roll = 0;
    // yaw = 0;

    // voir notre doc mpu_capteurs.docx page 6
    PWM_FR = throtle - roll - pitch - yaw; // motor 1
    PWM_RR = throtle - roll + pitch + yaw; // motor 2
    PWM_RL = throtle + roll + pitch - yaw; // motor 3
    PWM_FL = throtle + roll - pitch + yaw; // motor 4

    if (PWM_FR < 1000) {
        PWM_FR = 1000;
    }
    if (PWM_FR > 1800) {
        PWM_FR = 1800;
    }

    if (PWM_RR < 1000) {
        PWM_RR = 1000;
    }
    if (PWM_RR > 1800) {
        PWM_RR = 1800;
    }
    if (PWM_RL < 1000) {
        PWM_RL = 1000;
    }
    if (PWM_RL > 1800) {
        PWM_RL = 1800;
    }

    if (PWM_FL < 1000) {
        PWM_FL = 1000;
    }
    if (PWM_FL > 1800) {
        PWM_FL = 1800;
    }

    // Pour la securite: apres 10 tours de boucles
    // si on a pas recu de nouvelle du jostick on met tous les moteurs a l'arret
    if (counter_loop > 10) { // au bout de 10 tours (~100ms) on check si on a recu
        if (counter_recieved < 1) { // si on a recu moins de 1 trame de l'emmeteur
            faute_recieved = true;
            Serial.println("Lost connection ...");
        } else {
            faute_recieved = false;
        }
        counter_loop = 0;
        counter_recieved = 0;
    }
    // Serial.println(" ok from loop");
    if (faute_recieved == true) { // si on a recu moins de 1 trame de l'emmeteur on met tous le smoteurs a 0
        ESC_FL.writeMicroseconds(1000);
        ESC_RL.writeMicroseconds(1000);
        ESC_RR.writeMicroseconds(1000);
        ESC_FR.writeMicroseconds(1000);
    } else { // sinon on met les bonnes valeurs
        ESC_FL.writeMicroseconds(PWM_FL);
        ESC_RL.writeMicroseconds(PWM_RL);
        ESC_RR.writeMicroseconds(PWM_RR);
        ESC_FR.writeMicroseconds(PWM_FR);
    }

 // ************************* for debuging /monitoring ****************************************
    /*
        Serial.print("Throtle: ");
        Serial.print(throtle);
        Serial.print("\tpitch: ");
        Serial.print(pitch);
        Serial.print("\t Roll: ");
        Serial.print(roll);
        Serial.print("\t  Yaw: ");
        Serial.print(yaw);

     */

    /*  Serial.print("\tP: ");
     Serial.print(ypr[1]);
     Serial.print("\tR: ");
     Serial.print(ypr[2]);
     Serial.print("\tY: ");
     Serial.print(ypr[0]);

     Serial.print("\t\tareal\t");
     Serial.print(aaReal.x);
     Serial.print("\t");
     Serial.print(aaReal.y);
     Serial.print("\t");
     Serial.print(aaReal.z);

     if (abs(aaWorld.x) < 100)
         aaWorld.x = 0;
     if (abs(aaWorld.y) < 100)
         aaWorld.y = 0;
     if (abs(aaWorld.z) < 50)
         aaWorld.z = 0;

     Serial.print("\t\tAworld\t");
     Serial.print(aaWorld.x);
     Serial.print("\t");
     Serial.print(aaWorld.y);
     Serial.print("\t");
     Serial.println(aaWorld.z);

     Serial.print(aaReal.x);
     Serial.print("\t");
     Serial.print(aaReal.y);
     Serial.print("\t");
     Serial.print(aaReal.z);

      if (abs(aaWorld.x) < 100)
         aaWorld.x = 0;
     if (abs(aaWorld.y) < 100)
         aaWorld.y = 0;
     if (abs(aaWorld.z) < 50)
         aaWorld.z = 0;
     Serial.print("\t");
     Serial.print(aaWorld.x);
     Serial.print("\t");
     Serial.print(aaWorld.y);
     Serial.print("\t");
     Serial.println(aaWorld.z);
 */

    /*
        Serial.print("\t\tPx_P: ");
        Serial.print(pitch_kp);
        Serial.print("\tPx_R: ");
        Serial.print(roll_kp);
        Serial.print("\tPx_Y: ");
        Serial.print(yaw_kp);

        Serial.print("\t\tFL: ");
        Serial.print(PWM_FL);
        Serial.print("\tFR: ");
        Serial.print(PWM_FR);
        Serial.print("\tRL: ");
        Serial.print(PWM_RL);
        Serial.print("\tRR: ");
        Serial.println(PWM_RR); */
// ************************* END for debuging ****************************************

    // ************************* on fait bouger les ailerons toutes les 2 secondes *********************
    /*    compteur_ailreons++;
       if (compteur_ailreons > 200) { // 200 * 10mS =2sec
           if (pwm_AL == 140) {
               pwm_AL = 50;
               pwm_AR = 140;
           } else {
               pwm_AL = 140;
               pwm_AR = 50;
           }
           // pwm_D = pwm_P = 90;
           servo_AL.write(pwm_AL);
           servo_AR.write(pwm_AR);
           // servo_D.write(pwm_D);
           // servo_P.write(pwm_P);
           compteur_ailreons = 0; // reset le compteur
       }
    */
/* 
    compteur_esc++;
    if (compteur_esc > 1000) { // 1000 * 10mS =10sec
        if (servo_pwm_FL == 90) {
            servo_pwm_FL = servo_pwm_FR = servo_pwm_RL = servo_pwm_RR = 60;
            servo_FL.write(servo_pwm_FL);
            servo_RL.write(servo_pwm_RL);
            servo_RR.write(servo_pwm_RR);
            servo_FR.write(servo_pwm_FR);
            delay(200);
            servo_pwm_FL = servo_pwm_FR = servo_pwm_RL = servo_pwm_RR = 30;
            servo_FL.write(servo_pwm_FL);
            servo_RL.write(servo_pwm_RL);
            servo_RR.write(servo_pwm_RR);
            servo_FR.write(servo_pwm_FR);
            delay(200);

            servo_pwm_FL = servo_pwm_FR = servo_pwm_RL = servo_pwm_RR = 0;
            servo_FL.write(servo_pwm_FL);
            servo_RL.write(servo_pwm_RL);
            servo_RR.write(servo_pwm_RR);
            servo_FR.write(servo_pwm_FR);
            delay(200);

        } else {
            servo_pwm_FL = servo_pwm_FR = servo_pwm_RL = servo_pwm_RR = 30;
            servo_FL.write(servo_pwm_FL);
            servo_RL.write(servo_pwm_RL);
            servo_RR.write(servo_pwm_RR);
            servo_FR.write(servo_pwm_FR);
            delay(200);
            servo_pwm_FL = servo_pwm_FR = servo_pwm_RL = servo_pwm_RR = 60;
            servo_FL.write(servo_pwm_FL);
            servo_RL.write(servo_pwm_RL);
            servo_RR.write(servo_pwm_RR);
            servo_FR.write(servo_pwm_FR);
            delay(200);

            servo_pwm_FL = servo_pwm_FR = servo_pwm_RL = servo_pwm_RR = 90;
            servo_FL.write(servo_pwm_FL);
            servo_RL.write(servo_pwm_RL);
            servo_RR.write(servo_pwm_RR);
            servo_FR.write(servo_pwm_FR);
            delay(200);
        }
        compteur_esc = 0; // reset le compteur
    }
 */
    delay(10);
    counter_loop++; // on incremente le compteur de la boucle
}

void init_esc_cal(void)
{
    /* while (1)
    {
      throtle = analogRead(A3);
      Serial.println(throtle);
      delay(250);
    } */
    // Servo for the ESC
    ESC_FL.attach(
        PIN_FL, 1000,
        2000); // FL range 1000 usec =full stop to 2000 usec =>full speed

    ESC_RL.attach(
        PIN_RL, 1000,
        2000); // RL range 1000 usec =full stop to 2000 usec =>full speed

    ESC_RR.attach(
        PIN_RR, 1000,
        2000); // RR range 1000 usec =full stop to 2000 usec =>full speed

    ESC_FR.attach(
        PIN_FR, 1000,
        2000); // FR range 1000 usec =full stop to 2000 usec =>full speed

    PWM_FL = PWM_RL = PWM_RR = PWM_FR = 2000;
    ESC_FL.writeMicroseconds(PWM_FL);
    ESC_RL.writeMicroseconds(PWM_RL);
    ESC_RR.writeMicroseconds(PWM_RR);
    ESC_FR.writeMicroseconds(PWM_FR);

    delay(100);
    Serial.println(
        " mettre la battery et attendre les bips et ensuite appuez sur une "
        "touche");
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

void init_esc(void)
{
    // Servo for the ESC
    ESC_FL.attach(PIN_FL, 1000, 2000); // FL range 1000 usec =full stop to 2000 usec =>full speed
    ESC_RL.attach(PIN_RL, 1000, 2000); // RL range 1000 usec =full stop to 2000 usec =>full speed
    ESC_RR.attach(PIN_RR, 1000, 2000); // RR range 1000 usec =full stop to 2000 usec =>full speed
    ESC_FR.attach(PIN_FR, 1000, 2000); // FR range 1000 usec =full stop to 2000 usec =>full speed

    PWM_FL = PWM_RL = PWM_RR = PWM_FR = 1000;
    ESC_FL.writeMicroseconds(PWM_FL);
    ESC_RL.writeMicroseconds(PWM_RL);
    ESC_RR.writeMicroseconds(PWM_RR);
    ESC_FR.writeMicroseconds(PWM_FR);

    delay(5000);
}

void init_servos_esc(void)
{
    servo_FL.attach(PIN_SERVO_FL);
    servo_RL.attach(PIN_SERVO_RL);
    servo_RR.attach(PIN_SERVO_RR);
    servo_FR.attach(PIN_SERVO_FR);

    servo_pwm_FL = servo_pwm_RL = servo_pwm_RR = servo_pwm_FR = 90;
    servo_FL.write(servo_pwm_FL);
    servo_RL.write(servo_pwm_RL);
    servo_RR.write(servo_pwm_RR);
    servo_FR.write(servo_pwm_FR);

    delay(5);
}

void init_servos_avion(void)
{
    servo_AL.attach(PIN_SERVO_AL);
    servo_AR.attach(PIN_SERVO_AR);
    servo_D.attach(PIN_SERVO_D);
    servo_P.attach(PIN_SERVO_P);

    pwm_AL = pwm_AR = pwm_D = pwm_P = 90;
    servo_AL.write(pwm_AL);
    servo_AR.write(pwm_AR);
    servo_D.write(pwm_D);
    servo_P.write(pwm_P);

    delay(5);
}

// *********************** pour NRF et fonction appeller uniquement quand le NRF
// a recu un truc *****************
void NRF24L01_IRQ() { messageAvailable = true; }

void setup_mpu6050(void)
{

    // ************************ Setup  MPU6050 **************************************************
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock.
    // ----- initialize device -------------------------
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // ------ load and configure the DMP --------------
    delay(100);
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
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
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
