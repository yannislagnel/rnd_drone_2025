#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>

// A3=Throtle, Pitch=A2 et Roll=A1
#define THROTLE_PIN A3
#define PITCH_PIN A2
#define ROLL_PIN A9
#define YAW_PIN A1

#define PX_PITCH_PIN A0
#define PX_ROLL_PIN A11
#define PX_YAW_PIN A10

// doit etre le meme que pour drone
// #define PRY_MAX 200
// #define PRY_MIN -200

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
const byte address[6] = "00001";
volatile bool messageAvailable = false;

void NRF24L01_IRQ();

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

void setup()
{
    Serial.begin(115200);
    pinMode(32, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(32), NRF24L01_IRQ, FALLING);
    pinMode(THROTLE_PIN, INPUT);
    pinMode(PITCH_PIN, INPUT);
    pinMode(ROLL_PIN, INPUT);
    pinMode(YAW_PIN, INPUT);
    pinMode(PX_PITCH_PIN, INPUT);
    pinMode(PX_ROLL_PIN, INPUT);
    pinMode(PX_YAW_PIN, INPUT);

    dataToSend.throtle = analogRead(THROTLE_PIN);
    dataToSend.throtle = analogRead(THROTLE_PIN);

    dataToSend.pitch = analogRead(PITCH_PIN);
    dataToSend.pitch = analogRead(PITCH_PIN);

    dataToSend.roll = analogRead(ROLL_PIN);
    dataToSend.roll = analogRead(ROLL_PIN);

    dataToSend.yaw = analogRead(YAW_PIN);
    dataToSend.yaw = analogRead(YAW_PIN);
    // dataToSend.yaw = 0;

    dataToSend.px_pitch = analogRead(PX_PITCH_PIN);
    dataToSend.px_pitch = analogRead(PX_PITCH_PIN);

    dataToSend.px_roll = analogRead(PX_ROLL_PIN);
    dataToSend.px_roll = analogRead(PX_ROLL_PIN);

    dataToSend.px_yaw = analogRead(PX_YAW_PIN);
    dataToSend.px_yaw = analogRead(PX_YAW_PIN);

    Serial.println("checking SPI pins....");
    delay(10);
    while (!radio.begin()) {
        Serial.println(F("radio hardware not responding!"));
        delay(50);
    }
    radio.setChannel(52);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_1MBPS);
    //    radio.maskIRQ(1, 1, 0); // enable only IRQ on RX event

    radio.openWritingPipe(address);
    radio.openReadingPipe(1, address);

    radio.printDetails();
    radio.startListening();
    Serial.println(radio.available());
    Serial.println(radio.getPALevel());
    Serial.println(radio.isChipConnected());

    radio.stopListening();
    delay(2000);
}

void loop()
{
    // *************** read from drone *************************
    if (messageAvailable) {
        messageAvailable = false;
        if (radio.available()) {
            radio.read(&dataReceived, sizeof(Data));
            Serial.print("RECIEVED TEENSY\tthrotle: ");
            Serial.print(dataReceived.throtle);
            Serial.print("\tpitch: ");
            Serial.print(dataReceived.pitch);
            Serial.print("\troll: ");
            Serial.print(dataReceived.roll);
            Serial.print("\tyaw: ");
            Serial.println(dataReceived.yaw);
        }
    }
    radio.stopListening();
    delay(5);

    // *********** to send *******************

    // ******** on lit les valeurs des Analogs ************
    dataToSend.throtle = analogRead(THROTLE_PIN);
    dataToSend.throtle = analogRead(THROTLE_PIN);
    dataToSend.throtle = analogRead(THROTLE_PIN);

    if (dataToSend.throtle < 5) // pour eviter les erreurs de centrage des joysticks
        dataToSend.throtle = 0;
    dataToSend.throtle = map(dataToSend.throtle, 0, 1023, 1000, 1500);

    // ******************* TRIM pitch ***********************************************
    dataToSend.pitch = analogRead(PITCH_PIN);
    dataToSend.pitch = analogRead(PITCH_PIN);
    dataToSend.pitch = analogRead(PITCH_PIN);

    // on prend ce potentiometre pour le trim du pitch
    dataToSend.px_pitch = analogRead(PX_PITCH_PIN);
    dataToSend.px_pitch = analogRead(PX_PITCH_PIN);
    dataToSend.px_pitch = analogRead(PX_PITCH_PIN);

    // on a les valeurs du joystick pitch et on map pour le min et max
    dataToSend.pitch = map(dataToSend.pitch, 0, 1023, +100, -100); //  on mappe entre (-1 * )potentiometre4 et +potentiometre4
    // on corrige pour le centrage du joystick
    if (abs(dataToSend.pitch) < 10)
        dataToSend.pitch = 0;

    // pour le pot yaw PX_YAW_PIN on map pour avoir les valeurs min et max pour le trim
    dataToSend.px_pitch = map(dataToSend.px_pitch, 0, 1023, +100, -100); // -100 to +100 par exemple. A voir pour le sens

    // on applique le trim issue du pot PX_PITCH_PIN
    dataToSend.pitch = dataToSend.pitch + dataToSend.px_pitch;
    // on verifie qu'on est bien dans le range A ajuster comme on veut
    if (dataToSend.pitch < -200)
        dataToSend.pitch = -200;
    if (dataToSend.pitch > 200)
        dataToSend.pitch = 200;
    // ******************* FIN TRIM pitch ***********************************************

    // ******************* TRIM roll ***********************************************
    dataToSend.roll = analogRead(ROLL_PIN);
    dataToSend.roll = analogRead(ROLL_PIN);
    dataToSend.roll = analogRead(ROLL_PIN);

    // on prend ce potentiometre pour le trim du roll
    dataToSend.px_roll = analogRead(PX_ROLL_PIN);
    dataToSend.px_roll = analogRead(PX_ROLL_PIN);
    dataToSend.px_roll = analogRead(PX_ROLL_PIN);

    // on a les valeurs du joystick roll et on map pour le min et max
    dataToSend.roll = map(dataToSend.roll, 0, 1023, -100, +100); //  on mappe entre (-1 * )potentiometre4 et +potentiometre4
    // on corrige pour le centrage du joystick
    if (abs(dataToSend.roll) < 10)
        dataToSend.roll = 0;

    // pour le pot yaw PX_ROLL_PIN on map pour avoir les valeurs min et max pour le trim
    dataToSend.px_roll = map(dataToSend.px_roll, 0, 1023, -100, +100); // -100 to +100 par exemple. A voir pour le sens

    // on applique le trim issue du pot PX_ROLL_PIN
    dataToSend.roll = dataToSend.roll + dataToSend.px_roll;
    // on verifie qu'on est bien dans le range A ajuster comme on veut
    if (dataToSend.roll < -200)
        dataToSend.roll = -200;
    if (dataToSend.roll > 200)
        dataToSend.roll = 200;
    // ******************* FIN TRIM pitch ***********************************************

    // ******************* TRIM Yaw ***********************************************
    dataToSend.yaw = analogRead(YAW_PIN);
    dataToSend.yaw = analogRead(YAW_PIN);
    dataToSend.yaw = analogRead(YAW_PIN);

    // on prend ce potentiometre pour le trim du yaw
    dataToSend.px_yaw = analogRead(PX_YAW_PIN);
    dataToSend.px_yaw = analogRead(PX_YAW_PIN);
    dataToSend.px_yaw = analogRead(PX_YAW_PIN);

    // on a les valeurs du joystick yaw et on map pour le min et max
    dataToSend.yaw = map(dataToSend.yaw, 0, 1023, 90, -90); //  on mappe entre (-1 * )potentiometre4 et +potentiometre4
    // on corrige pour le centrage du joystick
    if (abs(dataToSend.yaw) < 10)
        dataToSend.yaw = 0;

    // pour le pot yaw PX_YAW_PIN on map pour avoir les valeurs min et max pour le trim
    dataToSend.px_yaw = map(dataToSend.px_yaw, 0, 1023, -90, +90); // -90 to +90 par exemple. A voir pour le sens

    // on applique le trim issue du pot PX_YAW_PIN
    dataToSend.yaw = dataToSend.yaw + dataToSend.px_yaw;
    // on verifie qu'on est bien dans le range A ajuster comme on veut
    if (dataToSend.yaw < -180)
        dataToSend.yaw = -180;
    if (dataToSend.yaw > 180)
        dataToSend.yaw = 180;
    // ******************* FIN TRIM Yaw ***********************************************

    // radio.write(&dataToSend, sizeof(Data));

    if (radio.write(&dataToSend, sizeof(Data)) == true) {
        Serial.print("Sent ok\t");
    } else {
        Serial.print("Sent Error\t");
    }

    Serial.print("Throtle: ");
    Serial.print(dataToSend.throtle);
    Serial.print("\tPitch: ");
    Serial.print(dataToSend.pitch);
    Serial.print("\t Roll: ");
    Serial.print(dataToSend.roll);
    Serial.print("\t  Yaw: ");
    Serial.print(dataToSend.yaw);
    Serial.print("\t\tPx_P: ");
    Serial.print(dataToSend.px_pitch);
    Serial.print("\tPX_R: ");
    Serial.print(dataToSend.px_roll);
    Serial.print("\tPX_Y: ");
    Serial.println(dataToSend.px_yaw);

    // delay(5);
    radio.startListening();
    delay(5);
}

void NRF24L01_IRQ() { messageAvailable = true; }
