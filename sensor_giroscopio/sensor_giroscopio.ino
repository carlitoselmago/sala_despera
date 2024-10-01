#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <WiFi.h>  // ESP32 Wi-Fi library
#include <OSCMessage.h>  // OSC message library
#include <WiFiUdp.h>  // UDP library for OSC
#include "Adafruit_MAX1704X.h"


Adafruit_MAX17048 maxlipo;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;

// Wi-Fi credentials
const char *ssid = "MANGO";
const char *password = "remotamente";

// OSC settings
const IPAddress outIp(192, 168, 5, 255);  // Replace with the IP of your OSC receiver
const int outPort = 54321;  // OSC receiver port

WiFiUDP Udp;  // UDP instance for sending OSC

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Interrupt flag
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

// Connect to Wi-Fi
void connectToWiFi() {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void setup() {



    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #endif

    Serial.begin(115200);
    while (!Serial);

     // battery reader
    while (!maxlipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
      delay(2000);
    }
    Serial.print(F("Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(maxlipo.getChipID(), HEX);

    // Connect to Wi-Fi
    connectToWiFi();

    // Initialize MPU6050
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    if (mpu.testConnection()) {
        Serial.println(F("MPU6050 connection successful"));
    } else {
        Serial.println(F("MPU6050 connection failed"));
    }

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(11);
    mpu.setYGyroOffset(-51);
    mpu.setZGyroOffset(-19);
    mpu.setZAccelOffset(2048);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(LED_PIN, OUTPUT);
}

void sendOSCMessage(char* oscAddress,float yaw, float pitch, float roll) {
    OSCMessage msg(oscAddress);
    msg.add(yaw).add(pitch).add(roll);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);  // send the OSC message
    Udp.endPacket();
    msg.empty();  // empty the message to free up memory
}

// battery vars
//const int MAX_ANALOG_VAL = 4095;
//const float MAX_BATTERY_VOLTAGE = 4.2; // Max  voltage of a 3.7 battery is 4.2

int loopCounter = 0; // Counter to track the number of loops
const int loopInterval = 100; // Number of loops to wait

void loop() {

    loopCounter++; // Increment the loop counter

    if (loopCounter >= loopInterval) {

      //int rawValue = analogRead(A13);
      //float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3; // calculate voltage level
      //float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
      float batteryFraction=  maxlipo.cellPercent();
      //Serial.println(batteryFraction*100);
      //cpu temperature
      float cpu_celsius = temperatureRead();

       //send temperature and battery charge
      sendOSCMessage("/control",cpu_celsius,batteryFraction,0);
      // Reset the counter
      loopCounter = 0;
    }

    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float yaw = ypr[0] * 180 / M_PI;
        float pitch = ypr[1] * 180 / M_PI;
        float roll = ypr[2] * 180 / M_PI;

        // Send the Yaw, Pitch, Roll data via OSC
        sendOSCMessage("/coixi",yaw, pitch, roll);

       
        /*
        Serial.print("ypr\t");
        Serial.print(yaw);
        Serial.print("\t");
        Serial.print(pitch);
        Serial.print("\t");
        Serial.println(roll);

        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        */
    }
     delay(50);
}
