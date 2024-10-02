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

bool awake=true;
float lastypr[3]={0.0,0.0,0.0};
float acomulateddelta=0.0;

//loop speed
int loopspeed=50;

// difference vals
float yprsum;
float lastyprsum;
float yprdifference;

float acceleration;
float acomulatedacceleration;

//control vars
float batteryFraction;
float cpu_celsius;

//orientation vars
float yaw;
float pitch;
float roll;

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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; 
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
const int loopInterval = 1000; // Number of loops to wait

void loop() {
    if (!dmpReady) return;

    loopCounter++; // Increment the loop counter
    if (awake){
      if (loopCounter >= loopInterval) {
        
        batteryFraction=  maxlipo.cellPercent();
        cpu_celsius = temperatureRead();

        //send temperature and battery charge
        sendOSCMessage("/control",cpu_celsius,batteryFraction,0);
        // Reset the counter
        loopCounter = 0;

        /*
        //check for acomulateddelta
        Serial.print("acomulateddelta: ");
        Serial.println(acomulateddelta);
        if (acomulateddelta<30.0){
          //activate sleep mode
          awake=false;
          //loopspeed=500;
          Serial.println("SLEEP MODE ACTIVATED:::::::::::::::::::::::::::::::::::::");
        }
        acomulateddelta=0.0;
        */

        //check for acomulated acceleration

 
        Serial.print("acomulated acceleration: ");
        Serial.println(acomulatedacceleration);
        if (acomulatedacceleration<10000.0){
          //activate sleep mode
          awake=false;
          loopspeed=2000;
          Serial.println("SLEEP MODE ACTIVATED:::::::::::::::::::::::::::::::::::::");
        }
        acomulatedacceleration=0.0;
      }
    }

   

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

      /*
     
          yprdifference = 0.0;
          
          for (int i = 0; i < 3; i++) {
              yprdifference += abs(ypr[i] - lastypr[i]);
          }

          acomulateddelta += yprdifference;

          if (!awake) {
              // wakeup check with slower loop
              //Serial.println(acomulateddelta);
          }
      
      */
      //acceleration readings
      acceleration=accvalue();
      acomulatedacceleration+=acceleration; 

      if (awake){
        // orientation Readings
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        yaw = ypr[0] * 180 / M_PI;
        pitch = ypr[1] * 180 / M_PI;
        roll = ypr[2] * 180 / M_PI;

        if (awake) {
            sendOSCMessage("/coixi", yaw, pitch, roll);
        }
      } else {
        //sleeping
        Serial.print("acceleration: ");
        Serial.println(acceleration);
      }
      // Update lastypr[] AFTER calculating the difference
      //Serial.println(ypr[0]);
      /*
      lastypr[0] = ypr[0];
      lastypr[1] = ypr[1];
      lastypr[2] = ypr[2];
      */
  }
     delay(loopspeed);
}

float accvalue(){
       //accelerometer readings
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      /*
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);
      */
      return abs(aaReal.x)+abs(aaReal.y);
}
