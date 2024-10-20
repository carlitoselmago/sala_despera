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
bool softsleep=false;
float lastypr[3]={0.0,0.0,0.0};
float deltathreshold=150;
float acomulateddelta=deltathreshold;
float deltadiscountrate=2;
float delta=0.0;

//loop speed
int loopspeed=50;
int originalloopspeed=loopspeed;

// difference vals
float yprsum;
float lastyprsum;
float yprdifference;

float acceleration;
float acomulatedacceleration;

float yaw_adjusted;
float pitch_adjusted;
float roll_adjusted;


//control vars
float batteryFraction;
float cpu_celsius;

//orientation vars
float yaw;
float pitch;
float roll;

bool blinkState = false;

//raw accelerometer / gyro vars
int16_t ax, ay, az;
int16_t gx, gy, gz;


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
    //WiFi.begin(ssid, NULL);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

}

void blink(){
   // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

void setup() {

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #endif

    Serial.begin(115200);
    //while (!Serial);
    unsigned long start = millis();
    //while (!Serial && millis() - start < 5000); // Timeout after 5 seconds
    
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
    //calibrate sensor
    //calibratesensor();
     devStatus = mpu.dmpInitialize();

   
    mpu.setXGyroOffset(11);//11
    mpu.setYGyroOffset(-51);//-51 // Este controla el norte, ajustar en la miro
    mpu.setZGyroOffset(-19);//-19
    mpu.setZAccelOffset(2048);//2048
   
   

    if (devStatus == 0) {
        //mpu.CalibrateAccel(6);
        //mpu.CalibrateGyro(6);
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

    //blink();
}

void sendOSCMessage(char* oscAddress,float yaw, float pitch, float roll,float status) {
    OSCMessage msg(oscAddress);
    msg.add(yaw).add(pitch).add(roll).add(status);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);  // send the OSC message
    Udp.endPacket();
    msg.empty();  // empty the message to free up memory
}

float accvalue(){
       //accelerometer readings
       //computed ones
      
      /*
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      */
      //simpler raw ones
      //mpu.getAcceleration(&ax, &ay, &az);
      mpu.getRotation(&gx, &gy, &gz);

      //Serial.print("gyroraw:\t");
      //Serial.print(gx); Serial.print("\t");
      //Serial.print(gy); Serial.print("\t");
     
      //Serial.println(":::");
      /*
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);
      */
      return abs(gx)+abs(gy);
}



int loopCounter = 0; // Counter to track the number of loops
const int loopInterval = 1000; // Number of loops to wait

void loop() {
    if (!dmpReady) return;

    loopCounter++; // Increment the loop counter
    if (awake){
      if (loopCounter >= loopInterval) {
        
        //send battery level
        batteryFraction=  maxlipo.cellPercent();
        cpu_celsius = temperatureRead();

        //send temperature and battery charge
        sendOSCMessage("/control",cpu_celsius,batteryFraction,0,float(awake));

        // Reset the counter
        loopCounter = 0;

       
        //check for acomulated acceleration
        //Serial.print("acomulated acceleration: ");
        //Serial.println(acomulatedacceleration);

        //hard sleep mode
        if (acomulatedacceleration<53000.0){
          //activate sleep mode
          awake=false;
          loopspeed=2000;

          yaw_adjusted=yaw;
          pitch_adjusted=pitch;
          roll_adjusted=roll;
        
          Serial.println("SLEEP MODE ACTIVATED:::::::::::::::::::::::::::::::::::::");
          sendOSCMessage("/coixi", 0.0,0.0, 0.0,0.0);
          WiFi.disconnect();
        }

    
        acomulatedacceleration=0.0;
      }
     
      //soft sleep control
      if (1==1){
        //softsleep=true;
      }
    }

   

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

      //acceleration readings
      acceleration=accvalue();
      acomulatedacceleration+=acceleration; 

      if (awake){
        // orientation Readings
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        //yaw = (ypr[0] * 180 / M_PI)-yaw_adjusted;
        //pitch = (ypr[1] * 180 / M_PI)-pitch_adjusted;
        //roll = (ypr[2] * 180 / M_PI)-roll_adjusted;
        yaw = (ypr[0] * 180 / M_PI)*-1;
        pitch = (ypr[1]  *180 / M_PI)*-1; //avanzar
        roll = (ypr[2] * 180 / M_PI)*-1;
        if (softsleep==false){
          //delta difference
          delta=( abs(yaw-lastypr[0]) + abs(pitch-lastypr[1]) + abs(roll-lastypr[2]) );
          //Serial.print("delta: ");
          //Serial.println(delta);
          acomulateddelta+=delta;
          acomulateddelta-=deltadiscountrate;
          if (acomulateddelta>deltathreshold){acomulateddelta=deltathreshold;}
          //Serial.print("acomulateddelta: ");
          //Serial.println(acomulateddelta);
          if (acomulateddelta<0){
            Serial.println("SOFT SLEEP ACTIVATED:::");
            softsleep=true;
            yaw_adjusted=yaw;
            pitch_adjusted=pitch;
            roll_adjusted=roll;
            sendOSCMessage("/coixi", 0.0,0.0, 0.0,2.0);
            acomulateddelta=deltathreshold;
          }
       
          lastypr[0]=yaw;
          lastypr[1]=pitch;
          lastypr[2]=roll;
        }
        if (!softsleep){
          sendOSCMessage("/coixi", yaw, pitch, roll,float(awake));
        } else {
          //during softsleep
          //Serial.print("acceleration:: ");
          //Serial.println(acceleration);
          if (acceleration>300){
            softsleep=false;
          }
        }

      } else {
        //deep sleeping
         
        connectToWiFi();
        awake=true;
        loopspeed=originalloopspeed;
        Serial.println("WAKE UP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        //calibratesensor();
        
        //recalibrate sensor
        //Serial.println("reca
        Serial.print("acceleration: ");
        Serial.println(acceleration);
        if (acceleration>300){
          //wake up!librating sensor:::::");

    
      }
      }
  }
     delay(loopspeed);
     //blink();
}


