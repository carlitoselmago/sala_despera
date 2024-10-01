// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ArduinoOSCWiFi.h>
#define readId 0 //ID 0 MASTER

#if readId==0
 const char *board = "/board0";
#else
 const char *board = "/board1";
#endif

Adafruit_MPU6050 mpu;

//Wifi configuration
/*
const char *ssid = "HANGAR_lab";
const char *password = "mordorlab";

IPAddress ip(192, 168, 30, 200 + readId);
IPAddress gateway(192, 168, 30, 1);
IPAddress subnet(255, 255, 255, 0);
// for ArduinoOSC
const char* host_ip = "192.168.30.255";
*/


const char *ssid = "( o )( o )";
const char *password = "todojuntoyenminusculas";

IPAddress ip(192, 168, 1, 200 + readId);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
const char* host_ip = "192.168.1.139";


void setup(void) {
  Serial.begin(115200);
  //while (!Serial)
    //delay(5000); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println(readId);
        if (WiFi.config(ip, gateway, subnet) == false) {
          Serial.println("Configuration failed.");
        } else {
           Serial.println("Configuration success.");
        }
    WiFi.begin(ssid, password);
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      retry++;
      if(retry==10)
        {
          Serial.print("Restarting...\n\n");
          ESP.restart();
        }
      Serial.print("Connecting...\n\n");
    }

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
      Serial.println("Failed to find MPU6050 chip");
    }
  }
  Serial.println("MPU6050 Found!");
  /*
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  
  Serial.println("");
  delay(100);

  // subscribe osc messages
  int recv_port = 55555;
  OscWiFi.subscribe(recv_port, "/curtain/start",
          [&](const int& dir_osc, const int& val) {
              Serial.print("/curtain/start ");
              Serial.print(dir_osc);
              Serial.println();
          });
          */
}

//compensation degrees
float x_c=0.2

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  OscWiFi.update();
  /* Print out the values */
  OscWiFi.send(host_ip, 54321, board,(a.acceleration.x/10,a.acceleration.y/10,a.acceleration.z/10);
  //OscWiFi.send(host_ip, 54321,board, 0,0,a.acceleration.z/10);

  delay(50);
}