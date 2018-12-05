#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PWMServo.h>
#include <SPI.h>
#include <SD.h>


Sd2Card card;
SdVolume volume;
SdFile root;

PWMServo servo;  // create servo object to control a servo
Adafruit_BNO055 bno = Adafruit_BNO055(55); 

const int SERVO_PIN = 30 //Teensy PWM Pin

void dataLog(imu::Vector<3> euler,imu::Vector<3> gravity,imu::Vector<3> lin_acc) {
  
}

void setup() {
  pinMode(LED_BUILTIN , OUTPUT);
  servo.attach(SERVO_PIN); // some motors need min/max setting (second and third argument)

  Serial.begin(9600);
  if(!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);}
    
  bno.setExtCrystalUse(true);
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_M4G); // Because the gyroscope saturates at just over 5 RPS
  
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  //myservo.write(pos); // 0 to 180 degrees
}
