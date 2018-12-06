#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>
//#include <avr/io.h> ADD INTERRUPT CAPABILITY LATER?
//#include <avr/interrupt.h>

#define LOGGING_FREQUENCY 100.0
#define MAX_LOG_NUMBER 750

const int chipSelect = BUILTIN_SDCARD;
File dataFile;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

String rootFile = "datalog";
int fileIndex = 0;

class ControlObject { //contains all the needed info on a planet
public:
    imu::Vector<3> euler,gravity,lin_acc;
    double controlSetpoint;
    void logState();
    void updateState();
    void serialData();
    bool timeToLog();
    unsigned long curTime = millis();
    int numberLogs = 0;
    double updateTime = (1.0/LOGGING_FREQUENCY)*1000.0; 
};

void ControlObject::serialData() {
  String timeData = String("Time: "+String(this->curTime)+"\n");
  String eulerData = String("Euler: "+this->euler.toString()+"\n");
  String gravityData = String("Gravity: "+this->gravity.toString()+"\n");
  String linAccData = String("Linear Acceleration: "+this->lin_acc.toString()+"\n");
  String controlSetpointData = String("Setpoint: "+String(this->controlSetpoint)+"\n");

  String totalResult = String(timeData+eulerData+gravityData+linAccData+controlSetpointData);
  Serial.print(totalResult);
}

void ControlObject::updateState() {
  this->euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  this->gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  this->lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

void ControlObject::logState() {
  this->curTime = millis();
  
  String timeData = String(this->curTime);
  String eulerData = String(this->euler.toString());
  String gravityData = String(this->gravity.toString());
  String linAccData = String(this->lin_acc.toString());
  String controlSetpointData = String(this->controlSetpoint);

  String totalResult = String(timeData+","+eulerData+","+gravityData+","+linAccData+","+controlSetpointData);
  dataFile.println(totalResult);
  dataFile.flush();
  this->numberLogs = this->numberLogs + 1;
    
}

bool ControlObject::timeToLog(){
  return ( (this->numberLogs < MAX_LOG_NUMBER) && ((millis()-(this->curTime)) > updateTime) );
}

//ISR(TIMER0_OVF_vect)
//{
//    control_obj.logState()
//}

const char* string2char(const String& command){
  return command.c_str();
}

ControlObject control_obj;

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––



void setup() {
  pinMode(LED_BUILTIN , OUTPUT);

  Serial.begin(9600);
  if(!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);}

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("Card initialized!");

  while ( SD.exists( (rootFile+String(fileIndex)+String(".csv")).c_str() ) ) {
    fileIndex += 1;
  }
  
  Serial.println((rootFile+String(fileIndex)+String(".csv")).c_str());
  
  //String("testData.csv").c_str() , FILE_WRITE);
  
  dataFile = SD.open( (rootFile+String(fileIndex)+String(".csv")).c_str() , FILE_WRITE); // TODO: Add new file if "data_log.csv" already exists SD.exists()
  
  if (dataFile) {
    dataFile.println("Time,Euler_x,Euler_y,Euler_z,Grav_x,Grav_y,Grav_z,LinAcc_x,LinAcc_y,LinAcc_z,Control_Setpoint");
  } else {
    Serial.println("Data file failed to open... :(");
    while (1);
  }
    
  bno.setExtCrystalUse(true);
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_M4G); // Because the gyroscope saturates at just over 5 RPS
  
}

void loop() {
  control_obj.updateState();
  Serial.println(control_obj.curTime - millis());
  if (control_obj.timeToLog()) {
      Serial.println("LOGGING!");
      control_obj.logState();
      control_obj.serialData();
  }
}
