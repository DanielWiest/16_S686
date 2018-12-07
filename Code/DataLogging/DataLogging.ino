#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
//#include <avr/io.h> ADD INTERRUPT CAPABILITY LATER?
//#include <avr/interrupt.h>

#define LOGGING_FREQUENCY 100.0
#define MAX_LOG_NUMBER 850

const int chipSelect = BUILTIN_SDCARD;
File dataFile;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

String rootFile = "datalog";
int fileIndex = 0;

imu::Vector<3> upwardsVector(0.0,0.0,1.0);

class ControlObject { //contains all the needed info on a planet
public:
    imu::Vector<3> euler,gravity,lin_acc,initialThrowDirection; // initDir must be a unit vector!
    imu::Quaternion quat;
    double errorAngle;
    double controlSetpoint;
    void printErrorTerm();
    void logState();
    void updateState();
    void serialData();
    void gravityAngles();
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
  String errorValueData = String("Error Angle: "+String(this->errorAngle)+"\n");

  String totalResult = String(timeData+eulerData+gravityData+linAccData+controlSetpointData+errorValueData);
  Serial.print(totalResult);
}

void ControlObject::updateState() {
  this->euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  this->quat = bno.getQuat();
  this->gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  this->lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  this->gravity = this->gravity.scale(-1.0); // GRAVITY CORRECTION???
  
  imu::Vector<3> rot_state = this->quat.rotateVector(upwardsVector);
  imu::Vector<3> projRotOntoPlane = (rot_state - this->initialThrowDirection*(rot_state.dot(this->initialThrowDirection)));
  imu::Vector<3> projGravOntoPlane = (this->gravity - this->initialThrowDirection*(this->gravity.dot(this->initialThrowDirection)));
  
  double magAB = projRotOntoPlane.magnitude()*projGravOntoPlane.magnitude();
  if (magAB == 0.0) {
    magAB = 0.0000001;
  }
  double thetaFromCos = acos( (projRotOntoPlane.dot(projGravOntoPlane))/(magAB));

  //       sin(theta) = ((normal X a) dot b)/(mag a * mag b)
  double sinTheta = (((this->initialThrowDirection.cross(projGravOntoPlane)).dot(projRotOntoPlane))/magAB);

  if (sinTheta >= 0.0) {
      this->errorAngle = thetaFromCos - PI;
  } else {
      this->errorAngle = (PI-thetaFromCos);
  }
}

void ControlObject::printErrorTerm() {
  Serial.println(String(this->errorAngle));
}

void ControlObject::logState() {
  this->curTime = millis();
  
  String timeData = String(this->curTime);
  String eulerData = String(this->euler.toString());
  String gravityData = String(this->gravity.toString());
  String linAccData = String(this->lin_acc.toString());
  String controlSetpointData = String(this->controlSetpoint);
  String errorValueData = String(this->errorAngle);

  String totalResult = String(timeData+","+eulerData+","+gravityData+","+linAccData+","+controlSetpointData+errorValueData);
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

uint8_t *system_status;
uint8_t *self_test_result;
uint8_t *system_error;

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––



void setup() {
  pinMode(LED_BUILTIN , OUTPUT);

  Serial.begin(115200);
  if(!bno.begin()){
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);}

  bno.setExtCrystalUse(true);
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_M4G); // Because the gyroscope saturates at just over 5 RPS
    
  bno.getSystemStatus(system_status, self_test_result, system_error);
  Serial.println("ACC SELF TESTS TO FOLLOW:");
  Serial.println(*system_status,HEX);
  Serial.println(*self_test_result,BIN);
  Serial.println(*system_status,HEX);
  
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);} else {
  Serial.println("Card initialized!");
    }

  while ( SD.exists( (rootFile+String(fileIndex)+String(".csv")).c_str() ) ) {
    fileIndex += 1;}
  
  Serial.println((rootFile+String(fileIndex)+String(".csv")).c_str());
  
  
  dataFile = SD.open( (rootFile+String(fileIndex)+String(".csv")).c_str() , FILE_WRITE);
  
  if (dataFile) {
    dataFile.println("Time,Euler_yaw,Euler_roll,Euler_pitch,Grav_x,Grav_y,Grav_z,LinAcc_x,LinAcc_y,LinAcc_z,Control_Setpoint,Error_Angle");
  } else {
    Serial.println("Data file failed to open... :(");
    while (1);}

  // FOR TESTING PURPOSES!
  control_obj.initialThrowDirection.x() = 1.0;
  control_obj.initialThrowDirection.y() = 0.0;
  control_obj.initialThrowDirection.z() = 0.0;
  
}

void loop() {
  control_obj.updateState();
  //Serial.println(control_obj.curTime - millis());
  control_obj.printErrorTerm();
  //if (control_obj.timeToLog()) {
      //Serial.println("LOGGING!");
      //control_obj.logState();
      //control_obj.serialData();
      //control_obj.printErrorTerm();
 // }
}
