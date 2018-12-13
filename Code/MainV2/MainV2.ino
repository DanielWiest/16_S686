#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <EEPROM.h>
#include <PWMServo.h>

#define LOGGING_FREQUENCY 100.0
#define MAX_LOG_NUMBER 850

/* TODO: 
 *    ADD Throw detection and find the initial velocity direction!
 *    SPEED UP the SD writing!
 *    ADD PID Control???
 *    TUNE Servo throw!
 *    TEST servo movement direction in opposing positions!
 */

const int chipSelect = BUILTIN_SDCARD;

PWMServo myservo;  // create servo object to control a servo
File dataFile;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

adafruit_bno055_offsets_t calibrationData;

String rootFile = "datLOG";
int fileIndex = 0;

imu::Vector<3> upwardsVector(0.0,0.0,1.0);
imu::Vector<3> forwardsVector(1.0,0.0,0.0);
imu::Vector<3> gravityVec(0.0,0.0,-1.0);

class ControlObject { //contains all the needed info on a planet
public:
    imu::Vector<3> euler,gravity,lin_acc,initialThrowDirection; // initDir must be a unit vector!
    imu::Quaternion quat;
    double throwEulerHeading; // SET ON DEFINITION OF initialThrowDirection! (only calculate once) 
    double errorAngle;
    double controlSetpoint = 0.0;
    unsigned long curTime = millis();
    int numberLogs = 0;
    double updateTime = (1.0/LOGGING_FREQUENCY)*1000.0;
    const int rotation_direction = 1;

    double phaseError;

    //TODO: Add PID Parameters
    const double kPID = 2.0;
    const double dPID = 0.0; // Currently Unused
    const double iPID = 0.0; // Currently Unused
    
    void printErrorTerm();
    void logState();
    void updateState();
    void serialData();
    void gravityAngles();
    void PIDCalcAndSend();
    void calcEulerHeadingOffset();
    bool timeToLog();
    double calculateControlOutput();
      
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

double ControlObject::calculateControlOutput() {
  return ((this->phaseError)*(this->rotation_direction)*( (this->errorAngle)*(this->kPID) ));
}

void ControlObject::updateState() {
  // Update internal class variables
  this->euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // NOT BEING USED
  this->quat = bno.getQuat();
  this->gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY); // NOT BEING USED
  this->lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // NOT BEING USED

  imu::Vector<3> rot_state_vert = this->quat.rotateVector(upwardsVector);
  imu::Vector<3> rot_state_horiz = this->quat.rotateVector(forwardsVector);

  // Calculate error in tilt perpendicular to the direction of original throw
  imu::Vector<3> projRotOntoPlane = (rot_state_vert - (this->initialThrowDirection)*(rot_state_vert.dot(this->initialThrowDirection)));
  imu::Vector<3> projGravOntoPlane = (gravityVec - this->initialThrowDirection*(gravityVec.dot(this->initialThrowDirection)));
  double magAB = projRotOntoPlane.magnitude()*projGravOntoPlane.magnitude();
  double thetaFromCos = acos( (projRotOntoPlane.dot(projGravOntoPlane))/(magAB));
  //       sin(theta) = ((normal X a) dot b)/(mag a * mag b)
  double sinTheta = (((this->initialThrowDirection.cross(projGravOntoPlane)).dot(projRotOntoPlane))/magAB);
  double tiltAngle;
  
  if (sinTheta >= 0.0) {
      tiltAngle = (thetaFromCos - PI);
  } else {
      tiltAngle = (PI - thetaFromCos);
  }
  //Serial.println(tiltError);
  // –––––––––––––––––––––––––––––––––––––––––––––––––––––––

  // Calculate error in heading so that corrections can be made in the correct phase
  imu::Vector<3> projHeadingOntoXY = (rot_state_horiz - (upwardsVector)*(rot_state_horiz.dot(upwardsVector)));
  double magHeading = projHeadingOntoXY.magnitude();
  double thetaFromCosHeading = acos( (projHeadingOntoXY.dot(forwardsVector))/(magHeading));
  double sinThetaHeading = (((upwardsVector.cross(forwardsVector)).dot(projHeadingOntoXY))/magHeading);
  double headingAngle;
  if (sinThetaHeading >= 0.0) {
      headingAngle = (thetaFromCosHeading);
  } else {
      headingAngle = (2.0*PI - thetaFromCosHeading);
  }
  double directionalScalar = cos( headingAngle - this->throwEulerHeading );

  this->phaseError = directionalScalar;
  
  //Serial.println(directionalScalar);
  // –––––––––––––––––––––––––––––––––––––––––––––––––––––––

  //this->errorAngle = ( this->controlSetpoint - (tiltError*directionalScalar*(this->rotation_direction)) ); //Push the correct result to class
  this->errorAngle = ( this->controlSetpoint - tiltAngle ); //Push the correct result to class
  
}

void ControlObject::printErrorTerm() {
  Serial.println(String(calculateControlOutput()));
}

void ControlObject::calcEulerHeadingOffset() {
  imu::Vector<3> xyPlane(0.0,0.0,1.0);
  imu::Vector<3> throwProjOntoXY = (this->initialThrowDirection - (xyPlane)*(this->initialThrowDirection.dot(xyPlane)));

  double magAB2 = throwProjOntoXY.magnitude();
  double thetaFromCos2 = acos( (throwProjOntoXY.dot(forwardsVector))/(magAB2));

  //       sin(theta) = ((normal X a) dot b)/(mag a * mag b) 
  double sinTheta2 = (((xyPlane.cross(forwardsVector)).dot(throwProjOntoXY))/magAB2);

  if (sinTheta2 >= 0.0) {
      this->throwEulerHeading = thetaFromCos2;
  } else {
      this->throwEulerHeading = (2.0*PI-thetaFromCos2);
  }
  
  // Serial.println(this->throwEulerHeading);

  // TODO: SET THE ROTATION DIRECTION TOO! (Somehow)
  
}

void ControlObject::logState() {
  this->curTime = millis();
  
  String timeData = String(this->curTime);
  String eulerData = String(this->euler.toString());
  String gravityData = String(this->gravity.toString());
  String linAccData = String(this->lin_acc.toString());
  String controlSetpointData = String(this->controlSetpoint);
  String errorValueData = String(this->errorAngle);

  String totalResult = String(timeData+","+eulerData+","+gravityData+","+linAccData+","+controlSetpointData+","+errorValueData);
  dataFile.println(totalResult);
  dataFile.flush();
  this->numberLogs = this->numberLogs + 1;
    
}

bool ControlObject::timeToLog() {
  return ( (this->numberLogs < MAX_LOG_NUMBER) && ((millis()-(this->curTime)) > updateTime) );
}

void ControlObject::PIDCalcAndSend() {
  int servoControl = map( this->calculateControlOutput(), -PI, PI, 0, 180);
  int writeToServo = constrain(servoControl, 0, 180);
  myservo.write(writeToServo);
}

void loadSensorCalib() {
  int eeAddress = 0;
  long bnoID;

  EEPROM.get(eeAddress, bnoID);
  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data loaded into BNO055");
  }
}

void printSystemInfo() {

  /* IT DOESNT LIKE THIS:
  Main:208: warning: 'self_test_result' is used uninitialized in this function 
  Main:208: warning: 'system_error' is used uninitialized in this function 
  */
  
  uint8_t *system_status;
  uint8_t *self_test_result;
  uint8_t *system_error;

  bno.getSystemStatus(system_status, self_test_result, system_error);
  Serial.println("ACC SELF TESTS TO FOLLOW:");
  Serial.println(*system_status,HEX);
  Serial.println(*self_test_result,BIN);
  Serial.println(*system_status,HEX);
}


ControlObject control_obj;


//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––


void setup() {
  pinMode(LED_BUILTIN , OUTPUT);

  Serial.begin(115200);
  if(!bno.begin()){
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);}
    
  loadSensorCalib();

  bno.setExtCrystalUse(true);
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_M4G); // Because the gyroscope saturates at just over 5 RPS (2000 deg/s)

  printSystemInfo();
  
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);}} 
  else {
    Serial.println("Card initialized!");
  }

  while ( SD.exists( (rootFile+String(fileIndex)+String(".csv")).c_str() ) ) {
    fileIndex += 1;}
  
  Serial.println((rootFile+String(fileIndex)+String(".csv")).c_str());
  
  
  dataFile = SD.open( (rootFile+String(fileIndex)+String(".csv")).c_str() , O_WRITE | O_CREAT);
  
  if (dataFile) {
    dataFile.println("Time,Euler_yaw,Euler_roll,Euler_pitch,Grav_x,Grav_y,Grav_z,LinAcc_x,LinAcc_y,LinAcc_z,Control_Setpoint,Error_Angle");
  } else {
    Serial.println("Data file failed to open... :(");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
     }
    }

  // FOR TESTING PURPOSES!
  control_obj.initialThrowDirection.x() = 1.0;
  control_obj.initialThrowDirection.y() = 0.0;
  control_obj.initialThrowDirection.z() = 0.0;

  // FOR TESTING PURPOSES!
  control_obj.calcEulerHeadingOffset();

   myservo.attach(30, 1000, 2000);

  digitalWrite(LED_BUILTIN, HIGH);
  
}

void loop() {
  control_obj.updateState();
  //Serial.println(control_obj.curTime - millis());
  control_obj.printErrorTerm();
  control_obj.PIDCalcAndSend();
  
  if (control_obj.timeToLog()) {
      //Serial.println("LOGGING!");
      control_obj.logState();
      //control_obj.serialData();
  }
}
