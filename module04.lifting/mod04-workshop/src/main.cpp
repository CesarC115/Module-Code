
//Cesar Corral Bryan Amato

#include <Arduino.h>
#include <wpi-32u4-lib.h>
#include <IRdecoder.h>
#include <ir_codes.h>
#include <Chassis.h>
#include <math.h>
#include <Rangefinder.h>
#include <servo32u4.h>
#include <QTRSensors.h>
#include <ArduinoSTL.h>
//#include <test.cpp>
//#include <Romi32U4Buzzer.h>
// #include <Adafruit_GFX.h>


#define sensor0 A0
#define sensor1 A1
#define sensor2 A2
#define sensor5 A5
#define BUZZER_PIN 6

#define LED_PIN 1
#define LED_IR_CALIBRATE 0

#define SERVO_DOWN 500
#define SERVO_UP 1950 // 1750


// Fucntion prototypes
void QTR_init(void);
void handleMotionComplete(); 
bool detectPackage();
void pickupPackage();
void alignToIntersection();
void handleIntersection();
int deliverTo(String location, int intersection_count, bool start, bool packet_detected);
bool checkIntersectionEvent(uint16_t darkThreshold);
void PID_control();
void beep(bool);
void dropPackage(int floor_level);
void goWarehouse();

// Global Vars
const uint8_t IR_DETECTOR_PIN = 1;
volatile int baseSpeed = 10;
unsigned long previousTime = 0;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
uint16_t darkThreshold = 500;
volatile uint16_t leftLine;
volatile uint16_t rightLine; 
int intersection_count = 0;

String delivery_location;
bool packet_detected = false;
bool done_delivering = false; 
bool start = false;
int floor_level;
int deliveries = 0;
 

int lastError = 0;
float Kp = 0.07;
float Ki = 0.0;
float Kd = 0.06;

int P = 0;
int I = 0;
int D = 0;

const uint8_t maxspeedA = 20;
const uint8_t maxspeedB = 20;
const uint8_t basespeedA = 10;
const uint8_t basespeedB = 10;

//Save error history in vector
const int ERR_HISTORY_SIZE = 10;
int errorHistory[ERR_HISTORY_SIZE];
int errorHistoryIndex = 0;

// Main sensors init
IRDecoder decoder(IR_DETECTOR_PIN);
Chassis chassis(7,1440,13.2);
Rangefinder rangefinder(2, 3);
Servo32U4Pin5 servo;
QTRSensors qtr;

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR, ROBOT_LINING, ROBOT_LOST_LINE, ROBOT_WAREHOUSE, ROBOT_DEBUG, ROBOT_DELIVERY, ROBOT_PICKUP, ROBOT_DROP, ROBOT_WAIT};
ROBOT_STATE robotState = ROBOT_IDLE;


int deliverTo(String location, int intersection_count, bool done_delivering, bool packet_detected) {
  Serial.print("Location: ");Serial.print(location);Serial.print("\t");
  Serial.print("Intersection count: ");Serial.print(intersection_count); Serial.print("\t"); 
  Serial.print("Packet Detected: ");Serial.print(packet_detected); Serial.print("\t"); 
  Serial.print("Done Deliver: ");Serial.println(done_delivering);


  // ALL LOCATIONS
  if(location == "sunlandpark"){
    while(intersection_count != 4){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.driveFor(5, 10, true);
    dropPackage(floor_level);  
    chassis.turnFor(185, 30, true);
    intersection_count = 0;

    // Going back to starting point
    while(intersection_count != 2){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    goWarehouse();
    return deliveries += 1;
  }
  else if(location == "airport"){
    chassis.turnFor(90, 30, true);
    while(intersection_count != 1){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.turnFor(95, 30, true);
    while(intersection_count != 2){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.driveFor(5, 10, true);
    dropPackage(floor_level);  
    chassis.turnFor(185, 30, true);
    intersection_count = 0;

    // Going back to starting point
    while(intersection_count != 1){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.turnFor(-95, 30, true);
    while(intersection_count != 2){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.turnFor(-95, 30, true);
    goWarehouse();
    return deliveries += 1;   
  }
  else if(location == "utep"){
    chassis.turnFor(-95, 30, true);
    while(intersection_count != 1){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.driveFor(5, 10, true);
    dropPackage(floor_level);  
    chassis.turnFor(185, 30, true);

    //Going back to starting point
    intersection_count = 0;
    while(intersection_count != 1){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.turnFor(95, 30, true);  
    goWarehouse();
    return deliveries += 1;
  }
  else if(location == "executive"){
    while(intersection_count != 1){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.turnFor(-95, 30, true);
    chassis.driveFor(10, 10, true);
    dropPackage(floor_level);
    intersection_count = 0;

    // GO back
    chassis.turnFor(185, 30, true);
    while(intersection_count != 1){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.turnFor(95, 30, true);
    intersection_count = 0;

    while(intersection_count != 1){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    goWarehouse();
  }
  else if(location == "country-club"){
    while(intersection_count != 1){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.turnFor(95, 30, true);
    while(intersection_count != 2){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.turnFor(-95, 30, true);
    chassis.driveFor(10, 10, true);
    dropPackage(floor_level);
    chassis.turnFor(185, 30, true);
    intersection_count = 0;

    // GO BACK
    while(intersection_count != 1){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.turnFor(95, 30, true);
    while(intersection_count != 2){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
    chassis.turnFor(-95, 30, true);
    while(intersection_count != 3){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }

    intersection_count = 0;
    goWarehouse();
  }
  else if(location == "franklin"){
    while(intersection_count != 1){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
    }
    chassis.turnFor(95, 30, true);
    while(intersection_count != 2){
      PID_control();
      if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
      }
    }
      chassis.turnFor(95, 30, true);
      chassis.driveFor(10, 10, true);
      dropPackage(floor_level);
      chassis.turnFor(185, 30, true);
      intersection_count = 0;

      // Go Back
      while(intersection_count != 1){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(-95, 30, true);
      while(intersection_count != 2){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(-95, 30, true);
      while(intersection_count != 3){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      intersection_count = 0;
      goWarehouse();
    }
    else if(location == "outlets"){
      chassis.turnFor(95, 30, true);  
      while(intersection_count != 2){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
            Serial.println("Intersection Detected");
            intersection_count++;
        }
      }
      chassis.turnFor(-95, 30, true);
      while(intersection_count != 4){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      dropPackage(floor_level);
      intersection_count = 0;
      chassis.turnFor(185, 30, true);
      // GO back
      while(intersection_count != 2){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(95, 30, true);
      while(intersection_count != 4){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(-95,30,true);
      intersection_count = 0;
      goWarehouse();
    }
    else if(location == "lascruces"){
      chassis.turnFor(95, 30, true);  
      while(intersection_count != 2){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
            Serial.println("Intersection Detected");
            intersection_count++;
        }
      }
      chassis.turnFor(-95, 30, true);
      while(intersection_count != 3){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(95, 30, true);
      while(intersection_count != 4){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      dropPackage(floor_level);
      chassis.turnFor(185, 30, true);
      intersection_count = 0;

      // GO back
      while(intersection_count != 1){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(-95, 30, true);
      while(intersection_count != 2){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(95, 30, true);
      while(intersection_count != 4){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(-95, 30, true);
      intersection_count = 0;
      goWarehouse();
    }
    else if(location == "alamogordo"){
      chassis.turnFor(95, 30, true);
      while(intersection_count != 3){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      dropPackage(floor_level);
      chassis.turnFor(185, 30, true);
      intersection_count = 0;

      // GO back
      while(intersection_count != 3){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(-95, 30, true);
      intersection_count = 0;
      goWarehouse();
    }
    else if(location == "ftbliss"){
      chassis.turnFor(95, 30, true);
      while(intersection_count != 2){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(95, 30, true);
      while(intersection_count != 3){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      dropPackage(floor_level);
      chassis.turnFor(185, 30, true);
      intersection_count = 0;

      // GO back
      while(intersection_count != 1){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(-95, 30, true);
      while(intersection_count != 2){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
          Serial.println("Intersection Detected");
          intersection_count++;
        }
      }
      chassis.turnFor(-95,30,true);
      intersection_count = 0;
      goWarehouse();
    }

    

  return 0;
}

void beep(boolean input) {
  // Play a tone on the buzzer pin
  tone(BUZZER_PIN, 500, 1000); // 440 Hz for 200 ms
  if (input) {
    Serial.println("BEEP");
  }
}

bool checkIntersectionEvent(uint16_t darkThreshold){
  static bool prevIntersection = false;

  bool retVal = false;

  bool leftDetect = (sensorValues[0]) > darkThreshold ? true : false;
  bool rightDetect = (sensorValues[SensorCount-1]) > darkThreshold ? true : false;

  bool intersection = leftDetect && rightDetect;
  if(intersection && !prevIntersection) retVal = true;
  prevIntersection = intersection;

  return retVal;
}

bool detectPackage() {
  float distance  = 0.0;
  distance = rangefinder.getDistance();
  delay(50);
  if (distance < 10.0){
    return true;
  }
  else{
    return false;
  }
  Serial.print(distance);
  Serial.print(" cm\n");

}

void pickupPackage() {
  Serial.println("Picking up...");
  servo.writeMicroseconds(SERVO_DOWN);
  
  //Picking up slowly
  for (int slow = SERVO_DOWN; slow <= SERVO_UP; slow += 50){
    delay(50);
    servo.writeMicroseconds(slow); 
  }
  
}

void dropPackage(int floor_level) {
  Serial.print("Dropping packet...");

  int servo_adjust = SERVO_DOWN;

  if (floor_level == 3) {
    servo_adjust +=  800;
  } else if (floor_level == 2) {
    servo_adjust +=  500;
  } else {// floor level
    servo_adjust = SERVO_DOWN;
  }
  
  //Dropping slowly
  for (int slow = SERVO_UP; slow >= servo_adjust; slow -= 25){
    delay(50);
    servo.writeMicroseconds(slow); 
  }
  
}

void setLED(bool value)
{
  digitalWrite(LED_PIN, value);
}


// idle() stops the motors
void idle(void)
{
  Serial.println("idle() state");
  setLED(LOW);
  servo.writeMicroseconds(SERVO_DOWN);
  chassis.idle();

  //set state to idle
  robotState = ROBOT_IDLE;
}

void setup() 
{
  //Init Comms with console
  Serial.begin(115200);
  Serial.println("\n\n-------BEGINNING SETUP-------");

  //Init motors
  chassis.init();
  chassis.setMotorPIDcoeffs(7,2); 
  rangefinder.init();
  servo.attach();
  servo.setMinMaxMicroseconds(SERVO_DOWN, SERVO_UP);
  servo.writeMicroseconds(SERVO_DOWN);
  decoder.init();

  //Setup the Line Sensors
  pinMode(sensor0, INPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(LEFT_LINE_SENSE, INPUT);  //A3
  pinMode(RIGHT_LINE_SENSE, INPUT); //A4
  pinMode(sensor5, INPUT);
  idle();

}

void QTR_init(){
  Serial.println("\n\n-------BEGINNING CALIBRATION-------");
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, SensorCount);

  delay(500);

  
  pinMode(LED_IR_CALIBRATE, OUTPUT);
  digitalWrite(LED_IR_CALIBRATE, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  for (uint16_t i = 0; i < 400; i++){
    if(i % 40 == 0) chassis.setWheelSpeeds(10, 10);
    else if(i % 40 == 20) chassis.setWheelSpeeds (-10, -10);

    qtr.calibrate();
  }
  chassis.setWheelSpeeds(0, 0);

  digitalWrite(LED_IR_CALIBRATE, LOW);

  int sensorCurr = 0;
  int sensorMinMax = 0;
  int sensorMaxMin = 1000;

  for (uint8_t i = 0; i < SensorCount; i++){
    sensorCurr = qtr.calibrationOn.minimum[i];
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
    if(sensorCurr > sensorMinMax) sensorMinMax = sensorCurr;
  }
  Serial.println();

    // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++){
    sensorCurr = qtr.calibrationOn.maximum[i];
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
    if(sensorCurr < sensorMaxMin) sensorMaxMin = sensorCurr;
  }
  Serial.println();
  Serial.println();
  delay(500);

  darkThreshold = sensorMaxMin - sensorMinMax;
  
  Serial.print("Maximum Min Val: "); Serial.println(sensorMinMax);
  Serial.print("Minimum Max Val: "); Serial.println(sensorMaxMin);
  Serial.print("Sensor darkThreshold: "); Serial.println(darkThreshold);
  
  //set state to idle
  robotState = ROBOT_IDLE;
  //keyPress = 60;  //Setting keyPress to unhandled value to avoid bouncing buttons
  Serial.println("\n-----CALIBRATION COMPLETE-------");

}

void forward_movement(int speedA, int speedB) {
  if (speedA < 0){
    speedA = 0 - speedA;
    chassis.setWheelSpeeds(-speedA,-speedB);
  }
  else{
    chassis.setWheelSpeeds(speedA,speedB);
  }
  if (speedB < 0){
    speedB = 0 - speedB;
    chassis.setWheelSpeeds(-speedA,-speedB);
  }
  else{
    chassis.setWheelSpeeds(speedA,speedB);
  }

}

void PID_control(){  
  //beep(true);
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  int error = (2000 - positionLine); 

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error;

  int motorSpeedChange = P * Kp + I * Ki + D * Kd; //Need values between [0,255]

  int motorSpeedA = baseSpeed - (motorSpeedChange/10);
  int motorSpeedB = baseSpeed + (motorSpeedChange/10);

  if(motorSpeedA > maxspeedA){
    motorSpeedA = maxspeedA;
  }
  if(motorSpeedB > maxspeedB){
    motorSpeedB = maxspeedB;
  }
  if(motorSpeedA < 0){
    motorSpeedA = 0;
  }
  if(motorSpeedB < 0){
    motorSpeedB = 0;
  }
  // Serial.print("MotorSpeedA: ");
  // Serial.print(motorSpeedA);
  // Serial.print("\tMotorSpeedB: ");
  // Serial.print(motorSpeedB);

  // Serial.print("A0: ");
  // Serial.print(sensorValues[0]);
  // Serial.print("  A1: ");
  // Serial.print(sensorValues[1]);
  // Serial.print("  A2: ");
  // Serial.print(sensorValues[2]);
  // Serial.print("  A3: ");
  // Serial.print(sensorValues[3]);
  // Serial.print("  A4: ");
  // Serial.print(sensorValues[4]);
  // // Serial.print("  A5: ");
  // // Serial.print(sensorValues[5]);
  // Serial.print("  Position: ");
  // Serial.print(qtr.readLineBlack(sensorValues));

  // Serial.print(" Error: ");
  // Serial.print(error);
  // Serial.print(" Speed Change: ");
  // Serial.println(motorSpeedChange);

  forward_movement(motorSpeedA, motorSpeedB); 

}

void debug(){

  /*DIRECT VALUES PULLED FROM SENSOR*/
  // Serial.print("A0: ");
  // Serial.print(analogRead(A0));
  // Serial.print("  A1: ");
  // Serial.print(analogRead(A1));
  // Serial.print("  A2: ");
  // Serial.print(analogRead(A2));
  // Serial.print("  A3: ");
  // Serial.print(analogRead(A3));
  // Serial.print("  A4: ");
  // Serial.print(analogRead(A4));
  // Serial.print("  A5: ");
  // Serial.println(analogRead(A5));


  /*VALUES PULLED FROM QTR LIBRARY FUNCTION FOR ANALOG READ*/
  Serial.print("A0: ");
  Serial.print(sensorValues[0]);
  Serial.print("  A1: ");
  Serial.print(sensorValues[1]);
  Serial.print("  A2: ");
  Serial.print(sensorValues[2]);
  Serial.print("  A3: ");
  Serial.print(sensorValues[3]);
  Serial.print("  A4: ");
  Serial.print(sensorValues[4]);
  Serial.print("  A5: ");
  Serial.print(sensorValues[5]);
  Serial.print("Position: ");
  Serial.println(qtr.readLineBlack(sensorValues));

  

  robotState = ROBOT_DEBUG;
}

void drive(float distancecm, float speed)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;
  chassis.driveFor(distancecm,speed,false);
}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;
  chassis.turnFor(ang, speed);
}

//Function to handle the motion complete event
void handleMotionComplete(){
  idle();

  // When motion is complete decide where to go
} 

//Function that handles Line Following
void beginLineFollowing(){
  //Setup
  Serial.println("beginLineFollowing()");
  setLED(HIGH);
  robotState = ROBOT_LINING;
}

//Function to handle the Line Following
void handleLineFollow(int speed){

  unsigned int whiteMin = 120;
  unsigned int whiteMax = 220;
  unsigned int blackMin = 500;   //Sensor min was 680 but depends on proximity to black tape so overshooting
  unsigned int blackMax = 900;   //Sensor max was ~850 but depends on proimity so overshooting


  // Read the sensor values inside the loop
  leftLine = analogRead(LEFT_LINE_SENSE);
  rightLine = analogRead(RIGHT_LINE_SENSE);

  //Define error between sensors
  //int error = leftLine - rightLine;
  //int turnEffort = error * 1.1; //Error * K_p

  Serial.print("LEFT S:\t");
  //Serial.print(analogRead(LEFT_LINE_SENSE));
  Serial.print(leftLine);
  Serial.print("\tRIGHT S:\t");
  Serial.print(rightLine);
  //Serial.println(analogRead(RIGHT_LINE_SENSE));
  
  while ((rightLine > whiteMin && rightLine < whiteMax) && (leftLine > whiteMin && leftLine < whiteMax)) {
    // Straight movement when both sensors are on white
    chassis.setWheelSpeeds(30, 30);

    if (leftLine > blackMin && leftLine < blackMax) {
        // Adjust speed and turn left when the left sensor detects black
        chassis.setTwist(speed, 10);   // Turn Left
        // chassis.turnFor(-30, 40);
    } else if (rightLine > whiteMin && rightLine < whiteMax) {
        // Adjust speed and turn right when the right sensor detects white
        chassis.setTwist(speed, -10);   // Turn Right
    }

    // Update sensor readings
    leftLine = analogRead(LEFT_LINE_SENSE);
    rightLine = analogRead(RIGHT_LINE_SENSE);
  }

  // If either sensor is on black, adjust the direction accordingly
  if (leftLine < whiteMin || leftLine > whiteMax) {
      chassis.setTwist(speed, 10); // Turn Left
      // chassis.turnFor(30, 200);
  } else if (rightLine < whiteMin || rightLine >= whiteMax) {
      chassis.setTwist(speed, -10);   // Turn Right
  }

 
  //Follow the black line -------------------------------------- DO NOT DELETE
  // if(leftLine >= 800 && rightLine >= 800){
  //   chassis.driveFor(1,speed);
  // }
  // else { //correct turn direction'
  //   //ADJUSTING TURNING EFFORT BASED ON ERROR
  //   // if(error >= 50) turnEffort = error * 0.2;
  //   // if(error >= 100) turnEffort = error * 0.5;
  //   // if(error >= 200) turnEffort = error * 1.5;
  //   chassis.setTwist(speed, -turnEffort);
  // }
  // //If line is lost continue for 2 seconds
  // if(leftLine < 400 && rightLine < 400){

  //   static unsigned long lostLineTimer = 0;

  //   if(lostLineTimer == 0){
  //     lostLineTimer = millis(); //Start timer
  //     Serial.println("Start Timer");
  //   }

  //   //after 2s find line
  //   if(millis() - lostLineTimer > 4000){   //When timer exceeds 2 seconds find line again

  //     Serial.println("LOST LINE!");
  //     lostLineTimer = 0;    //Reset timer
  //     //robotState = ROBOT_LOST_LINE;  
      

  //   }
    
        // -----------------PRINT SENSOR VALUES------------------------
      // Serial.print("A0: ");
      // Serial.print(Analog0);
      // Serial.print("\t");
      // Serial.print("A1: ");
      // Serial.print(Analog1);
      // Serial.print("\t");
      // Serial.print("A2: ");
      // Serial.print(Analog2);
      // Serial.print("\t");
      // Serial.print("A3 ");
      // Serial.print(Analog3);
      // Serial.print("\t");
      // Serial.print("A4: ");
      // Serial.print(rightLine);
      // Serial.print("\t");
      // Serial.print("A5: ");
      // Serial.print(Analog5);
      // Serial.print("\t");
      // Serial.print("Error: ");
      // Serial.print(error);
      // //Serial.print("\t");
      // // Serial.print("tunrnEffort: ");
      // // Serial.println(turnEffort);
      // //chassis.printEncoderCounts();
      // Serial.println("\t");
}
void findLine(int baseSpeed){
  Serial.println("Finding Line...");

  int leftLine = analogRead(LEFT_LINE_SENSE);
  int rightLine = analogRead(RIGHT_LINE_SENSE);

  const int turn_speed = 20;

  chassis.setTwist(turn_speed, 100);
  chassis.driveFor(10, turn_speed);

  // Drive forward slowly until line is detected
  if(leftLine > 800 || rightLine > 800){
    
    //Line found
    robotState = ROBOT_LINING;
    Serial.println("Line Found!");

  }
  else {
    //Line not found yet
    chassis.setTwist(baseSpeed, 100);
  
    Serial.println("Turning...");
  }
}


// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress){
  Serial.println("Key: " + String(keyPress));
  switch(robotState)
  {
    case ROBOT_IDLE: 
      if(keyPress == UP_ARROW) drive(100, 30);
      if(keyPress == DOWN_ARROW) drive(-20, 30);
      if(keyPress == LEFT_ARROW) turn(90, 200);
      if(keyPress == RIGHT_ARROW) turn(-90, 200);
      if(keyPress == SETUP_BTN) beginLineFollowing();
      if(keyPress == REWIND) debug();
      if(keyPress == PLAY_PAUSE) QTR_init();
      if(keyPress == NUM_0_10 ) goWarehouse();
      if(keyPress == NUM_4) {
        servo.writeMicroseconds(SERVO_UP); 
        dropPackage(3);
      }
      if(keyPress == NUM_5) servo.writeMicroseconds(SERVO_DOWN); 
      if(keyPress == NUM_6) pickupPackage(); 
      // if(keyPress == NUM_1){
      //   robotState = ROBOT_BAGGING; Serial.print("Start ROBOT BAGGING");
      // }
      // if(keyPress == NUM_2){
      //   pickupPackage();
      // }
      // if (keyPress == NUM_3){
      //   for (int slow = SERVO_UP; slow >= SERVO_DOWN; slow -= 50){
      //     delayMicroseconds(50000);
      //     servo.writeMicroseconds(slow); 
      //   }
      // }
    break;
    default: 
      Serial.println("Unknown State");
      break;
  }
}

void handleIntersection(void){
  Serial.println("Intersection!");
  //beep(true);

  //drive forward by dead reckoning to center the robot
  drive(5, 10);

  robotState = ROBOT_IDLE;
}

void goWarehouse(){
  while(!packet_detected){
    PID_control();
    if(detectPackage()){
      packet_detected = true;
      Serial.println("Object Detected");
    }
  }
  chassis.idle();
  pickupPackage();
  chassis.turnFor(185, 40, true);

  // Go to starting point
  while(intersection_count != 2){
    PID_control();
    if(checkIntersectionEvent(darkThreshold)){
        Serial.println("Intersection Detected");
        intersection_count++;
    }
  }
  intersection_count = 0;
  robotState = ROBOT_DELIVERY;
}

void loop()
{
  int16_t keyPress = decoder.getKeyCode();
  String delivery_location1, delivery_location2, delivery_location3;

  if (keyPress >= 0){
   Serial.println("Key pressed:" + String(keyPress));
   handleKeyPress(keyPress);
  }
  // Speed control
  if (keyPress == ENTER_SAVE)
  {
    idle();
    Serial.print("Idle key pressed");
  }
  if (keyPress == VOLplus)
  {
      baseSpeed += 5;
  }
  if (keyPress == VOLminus)
  {
      baseSpeed -= 5;
  }

  // A basic state machine
  switch (robotState)
  {
  case ROBOT_DRIVE_FOR:
      // Printing Speed of Wheels
      chassis.printSpeeds();
      // chassis.printEncoderCounts();
      if (keyPress == ENTER_SAVE)
      {
        idle();
        Serial.print("Idle key pressed");
      }
      if (chassis.checkMotionComplete())
          handleMotionComplete();
      break;

  case ROBOT_LINING:
    PID_control();
    //Interserctions
    //handleLineFollow(baseSpeed);
    break;

  case ROBOT_LOST_LINE:
      findLine(baseSpeed);
      break;

  case ROBOT_WAREHOUSE:


    while(intersection_count != 1 && !packet_detected){
        PID_control();
        if(checkIntersectionEvent(darkThreshold)){
            Serial.println("Intersection Detected");
            intersection_count++;
        }
    }
    pickupPackage();


    // //Line Follow
    // PID_control();

    // // Detecting Intersections
    // if(checkIntersectionEvent(darkThreshold)){
    //   Serial.println("Intersection Detected");
    //   intersection_count++;
    // } 

    // if(detectPackage()){
    //   packet_detected = true;
    //   Serial.println("Object Detected");
    //   //beep(true);
    //   //handle alternative route
    // }

    // Serial.println("Going to warehouse");
    // if(intersection_count == 1 && packet_detected){
    //   beep(true);
    //   // chassis.driveFor(5, 10, true);
    //   pickupPackage();
    //   chassis.turnFor(185, 30, false);

    //   while(intersection_count != 3){
    //     PID_control();
    //     if(checkIntersectionEvent(darkThreshold)){
    //       Serial.println("Intersection Detected");
    //       intersection_count++;
    //     }
    //   }
    //   intersection_count = 0;
    //   robotState = ROBOT_DELIVERY;
    // }
    break;
  
  case ROBOT_DELIVERY:
    // State that indicates robot is ON delivery mode.
    Serial.println("Delivery Mode");

    //Line follow
    PID_control();

    // Detecting Intersections
    if(checkIntersectionEvent(darkThreshold)){
      Serial.println("Intersection Detected");
      intersection_count++;
    } 

    if(detectPackage()){
      packet_detected = true;
      Serial.println("Object Detected");
      //beep(true);
      //handle alternative route
    }


    delivery_location1 = "sunlandpark";
    delivery_location2 = "airport";
    delivery_location3 = "utep";
  
    if(deliveries == 0){
      deliverTo(delivery_location1, intersection_count, done_delivering, packet_detected);
    } 
    else if(deliveries == 1){
      deliverTo(delivery_location2, intersection_count, done_delivering, packet_detected);
    }
    else if (deliveries == 2){
      deliverTo(delivery_location3, intersection_count, done_delivering, packet_detected);
      Serial.print(" ALL Delivery Complete!");
    }
        

    break;
  case ROBOT_PICKUP:
    // State to pick up package
    intersection_count = 0;
    packet_detected = false;
    start = false; 
    done_delivering = false;

    chassis.driveFor(5, 10, true);
    pickupPackage();  
    chassis.turnFor(185, 30, true);
    robotState = ROBOT_DELIVERY;
    break;
  
  case ROBOT_DROP:
    // State to drop package
    intersection_count = 0;
    packet_detected = false;
    done_delivering = true;

    chassis.driveFor(5, 10, true);
    dropPackage(floor_level);
    chassis.turnFor(185, 40, true);
    robotState = ROBOT_DELIVERY;

    break;

  case ROBOT_WAIT:

    //Line follow
    chassis.idle();
    



    // // Reset flags
    // intersection_count = 0;
    // packet_detected = false;
    // done_delivering = true;

    

    // // Wait for user to select delivery location
    // if(keyPress == NUM_1){
    //   delivery_location = "warehouse";
    //   Serial.println("Delivery Location set to Warehouse");
    // }
    // else if(keyPress == NUM_2){
    //   delivery_location = "warehouse-utep";
    //   Serial.println("Delivery Location set to Warehouse to UTEP");
    // }
    // else if(keyPress == NUM_3){
    //   delivery_location = "sunlandpark";
    //   Serial.println("Delivery Location set to Sunland Park");
    // }
    // else Serial.println("Waiting for user input");

    // // Which level to deliver
    // if(keyPress == NUM_9){
    //   // Delivery altitude to floor 3
    //   floor_level = 3;
    //   done_delivering = false;
    //   robotState = ROBOT_DELIVERY;
    // }
    // if(keyPress == NUM_8){
    //   //Delivery altitude floor 2
    //   floor_level = 2;
    //   done_delivering = false;
    //   robotState = ROBOT_DELIVERY;
    // }
    // if(keyPress == NUM_7){
    //   // Deliver to floor level
    //   floor_level = 1;
    //   done_delivering = false;
    //   // Serial.print("Done Deliver WAIT: ");Serial.println(done_delivering);
    //   robotState = ROBOT_DELIVERY;
    // }

   break;

  default:
    robotState = ROBOT_IDLE;
    break;
  }
}
