/*
 * Module 3 -- Move it! + Line Following
 */ 

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

#define SERVO_DOWN 750
#define SERVO_UP 1750 // 1750


// Fucntion prototypes
void QTR_init(void);
void handleMotionComplete(); 
bool detectPackage();
void pickupPackage();
void alignToIntersection();
void handleIntersection();
void deliverTo(String location, int intersection_count, bool start, bool packet_detected);
bool checkIntersectionEvent(uint16_t darkThreshold);
void PID_control();



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
bool start = false;
 

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
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR, ROBOT_LINING, ROBOT_LOST_LINE, ROBOT_BAGGING, ROBOT_DEBUG, ROBOT_DELIVERY, ROBOT_PICKUP, ROBOT_DROP};
ROBOT_STATE robotState = ROBOT_IDLE;


void deliverTo(String location, int intersection_count, bool start, bool packet_detected) {
  // if(location == "warehouse" &&  start){
  //   chassis.turnFor(180, 30, true);
  //   if(checkIntersectionEvent(darkThreshold)){
  //     handleIntersection();
  //   }
  //   start = false;
  // } else {  // DONT TURN 180
  //   if(checkIntersectionEvent(darkThreshold)){
  //     handleIntersection();
  //   }
  // }

  // ALL LOCATIONS
  if(location == "sunlandpark" && intersection_count == 2 &&  packet_detected){
    Serial.print("Arrived to sunland park");
    robotState = ROBOT_PICKUP;
    intersection_count = 0;
    start = false;  
  }
    // GO back to warehouse
    // if(!start){
    //   chassis.turnFor(180, 30, true);
    //   if(intersection_count == 3){
    //     robotState = ROBOT_DROP;
    //     intersection_count = 0;
    //   } 
    //   start = true;
    // }
  
  // else chassis.idle();

  
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

void pickupPackage(){
  Serial.print("Picking up...");
  servo.writeMicroseconds(SERVO_DOWN);
  
  //Picking up slowly
  for (int slow = SERVO_DOWN; slow <= SERVO_UP; slow += 50){
    delay(50);
    servo.writeMicroseconds(slow); 
  }
  
}

void dropPackage(){
  Serial.print("Dropping packet...");
  
  //Dropping slowly
  for (int slow = SERVO_UP; slow >= SERVO_DOWN; slow -= 50){
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
  Serial.print("MotorSpeedA: ");
  Serial.print(motorSpeedA);
  Serial.print("\tMotorSpeedB: ");
  Serial.print(motorSpeedB);

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
  // Serial.print("  A5: ");
  // Serial.print(sensorValues[5]);
  Serial.print("  Position: ");
  Serial.print(qtr.readLineBlack(sensorValues));

  Serial.print(" Error: ");
  Serial.print(error);
  Serial.print(" Speed Change: ");
  Serial.println(motorSpeedChange);

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
      if(keyPress == NUM_0_10 ) robotState = ROBOT_DELIVERY;
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


void loop()
{
  int16_t keyPress = decoder.getKeyCode();
  String delivery_location = "";
  bool packet_detected = false;
  bool delivering = false; 

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

  case ROBOT_BAGGING:
    //Line Follow until package is detected
    PID_control();
    //handleLineFollow(baseSpeed);

    if (detectPackage() == true){
      Serial.print("Package detected!");
      chassis.driveFor(5, 10, true);
      robotState = ROBOT_PICKUP;

      //Change State to delivery mode
      //robotState = ROBOT_DELIVERY;

    }
    break;
  
  case ROBOT_DELIVERY:
    // State that indicates robot is ON delivery mode.
    Serial.print("Delivery Mode");

    //Line follow
    PID_control();

    // Detecting Intersections
    if(checkIntersectionEvent(darkThreshold)) intersection_count++;

    if(detectPackage()){
      packet_detected = true;
      Serial.println("Object Detected");
      //beep(true);
      //handle alternative route
    }



    //Function that detects traffic and obstacles along the way

    //Function that drops package at delivery location

    break;
  case ROBOT_PICKUP:
    // State to pick up package
    delivering = false;
    chassis.driveFor(5, 10, true);
    pickupPackage();
    chassis.turnFor(180, 30, true);
    robotState = ROBOT_DELIVERY;
    break;
  // case ROBOT_DEBUG:
  //   debug();
  //   break;
  // case ROBOT_IDLE:
  //   break;

  case ROBOT_WAIT:
      // Wait for user to select delivery location
    if(keyPress == NUM_1){
      delivery_location = "warehouse";
      deliverTo(delivery_location, intersection_count, start, packet_detected);
      Serial.println("Delivery Location set to Warehouse");
    }
    if(keyPress == NUM_3){
      delivery_location = "sunlandpark";
      deliverTo(delivery_location, intersection_count, start, packet_detected);
      Serial.println("Delivery Location set to Sunland Park");
    }

    robotState = ROBOT_DELIVERY;
    // else {
    //   // deliverTo("wait", intersection_count, start, packet_detected);
    //   Serial.println("Waiting for delivery location selection...");
    // }
    // else if(keyPress == NUM_2){
    //     deliveryLocation = LOCATION_2;
    //     Serial.println("Delivery Location set to Location 2");
    // }
    // else if(keyPress == NUM_3){
    //     deliveryLocation = LOCATION_3;
    //     Serial.println("Delivery Location set to Location 3");
    // }

   break;

  default:
    robotState = ROBOT_IDLE;
    break;
  }
}
