/*
 * Module 3 -- Move it! + Line Following
 */ 

//Cesar Corral Bryan Amato

#include <Arduino.h>
#include <wpi-32u4-lib.h>
#include <QTRSensors.h>
#include <IRdecoder.h>
#include <ir_codes.h>
#include <Chassis.h>

#define sensor0 A0
#define sensor1 A1
#define sensor2 A2
#define sensor5 A5

void handleMotionComplete(); 

// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
volatile int baseSpeed = 20;
unsigned long previousTime = 0;

IRDecoder decoder(IR_DETECTOR_PIN);
Chassis chassis(7,1440,13.2);

QTRSensors qtr;
const uint8_t SensorCount = 2;    // number of sensors used
uint16_t sensorValues[SensorCount];
void QTR_init();

// A helper function for debugging
#define LED_PIN 13
void setLED(bool value)
{
  digitalWrite(LED_PIN, value);
}

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR, ROBOT_LINING, ROBOT_LOST_LINE};
ROBOT_STATE robotState = ROBOT_IDLE;

void QTR_init(){
  Serial.print("QTR initializing...");
  qtr.setTypeRC(); // Reflectance mode
  qtr.setSensorPins( (const uint8_t[]){A3,A4}, SensorCount);
  qtr.setEmitterPin(A3);

  for(int i=0; i< 400 ; i++){
    qtr.calibrate();
  }

  Serial.print("Calibrated ");
}

// idle() stops the motors
void idle(void)
{
  Serial.println("idle() state");
  setLED(LOW);

  chassis.idle();

  //set state to idle
  robotState = ROBOT_IDLE;
}

void setup() 
{
  //Init Comms with console
  Serial.begin(115200);

  //Init motors
  chassis.init();
  chassis.setMotorPIDcoeffs(1.1,0.05);
  //chassis.setMotorEfforts(63, 50);
  
  Serial.print("Setup");


  //Setup the Line Sensors
  pinMode(sensor0, INPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(LEFT_LINE_SENSE, INPUT);  //A3
  pinMode(RIGHT_LINE_SENSE, INPUT); //A4
  pinMode(sensor5, INPUT);
  idle();

  // Initializes the IR decoder
  decoder.init();

  Serial.println("/setup()");
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
  
  uint16_t positionLine = qtr.readLineBlack(sensorValues);

  //Read line sensors
  int leftLine = analogRead(LEFT_LINE_SENSE); //A3
  int rightLine = analogRead(RIGHT_LINE_SENSE); //A4

  int Analog0 = analogRead(sensor0); // 
  int Analog1 = analogRead(sensor1) ; // 
  int Analog2 = analogRead(sensor2); // 
  int Analog5 = analogRead(sensor5); //
  //Define error between sensors
  int error = leftLine - rightLine;
  
  //Error
   float Kp = 0.01;
  float KGain = Kp;

  //Integral
  float Ki = 0.007;
  float integral = 0;
  integral += error;

  //Derivative
  const float Kd = 0.05;
  int prevError = 0;
  int derivative = error - prevError;

  //NonLinear Error gain for very sharp turns
  if(abs(error) > 150) {
    KGain = Kp * 3.0;
  }

  //Reset integral on direction change
  if(error*prevError < 0){
    integral = 0;
  }

  //Calculate turn effor with Kp and Kd
  float turnEffort = KGain*error + Ki*integral + Kd*derivative; //Error * K_p
  //float turnEffort = Kp * error;
  //Update previous error
  prevError = error;

  //On black line
  if(leftLine >= 300 && rightLine >= 300){
    chassis.setTwist(speed, turnEffort);
  }
  //On not black Line
  // if(leftLine < 200 && rightLine < 200){
  //   chassis.setTwist(speed, turnEffort);
  // }

  //Follow the black line
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
    
      Serial.print("A0: ");
      Serial.print(Analog0);
      Serial.print("\t");
      Serial.print("A1: ");
      Serial.print(Analog1);
      Serial.print("\t");
      Serial.print("A2: ");
      Serial.print(Analog2);
      Serial.print("\t");
      Serial.print("A3 ");
      Serial.print(leftLine);
      Serial.print("\t");
      Serial.print("A4: ");
      Serial.print(rightLine);
      Serial.print("\t");
      Serial.print("A5: ");
      Serial.print(Analog5);
      Serial.println("\t");
      // Serial.print("Error: ");
      // Serial.print(error);
      // Serial.print("\t");
      // Serial.print("tunrnEffort: ");
      // Serial.println(turnEffort);
      //chassis.printEncoderCounts();

                    //OUTER SENSORS DATA
    //        LIGHT                   Dark
    //LEFT ~854   RIGHT ~870    LEFT ~117    RIGHT ~120
}

// void findLine(int baseSpeed){
//   // Add code here to find the line
//   //Serial.println("Finding Line...");

//   int leftLine = analogRead(LEFT_LINE_SENSE);
//   int rightLine = analogRead(RIGHT_LINE_SENSE);

//   const int turn_speed = 20;

//   chassis.setTwist(turn_speed, 100);
//   chassis.driveFor(10, turn_speed);

//   // Drive forward slowly until line is detected
//   if(leftLine > 800 || rightLine > 800){
    
//     //Line found
//     robotState = ROBOT_LINING;
//     Serial.println("Line Found!");

//   }
//   else {
//     //Line not found yet
//     chassis.setTwist(baseSpeed, 100);
  
//     Serial.println("Turning...");
//   }
// }


// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  switch(robotState)
  {
    case ROBOT_IDLE: 
      if(keyPress == UP_ARROW){
        drive(200,baseSpeed);
        Serial.print("Button UP ARROW");
      }
      if(keyPress == RIGHT_ARROW){
        turn(-90, 80);
        Serial.print("Button RIGHT ARROW");
      }
      if(keyPress == LEFT_ARROW){
        turn(90,80);
        Serial.print("Button LEFT ARROW");
      }
      if(keyPress == DOWN_ARROW){
        drive(-20.0,20.0);
        Serial.print("Button DOWN ARROW");
      }
      if(keyPress == SETUP_BTN){
        beginLineFollowing();
      }
      break;
    default:
      break;
  }
}


void loop()
{
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);

  //Speed control
  if(keyPress == ENTER_SAVE){
    idle();
    Serial.print("Idle key pressed");
  }
  if(keyPress == VOLplus){
    baseSpeed += 5;
  }
  if(keyPress == VOLminus){
    baseSpeed -= 5;
  }

  // A basic state machine
  switch(robotState)
  {
    case ROBOT_DRIVE_FOR: 
      //Printing Speed of Wheels
      chassis.printSpeeds();
     //chassis.printEncoderCounts();
      if(keyPress == ENTER_SAVE){
        idle();
        Serial.print("Idle key pressed");
      }
      
      if(chassis.checkMotionComplete()) handleMotionComplete(); 
      break;
    
    case ROBOT_LINING:
      handleLineFollow(baseSpeed);
      break;
    
    // case ROBOT_LOST_LINE:
    //   findLine(baseSpeed);
       
    //   break;

    default:
      robotState = ROBOT_IDLE;
      break;
  }
}
