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
// #include <Adafruit_GFX.h>


#define sensor0 A0
#define sensor1 A1
#define sensor2 A2
#define sensor5 A5

#define SERVO_DOWN 750
#define SERVO_UP 2600 // 1750

void handleMotionComplete(); 

// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
volatile int baseSpeed = 10;
unsigned long previousTime = 0;

IRDecoder decoder(IR_DETECTOR_PIN);
Chassis chassis(7,1440,13.2);
Rangefinder rangefinder(2, 3);
Servo32U4Pin5 servo;

// A helper function for debugging
#define LED_PIN 13
void setLED(bool value)
{
  digitalWrite(LED_PIN, value);
}

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR, ROBOT_LINING, ROBOT_LOST_LINE, ROBOT_BAGGING};
ROBOT_STATE robotState = ROBOT_IDLE;

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
  chassis.setMotorPIDcoeffs(1.5,0.03);   //Working 
  rangefinder.init();
  servo.attach();
  servo.setMinMaxMicroseconds(SERVO_DOWN, SERVO_UP);
  servo.writeMicroseconds(SERVO_DOWN);
  
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

  //Save error history in vector
  const int ERR_HISTORY_SIZE = 10;
  int errorHistory[ERR_HISTORY_SIZE];
  int errorHistoryIndex = 0;

//Function to handle the Line Following
void handleLineFollow(int speed){
  
  //uint16_t positionLine = qtr.readLineBlack(sensorValues);

  //Read line sensors
  int Analog3 = analogRead(LEFT_LINE_SENSE); //A3 --------------------USING THIS ONE RIGHT sensor
  int rightLine = analogRead(RIGHT_LINE_SENSE); //A4

  int Analog0 = analogRead(sensor0); // 
  int Analog1 = analogRead(sensor1) ; // 
  int Analog2 = analogRead(sensor2); //  -------------------------USING THIS ONE  LEFT sensor
  int Analog5 = analogRead(sensor5); //
  //Define error between sensors
  int error = Analog2 - Analog3;

  errorHistory[errorHistoryIndex] = error;
  errorHistoryIndex = (errorHistoryIndex + 1) % ERR_HISTORY_SIZE;

  //Calculate average error from history
  float avgError = 0.0;
  long sum = 0;
  for(int i=0; i<ERR_HISTORY_SIZE; i++){
    sum += errorHistory[i];
  }

  avgError = (float)sum / ERR_HISTORY_SIZE;
  
  //Error
  float Kp = 0.6;

  //SHAR TURNS CONTROL
  if(abs(error) > 5*avgError){
    Kp *= 1.1;
  }
  if(abs(error) > 6*avgError){
    Kp *= 1.2;
  }
  if(abs(error) > 7*avgError){
    Kp *= 1.3;
  }

  // Additional gain for sharp turns (adaptive control)
  // const int MAX_EXPECTED_ERROR = 1000;
  // float threshold = 0.7 * MAX_EXPECTED_ERROR;
  // if(abs(error) > threshold) {
  //   Kp *= 1.3;    
  // }

  // //Reset integral on direction change
  // if(error*prevError < 0){
  //   integral = 0;
  // }

  //Calculate turn effor with Kp and Kd
  //float turnEffort = KGain*error + Ki*integral + Kd*derivative; //Error * K_p
  float turnEffort = Kp * error;
  //Update previous error
  //prevError = error;

  chassis.setTwist(speed, turnEffort);

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
      Serial.print(Analog3);
      Serial.print("\t");
      Serial.print("A4: ");
      Serial.print(rightLine);
      Serial.print("\t");
      Serial.print("A5: ");
      Serial.print(Analog5);
      Serial.print("\t");
      Serial.print("Error: ");
      Serial.print(error);
      //Serial.print("\t");
      // Serial.print("tunrnEffort: ");
      // Serial.println(turnEffort);
      //chassis.printEncoderCounts();
      Serial.println("\t");

                    //OUTER SENSORS DATA
    //        LIGHT                   Dark
    //LEFT ~854   RIGHT ~870    LEFT ~117    RIGHT ~120
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
      if(keyPress == NUM_1){
        robotState = ROBOT_BAGGING;
        Serial.print("Start ROBOT BAGGING");
      }
      break;
    default:
      break;
  }
}


void loop()
{
    int16_t keyPress = decoder.getKeyCode();
    if (keyPress >= 0)
        handleKeyPress(keyPress);

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
        handleLineFollow(baseSpeed);
        break;

    case ROBOT_LOST_LINE:
        findLine(baseSpeed);

        break;

    case ROBOT_BAGGING:
      float distance = rangefinder.getDistance();
      delay(50);
      if (distance < 10.0)
      {
          Serial.print("Picking up");
          servo.writeMicroseconds(SERVO_UP);
          
          // for (int slow = SERVO_DOWN; slow <= SERVO_UP; slow += 50)
          // {
          //   servo.writeMicroseconds(slow);
            
          // }
      }
      else {
        servo.writeMicroseconds(SERVO_DOWN);
      }
      Serial.print(millis());
      Serial.print('\t');
      Serial.print(distance);
      Serial.print(" cm\n");
      break;

    // case ROBOT_IDLE:
    //     break;

    default:
        break;
    }
}
