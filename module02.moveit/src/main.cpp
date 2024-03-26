/*
 * Module 2 -- Move it!
 */ 

//TODO, Add your group information to the top of your code.
//Cesar Corral Bryan Amato

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

void handleMotionComplete(); 

// TODO, Section, 4.2: Add line to include Chassis.h
#include <Chassis.h>

// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

// TODO, Section 4.2: Declare the chassis object (with default values)
//Chassis chassis(7,1440,14.7);

// TODO, Section 6.2: Adjust parameters to better match actual motion
Chassis chassis(7,1440,13.2);

// A helper function for debugging
#define LED_PIN 13
void setLED(bool value)
{
  Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR};
ROBOT_STATE robotState = ROBOT_IDLE;

// idle() stops the motors
void idle(void)
{
  Serial.println("idle() state");
  setLED(LOW);

  // TODO, Section 4.2: Uncomment call to chassis.idle() to stop the motors
  chassis.idle();

  //set state to idle
  robotState = ROBOT_IDLE;
  // Serial.println("idle()");
}

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);

  // TODO, Section 4.2: Initialize the chassis (which also initializes the motors)
  chassis.init();
  // TODO, Section 5.1: Adjust the PID coefficients
  chassis.setMotorPIDcoeffs(7,1);
  //Setup the Line Sensors
  pinMode(LEFT_LINE_SENSE, INPUT);  //A3
  pinMode(RIGHT_LINE_SENSE, INPUT); //A4
  idle();

  // Initializes the IR decoder
  decoder.init();

  Serial.println("/setup()");
}

// A helper command to drive a set distance
// At the start, it will take no arguments and we'll hardcode a motion
// TODO, Section 6.1 (but not before!): Edit the function definition to accept a distance and speed
void drive(float distancecm, float speed)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;

  // TODO: In Section 4.2 and 5.1, add a call to chassis.setWheelSpeeds() to set the wheel speeds
  //chassis.setWheelSpeeds(30,30);

  // TODO: In Section 6.1, remove the call to setWheelSpeeds() and add a call to chassis.driveFor()
  chassis.driveFor(distancecm,speed,false);
}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;

  // TODO, Section 6.1: Make a call to chassis.turnFor()
  chassis.turnFor(ang, speed);
}

// TODO, Section 6.1: Declare function handleMotionComplete(), which calls idle()
void handleMotionComplete(){
  idle();
} 

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  //println("Key: " + String(keyPress));

  switch(robotState)
  {
    case ROBOT_IDLE:
      // TODO, Section 3.2: Handle up arrow button
      if(keyPress == UP_ARROW){
        drive(40.0,35.0);
        Serial.print("Button UP ARROW");
      }
      // TODO, Section 6.1: Handle remaining arrows
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
    
      break;
      
    default:
      break;
  }
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
  // Checks for a key press on the remote
  // TODO, Section 3.1: Temporarily edit to pass true to getKeyCode()
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);

  // A basic state machine
  switch(robotState)
  {
    case ROBOT_DRIVE_FOR: 
      //Printing Speed of Wheels
     // chassis.printSpeeds();
     chassis.printEncoderCounts();
     //Printing the Line Sensors values
     int leftLineSensorReading = analogRead(LEFT_LINE_SENSE);
     int rightLineSensorReading = analogRead(RIGHT_LINE_SENSE);
      
      Serial.print(leftLineSensorReading);
      Serial.print('\t'); //print a TAB character to make the output prettier
      Serial.println(rightLineSensorReading);
      if(keyPress == ENTER_SAVE){
        idle();
        Serial.print("Idle key pressed");
      }
      
      // TODO, Section 6.1: Uncomment to handle completed motion
      if(chassis.checkMotionComplete()) handleMotionComplete(); 
      break;

    default:
      robotState = ROBOT_IDLE;
      break;
  }
}
