#include <Arduino.h>
 #include "CytronMotorDriver.h"


const int INPUT_STATE_RADIO_ON = 1;
const int INPUT_STATE_JOYSTICK_ON = 2;
const int INPUT_STATE_OFF = 0;

int MOTORLEFT = 3;
int MOTORRIGHT = 9;

int MOTORLEFTDIR = 4;
int MOTORRIGHTDIR = 10;

int RCYAW = A5; //
int RCSPEED = A4; //

int JOYYWIN = A0; //
int JOYXWIN = A1; //

int INPUT_STATE = INPUT_STATE_OFF; //1 = RADIO, 0 = OFF, 2 = JOYSTICK
int INPUT_STATE_RADIO = 5;
int INPUT_STATE_JOY = 6;


const int PWM_LIMIT = 200;
const int DEAD_ZONE = 10;
const int YAW_SENSITIVITY = 90;

// Configure the motor driver.
CytronMD motor1(PWM_DIR, MOTORLEFT, MOTORLEFTDIR);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, MOTORRIGHT, MOTORRIGHTDIR); // PWM 2 = Pin 9, DIR 2 = Pin 10.


void setup()
{

  motor1.setSpeed(0);
  motor2.setSpeed(0);

  // Set pin modes on motor pins
  //pinMode(MOTORLEFT, OUTPUT); // Set Arduino pin 6 (MOTORLEFT) as output.
  //pinMode(MOTORRIGHT, OUTPUT); // Set Arduino pin 5 (MOTORRIGHT) as output.
  //analogWrite(MOTORLEFT, 0);//
  //analogWrite(MOTORRIGHT, 0);//

  pinMode(RCSPEED,INPUT);
  pinMode(RCYAW,INPUT);

  pinMode(INPUT_STATE_JOY,INPUT);
  pinMode(INPUT_STATE_RADIO,INPUT);

  Serial.begin(9600);
  Serial.print(F("setup"));

}

void LeftMotorSetSpeed(int speed)
{
  // Set motor pins to run the motor forward at the specified speed
  (void) speed;
  Serial.print(F("leftmotor:"));Serial.println(int(speed));
  //analogWrite(MOTORLEFT, speed);
  motor1.setSpeed(speed);

}


void RightMotorSetSpeed(int speed)
{
  // Set motor pins to run the motor forward at the specified speed
  (void) speed;
   Serial.print(F("rightmotor:"));Serial.println(int(speed));
  //analogWrite(MOTORRIGHT, speed);
  motor2.setSpeed(speed);

}



void loop()
{

  byte inputStateJoy = digitalRead(INPUT_STATE_JOY);
  byte inputStateRC = digitalRead(INPUT_STATE_RADIO);

  Serial.print(F("InputStateRAWJoy:"));Serial.println(int(inputStateJoy));
  Serial.print(F("InputStateRAWRC:"));Serial.println(int(inputStateRC));


  int inputState = INPUT_STATE_OFF;
  if (inputStateJoy==HIGH) {
    inputState = INPUT_STATE_JOYSTICK_ON;
  } else if (inputStateRC==HIGH) {
    inputState = INPUT_STATE_RADIO_ON;
  }

  Serial.println(":");
  Serial.print(F("InputState:"));Serial.println(int(inputState));

  int leftSpeed =0;
  int rightSpeed = 0;

  //Switch is off
  if (inputState==INPUT_STATE_OFF) {
    Serial.print(F("STOP ALL!:"));Serial.println(int(inputState));
  }


//Switched to RC
  else if (inputState==INPUT_STATE_RADIO_ON) {
    int speedInput = pulseIn(RCSPEED, HIGH, 25000);
    int yawInput = pulseIn(RCYAW, HIGH, 25000);
      Serial.print(F("yawInputR:"));Serial.println(int(yawInput));
      Serial.print(F("speedInputR:"));Serial.println(int(speedInput));

    if (speedInput==0 || yawInput==0) {
        leftSpeed = 0;
        rightSpeed = 0;
    } else {

    
      Serial.println(":");


      // map 'speed' to the range -PWM_LIMIT (backward), +PWM_LIMIT (forward)
      speedInput = map(speedInput, 900, 2000, -PWM_LIMIT, PWM_LIMIT);
      yawInput = map(yawInput, 900, 2000, -YAW_SENSITIVITY, YAW_SENSITIVITY);

      // Put in dead zones
      if (speedInput > -DEAD_ZONE && speedInput < DEAD_ZONE)
        speedInput = 0;
      if (yawInput > -DEAD_ZONE && yawInput < DEAD_ZONE)
        yawInput = 0;
      speedInput = speedInput *-1;

      int gamma = 3; //gain
      int speedExp =(speedInput^gamma)/(PWM_LIMIT^(gamma-1));
      int gammaYaw = 2; //gain
      int yawExp =(yawInput^gammaYaw)/(YAW_SENSITIVITY^(gammaYaw-1));

      leftSpeed = speedExp + yawExp;
      rightSpeed = speedExp - yawExp;

    }
  }

//Switched to JOYSTICK
  else if (inputState==INPUT_STATE_JOYSTICK_ON) {

    int speedInput = analogRead(JOYYWIN); // Forward/Reverse
    int yawInput = analogRead(JOYXWIN); // Left/Right turn
    Serial.print(F("yawInputJ:"));Serial.println(int(yawInput));
    Serial.print(F("speedInputJ:"));Serial.println(int(speedInput));
    Serial.println(":");


    // map 'speed' to the range -PWM_LIMIT (backward), +PWM_LIMIT (forward)
    //speedInput = 1023-speedInput;
    yawInput = 1023-yawInput;
    speedInput = map(speedInput, 0, 1023, -PWM_LIMIT, PWM_LIMIT);
    yawInput = map(yawInput, 0, 1023, -YAW_SENSITIVITY, YAW_SENSITIVITY);

    Serial.print(F("speedInputAfterMap:"));Serial.println(int(speedInput));

    // Put in dead zones
    if (speedInput > -DEAD_ZONE && speedInput < DEAD_ZONE)
      speedInput = 0;
    if (yawInput > -DEAD_ZONE && yawInput < DEAD_ZONE)
      yawInput = 0;

    int gamma = 3; //gain
    int speedExp =(speedInput^gamma)/(PWM_LIMIT^(gamma-1));
    int gammaYaw = 2; //gain
    int yawExp =(yawInput^gammaYaw)/(YAW_SENSITIVITY^(gammaYaw-1));

    leftSpeed = speedExp + yawExp;
    rightSpeed = speedExp - yawExp;

  Serial.print(F("Left:"));Serial.println(int(leftSpeed));
  Serial.print(F("Right:"));Serial.println(int(rightSpeed));


  }

  Serial.println(":");

  LeftMotorSetSpeed(leftSpeed);
  RightMotorSetSpeed(rightSpeed);

  delay(1);
}

