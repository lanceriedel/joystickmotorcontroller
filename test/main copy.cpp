#include <Arduino.h>

const byte XPotPin = A0;  // Left-Right control
const byte YPotPin = A1;  // Forward-backward throttle

const int INPUT_STATE_RADIO_ON = 1;
const int INPUT_STATE_JOYSTICK_ON = 2;
const int INPUT_STATE_OFF = 0;

int MOTORLEFT = A2; // Arduino pin 6 is connected to MDDS60 pin AN1.
int MOTORRIGHT = A3; // Arduino pin 5 is connected to MDDS60 pin MOTORRIGHT.
int RCYAW = A5; // 
int RCSPEED = A4; // Arduino pin 5 is connected to MDDS60 pin MOTORRIGHT.

int JOYYWIN = A0; // Arduino pin 6 is connected to MDDS60 pin AN1.
int JOYXWIN = A1; // Arduino pin 5 is connected to MDDS60 pin MOTORRIGHT.

int INPUT_STATE = INPUT_STATE_OFF; //1 = RADIO, 0 = OFF, 2 = JOYSTICK
int INPUT_STATE_RADIO = 5;
int INPUT_STATE_JOY = 6;



const int PWM_LIMIT = 127;
const int DEAD_ZONE = 10;
const int YAW_SENSITIVITY = 100;


void setup()
{
  // Set pin modes on motor pins
  pinMode(MOTORLEFT, OUTPUT); // Set Arduino pin 6 (MOTORLEFT) as output.
  pinMode(MOTORRIGHT, OUTPUT); // Set Arduino pin 5 (MOTORRIGHT) as output.
  analogWrite(MOTORLEFT, 0);//
  analogWrite(MOTORRIGHT, 0);//

  pinMode(RCSPEED,INPUT);
  pinMode(RCYAW,INPUT);

  pinMode(INPUT_STATE_JOY,INPUT);
  pinMode(INPUT_STATE_RADIO,INPUT);

  Serial.begin(9600);
  Serial.print(F("setup"));

}

void LeftMotorSetSpeed(byte speed)
{
  // Set motor pins to run the motor forward at the specified speed
  (void) speed;
  Serial.print(F("leftmotor:"));Serial.println(int(speed));
  analogWrite(MOTORLEFT, speed);   

}


void RightMotorSetSpeed(byte speed)
{
  // Set motor pins to run the motor forward at the specified speed
  (void) speed;
   Serial.print(F("rightmotor:"));Serial.println(int(speed));
     analogWrite(MOTORRIGHT, speed);   

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

  int leftSpeed =127;
  int rightSpeed = 127;
  
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
    Serial.println(":");


    // map 'speed' to the range -PWM_LIMIT (backward), +PWM_LIMIT (forward)
    speedInput = map(speedInput, 900, 2000, -PWM_LIMIT, PWM_LIMIT);
    yawInput = map(yawInput, 900, 2000, -YAW_SENSITIVITY, YAW_SENSITIVITY);

    // Put in dead zones
    if (speedInput > -DEAD_ZONE && speedInput < DEAD_ZONE)
      speedInput = 0;
    if (yawInput > -DEAD_ZONE && yawInput < DEAD_ZONE)
      yawInput = 0;

    leftSpeed = speedInput + yawInput;
    rightSpeed = speedInput - yawInput;

    // Serial.print(F("yawInputRAW:"));Serial.println(int(yawInput));
    //Serial.print(F("speedInputRAW:"));Serial.println(int(speedInput));

  //   Serial.print(F("leftSpeedRAW:"));Serial.println(int(leftSpeed));
  //  Serial.print(F("rightSpeedRAW:"));Serial.println(int(rightSpeed));

    // neither motor can go faster than maximum speed
    leftSpeed = constrain(leftSpeed, -PWM_LIMIT, PWM_LIMIT);
    rightSpeed = constrain(rightSpeed, -PWM_LIMIT, PWM_LIMIT);
  }

//Switched to JOYSTICK
  else if (inputState==INPUT_STATE_JOYSTICK_ON) {

    int speedInput = analogRead(JOYYWIN); // Forward/Reverse
    int yawInput = analogRead(JOYXWIN); // Left/Right turn
    Serial.print(F("yawInputJ:"));Serial.println(int(yawInput));
    Serial.print(F("speedInputJ:"));Serial.println(int(speedInput));
    Serial.println(":");


    // map 'speed' to the range -PWM_LIMIT (backward), +PWM_LIMIT (forward)
    speedInput = 1023-speedInput;
    speedInput = map(speedInput, 0, 1023, -PWM_LIMIT, PWM_LIMIT);
    yawInput = map(yawInput, 0, 1023, -YAW_SENSITIVITY, YAW_SENSITIVITY);

    // Put in dead zones
    if (speedInput > -DEAD_ZONE && speedInput < DEAD_ZONE)
      speedInput = 0;
    if (yawInput > -DEAD_ZONE && yawInput < DEAD_ZONE)
      yawInput = 0;

    leftSpeed = speedInput + yawInput;
    rightSpeed = speedInput - yawInput;

    // Serial.print(F("yawInputRAW:"));Serial.println(int(yawInput));
    //Serial.print(F("speedInputRAW:"));Serial.println(int(speedInput));

  //   Serial.print(F("leftSpeedRAW:"));Serial.println(int(leftSpeed));
  //  Serial.print(F("rightSpeedRAW:"));Serial.println(int(rightSpeed));

    // neither motor can go faster than maximum speed
    leftSpeed = constrain(leftSpeed, -PWM_LIMIT, PWM_LIMIT);
    rightSpeed = constrain(rightSpeed, -PWM_LIMIT, PWM_LIMIT);
  }



 
  Serial.println(":");

  if (leftSpeed < 0)
    LeftMotorSetSpeed(leftSpeed + 127);
  else
    LeftMotorSetSpeed(leftSpeed );

  if (rightSpeed < 0)
    RightMotorSetSpeed(rightSpeed + 127);
  else
    RightMotorSetSpeed(rightSpeed );

  delay(1150);
}

