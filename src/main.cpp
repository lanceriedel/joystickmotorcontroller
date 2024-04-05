#include <Arduino.h>
#include "CytronMotorDriver.h"

// Constants and pin assignments
const int INPUT_STATE_RADIO_ON = 1;
const int INPUT_STATE_JOYSTICK_ON = 2;
const int INPUT_STATE_OFF = 0;

int MOTORLEFT = 3;
int MOTORRIGHT = 9;

int MOTORLEFTDIR = 4;
int MOTORRIGHTDIR = 10;

int RCYAW = A5;
int RCSPEED = A4;

int JOYYWIN = A0;
int JOYXWIN = A1;

int INPUT_STATE_RADIO = 5;
int INPUT_STATE_JOY = 6;

const int PWM_LIMIT = 200;
const int DEAD_ZONE = 20;
const int YAW_SENSITIVITY = 90;
const long updateInterval = 23; // Time interval (ms) to update the speed.
const int maxStep = PWM_LIMIT * 50 / 8000; // Max speed step per update.

// Motor driver setup
CytronMD motor1(PWM_DIR, MOTORLEFT, MOTORLEFTDIR);
CytronMD motor2(PWM_DIR, MOTORRIGHT, MOTORRIGHTDIR);

// Global variables for speed control
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
unsigned long lastUpdateTime = 0;

void setup() {
  Serial.begin(9600);
  Serial.print(F("setup"));

  pinMode(RCSPEED, INPUT);
  pinMode(RCYAW, INPUT);
  pinMode(INPUT_STATE_JOY, INPUT);
  pinMode(INPUT_STATE_RADIO, INPUT);

  motor1.setSpeed(0);
  motor2.setSpeed(0);
}


// Helper function to determine the sign of a number
int sign(int x) {
  return (x > 0) - (x < 0);
}


void updateMotorSpeeds() {
  if (millis() - lastUpdateTime >= updateInterval) {
    lastUpdateTime = millis();

    // Define the quick stop and dynamic step calculations
    int quickStopStep = maxStep; // Larger step for immediate action (stop/reverse)
    int dynamicStepLeft = max(1, abs(currentLeftSpeed) / 10);
    dynamicStepLeft = min(dynamicStepLeft, maxStep);
    
    int dynamicStepRight = max(1, abs(currentRightSpeed) / 10);
    dynamicStepRight = min(dynamicStepRight, maxStep);

    // Adjust left motor speed
    int stepSizeLeft = (currentLeftSpeed < targetLeftSpeed) ? dynamicStepLeft : -dynamicStepLeft;
    if ((currentLeftSpeed < 0 && targetLeftSpeed) >= 0 || (currentLeftSpeed >= 0 && targetLeftSpeed <= 0)) {
        // Immediate stop or reverse direction
        stepSizeLeft = (targetLeftSpeed > 0) ? quickStopStep : -quickStopStep;
    }
    currentLeftSpeed += stepSizeLeft;
    // Prevent overshooting the target speed
    if ((stepSizeLeft > 0 && currentLeftSpeed > targetLeftSpeed) || (stepSizeLeft < 0 && currentLeftSpeed < targetLeftSpeed)) {
        currentLeftSpeed = targetLeftSpeed;
    }

    // Adjust right motor speed using similar logic as left motor
    int stepSizeRight = (currentRightSpeed < targetRightSpeed) ? dynamicStepRight : -dynamicStepRight;
    if ((currentRightSpeed < 0 && targetRightSpeed >= 0) || (currentRightSpeed >= 0 && targetRightSpeed <= 0)) {
        // Immediate stop or reverse direction
        stepSizeRight = (targetRightSpeed > 0) ? quickStopStep : -quickStopStep;
    }
    currentRightSpeed += stepSizeRight;
    // Prevent overshooting the target speed
    if ((stepSizeRight > 0 && currentRightSpeed > targetRightSpeed) || (stepSizeRight < 0 && currentRightSpeed < targetRightSpeed)) {
        currentRightSpeed = targetRightSpeed;
    }

    motor1.setSpeed(currentLeftSpeed);
    motor2.setSpeed(currentLeftSpeed);
  }
}



void loop() {
  byte inputStateJoy = digitalRead(INPUT_STATE_JOY);
  byte inputStateRC = digitalRead(INPUT_STATE_RADIO);

  int inputState = INPUT_STATE_OFF;
  if (inputStateJoy == HIGH) {
    inputState = INPUT_STATE_JOYSTICK_ON;
  } else if (inputStateRC == HIGH) {
    inputState = INPUT_STATE_RADIO_ON;
  }

  targetLeftSpeed = 0;
  targetRightSpeed = 0;

  if (inputState == INPUT_STATE_OFF) {
    // Stop all motors if no input state is active
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
  }
  else if (inputState == INPUT_STATE_RADIO_ON) {
    int speedInput = pulseIn(RCSPEED, HIGH, 25000);
    int yawInput = pulseIn(RCYAW, HIGH, 25000);

    if (speedInput != 0 && yawInput != 0) {
      speedInput = map(speedInput, 900, 2000, -PWM_LIMIT, PWM_LIMIT);
      yawInput = map(yawInput, 900, 2000, -YAW_SENSITIVITY, YAW_SENSITIVITY);

      if (abs(speedInput) < DEAD_ZONE) speedInput = 0;
      if (abs(yawInput) < DEAD_ZONE) yawInput = 0;

      speedInput = -speedInput; // Adjust based on your control scheme

      targetLeftSpeed = constrain(speedInput + yawInput, -PWM_LIMIT, PWM_LIMIT);
      targetRightSpeed = constrain(speedInput - yawInput, -PWM_LIMIT, PWM_LIMIT);
    }
  }
  else if (inputState == INPUT_STATE_JOYSTICK_ON) {
    int speedInput = analogRead(JOYYWIN);
    int yawInput = analogRead(JOYXWIN);


    // Mapping the input
    speedInput = map(speedInput, 0, 1023, -PWM_LIMIT, PWM_LIMIT);
    yawInput = map(yawInput, 0, 1023, -YAW_SENSITIVITY, YAW_SENSITIVITY);

    // Debugging the input after mapping
   // Serial.print("Mapped Speed Input: "); Serial.println(speedInput);


    if (abs(speedInput) < DEAD_ZONE) speedInput = 0;
    if (abs(yawInput) < DEAD_ZONE) yawInput = 0;

//FOR JOYSTICK DONT USE YAW
  yawInput = 0;
    targetLeftSpeed = constrain(speedInput + yawInput, -PWM_LIMIT, PWM_LIMIT);
    targetRightSpeed = constrain(speedInput - yawInput, -PWM_LIMIT, PWM_LIMIT);
  }

  updateMotorSpeeds();

  delay(1); // Small delay to ensure stability and responsiveness
}


