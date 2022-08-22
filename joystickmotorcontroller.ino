const byte XPotPin = A0;  // Left-Right control
const byte YPotPin = A1;  // Forward-backward throttle

const int PWM_LIMIT = 255;
const int DEAD_ZONE = 10;
const int YAW_SENSITIVITY = 100;

void setup()
{
  // Set pin modes on motor pins
}

void LeftMotorSetSpeedForward(byte speed)
{
  // Set motor pins to run the motor forward at the specified speed
  (void) speed;
}

void LeftMotorSetSpeedBackward(byte speed)
{
  // Set motor pins to run the motor backward at the specified speed
  (void) speed;
}

void RightMotorSetSpeedForward(byte speed)
{
  // Set motor pins to run the motor forward at the specified speed
  (void) speed;
}

void RightMotorSetSpeedBackward(byte speed)
{
  // Set motor pins to run the motor backward at the specified speed
  (void) speed;
}

void loop()
{
  int speedInput = analogRead(YPotPin); // Forward/Reverse
  int yawInput = analogRead(XPotPin); // Left/Right turn

  // map 'speed' to the range -PWM_LIMIT (backward), +PWM_LIMIT (forward)
  speedInput = map(speedInput, 0, 1023, -PWM_LIMIT, PWM_LIMIT);
  yawInput = map(yawInput, 0, 1023, -YAW_SENSITIVITY, YAW_SENSITIVITY);

  // Put in dead zones
  if (speedInput > -DEAD_ZONE && speedInput < DEAD_ZONE)
    speedInput = 0;
  if (yawInput > -DEAD_ZONE && yawInput < DEAD_ZONE)
    yawInput = 0;

  int leftSpeed = speedInput + yawInput;
  int rightSpeed = speedInput - yawInput;

  // neither motor can go faster than maximum speed
  leftSpeed = constrain(leftSpeed, -PWM_LIMIT, PWM_LIMIT);
  rightSpeed = constrain(rightSpeed, -PWM_LIMIT, PWM_LIMIT);

  if (leftSpeed < 0)
    LeftMotorSetSpeedBackward(-leftSpeed);
  else
    LeftMotorSetSpeedForward(leftSpeed);

  if (rightSpeed < 0)
    RightMotorSetSpeedBackward(-rightSpeed);
  else
    RightMotorSetSpeedForward(rightSpeed);
}

