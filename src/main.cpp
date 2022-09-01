/* 
 * This example shows how to control MDDS60 in PWM mode with Arduino.
 * Set MDDS60 input mode to 0b10110100
 *
 * Reference Tutorial:
 * - Let's Arduino Controlling Your SmartDriveDuo-60
 *
 * Related Products:
 * - SmartDriveDuo-60: http://www.cytron.com.my/P-MDDS60
 * - CT UNO: http://www.cytron.com.my/p-ct-uno
 * - DC Brush Motors: http://www.cytron.com.my/c-84-dc-motor
 * - LiPo Battery: http://www.cytron.com.my/c-87-power/c-97-lipo-rechargeable-battery-and-charger
 *
 * URL: http://www.cytron.com.my
 */

#include <Arduino.h>


int DIG1 = 7; // Arduino pin 7 is connected to MDDS60 pin DIG1.
int DIG2 = 4; // Arduino pin 4 is connected to MDDS60 pin DIG2.
int AN1 = 5; // Arduino pin 6 is connected to MDDS60 pin AN1.
int AN2 = 6; // Arduino pin 5 is connected to MDDS60 pin AN2.

void setup()
{
  pinMode(DIG1, OUTPUT); // Set Arduino pin 7 (DIG1) as output.
  pinMode(DIG2, OUTPUT); // Set Arduino pin 4 (DIG2) as output.
  pinMode(AN1, OUTPUT); // Set Arduino pin 6 (AN1) as output.
  pinMode(AN2, OUTPUT); // Set Arduino pin 5 (AN2) as output.
  analogWrite(AN1, 127);//
  analogWrite(AN2, 127);//

  delay(500); // Delay for 5 seconds.
  Serial.begin(9600);
  Serial.println("Setup...");
}

void loop()
{
  // Controlling motor 1.
  int current = 0;
  int forward = 1;
  int switchdir = 0;
  while (1) {
   
    if (forward) current ++;
    else current --;
     if (current>255) {
      current = 255;
      forward = 0;
      if (switchdir==1) {
        digitalWrite(DIG1, HIGH);
        switchdir = 0;
      } else {
        digitalWrite(DIG1, LOW);
        switchdir = 1;
      }
      
    } else if (current<0) {
      current = 0;
      forward =1;
    } 
      Serial.print("currernt: ");
        Serial.println(current);
        analogWrite(AN1, current);
    delay(100); 

  }
  Serial.print("100: ");

  analogWrite(AN1, 100); // Set motor 1 speed less than half. Max is 255.
 // digitalWrite(DIG1, LOW); // Motor 1 start moving for 2s.
  delay(10000);
  // digitalWrite(DIG1, HIGH); // Motor 1 move to another direction for 2s.
  // delay(2000);
    Serial.print("0: ");

  analogWrite(AN1, 0); // Stop motor 1.
  delay(10000); // Delay for 1s
  Serial.print("100: ");
  analogWrite(AN1, 100);
  delay(10000); // Delay for 1s
    Serial.print("200: ");
  analogWrite(AN1, 200); // Stop motor 1.
    delay(10000); // Delay for 1s
  Serial.print("200: ");

    analogWrite(AN1, 20); // Stop motor 1.
      delay(10000); // Delay for 1s

Serial.print("again: ");


  // // Controlling motor 2.
  // analogWrite(AN2, 100); // Set motor 2 speed less than half. Max is 255.
  // digitalWrite(DIG2, LOW); // Motor 1 start moving for 2s.
  // delay(2000);
  // digitalWrite(DIG2, HIGH); // Motor 1 move to another direction for 2s.
  // delay(2000);
  // analogWrite(AN2, 0); // Stop motor 2.

  // delay(1000); // Delay for 1s
}