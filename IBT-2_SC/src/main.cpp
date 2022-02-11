/*
IBT-2 Motor Control Board driven by Arduino.
 
Speed and direction controlled by a potentiometer attached to analog input 0.
One side pin of the potentiometer (either one) to ground; the other side pin to +5V
 
Connection to the IBT-2 board:
IBT-2 pin 1 (RPWM) to Arduino pin 5(PWM) FORWARD direction
IBT-2 pins 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
IBT-2 pin 8 (GND) to Arduino GND
IBT-2 pins 2 (LPWM), 5 (R_IS) and 6 (L_IS) not connected
*/

#include <Arduino.h>

// Minimum and maximum output speeds
// Specified in the range 0 - 255
float minSpeed = 50.0;
float maxSpeed = 110.0;

int SENSOR_PIN = 0; // center pin of the potentiometer
int Fwd_PWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)


void setup() {
  // put your setup code here, to run once:
  pinMode(Fwd_PWM_Output, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue = (analogRead(SENSOR_PIN));
 
  // sensor value is in the range 0 to 1023
  // PWM value is 0 - 255
  // forward rotation
  int forwardPWM = sensorValue * (maxSpeed-minSpeed) / 1024 + minSpeed;
  analogWrite(Fwd_PWM_Output, forwardPWM);
}