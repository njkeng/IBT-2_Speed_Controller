/*
IBT-2 Motor Control Board driven by Arduino.
 
Speed and direction controlled by a potentiometer attached to analog input 0.
One side pin of the potentiometer (either one) to ground; the other side pin to +5V
 
Connection to the IBT-2 board:
IBT-2 pin 1 (RPWM) to Arduino pin 5(PWM) FORWARD direction
IBT-2 pins 3 (R_EN), 4 (L_EN) to Arduino pin 4
IBT-2 pin 7 (VCC) to Arduino 5V pin
IBT-2 pin 8 (GND) to Arduino GND
IBT-2 pins 2 (LPWM), 5 (R_IS) and 6 (L_IS) not connected

Other connections:
Potentiometer centre pin to Arduino pin A0
Enable pushbutton to Arduino pin 8

*/

#include <Arduino.h>

// Minimum and maximum output speeds
// Specified in the range 0 - 255
float minSpeed = 60.0;
float maxSpeed = 150.0;

// Starting and ramp up variables
float rampingTime = 2000;      // Time it takes to ramp up in milliseconds
float rampingValue = 0.0;
bool runEnable = false;
long timeStarted = 0;
bool buttonPressed = false;
bool startupComplete = true;

// IO pins
int SENSOR_PIN = 0;       // Center pin of the potentiometer to Arduino A0
int IBT_2_ENABLE = 4;     // Arduino digital output pin 4; connect to IBT-2 pin 3 and 4 (Enable)
int FWD_PWM_OUTPUT = 5;   // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int ENABLE_BUTTON = 8;    // Arduino digital input pin 8; connect to the enable pushbutton

// For button debounce.  Replicate for each button that needs debouncing
char enableButtonState = 0;           // state of button
unsigned long enableButtonCount = 0;  // button debounce timer

// Debounce subroutine
//
boolean buttonDown(char button, unsigned long *marker, char *butnstate, unsigned long interval) {

  // Deal with a button read; true if button pressed and debounced is a new event
  // Uses reading of button input, debounce store, state store and debounce interval.

  switch (*butnstate) {               // Odd states if was pressed, >= 2 if debounce in progress
    case 0: // Button up so far,
      if (button == HIGH) return false; // Nothing happening!
      else {
        *butnstate = 2;                 // record that is now pressed
        *marker = millis();             // note when was pressed
        return false;                   // and move on
      }

    case 1: // Button down so far,
      if (button == LOW) return false; // Nothing happening!
      else {
        *butnstate = 3;                 // record that is now released
        *marker = millis();             // note when was released
        return false;                   // and move on
      }

    case 2: // Button was up, now down.
      if (button == HIGH) {
        *butnstate = 0;                 // no, not debounced; revert the state
        return false;                   // False alarm!
      }
      else {
        if (millis() - *marker >= interval) {
          *butnstate = 1;               // jackpot!  update the state
          return true;                  // because we have the desired event!
        }
        else
          return false;                 // not done yet; just move on
      }

    case 3: // Button was down, now up.
      if (button == LOW) {
        *butnstate = 1;                 // no, not debounced; revert the state
        return false;                   // False alarm!
      }
      else {
        if (millis() - *marker >= interval) {
          *butnstate = 0;               // Debounced; update the state
          return false;                 // but it is not the event we want
        }
        else
          return false;                 // not done yet; just move on
      }
    default:                            // Error; recover anyway
      {
        *butnstate = 0;
        return false;                   // Definitely false!
      }
  }
}  // End of button debounce

void setup() {

  // Set up Arduino IO
  pinMode(IBT_2_ENABLE, OUTPUT);
  pinMode(FWD_PWM_OUTPUT, OUTPUT);
  pinMode(ENABLE_BUTTON, INPUT_PULLUP);

}

void loop() {


  // Read and debounce the encoder pushbutton
  //
  if (buttonDown(digitalRead(ENABLE_BUTTON), &enableButtonCount, &enableButtonState, 10UL )) {  // debounce timer is 10 msec
    buttonPressed = true;
  } else buttonPressed = false;
 
  // Check for startup or shutdown
  if (buttonPressed) {
    if (runEnable) {
      runEnable = false;
    }
    else {
      runEnable = true;
      timeStarted = millis();
    }
    startupComplete = false;
    buttonPressed = false;
  }

  // Ramp on startup
  if (runEnable && !startupComplete) {
    rampingValue = (millis() - timeStarted) / rampingTime;
    if (rampingValue >= 1.0) {
      startupComplete = true;
    }
  } 
  
  if (!runEnable) rampingValue = 0.0;

  // Read speed control potentiometer
  int sensorValue = (analogRead(SENSOR_PIN));

  // Scale potentiometer value to the range for PWM output
  // sensor value is in the range 0 to 1023
  // PWM value is 0 - 255
  float forwardPWM = sensorValue * (maxSpeed-minSpeed) * rampingValue / 1024 + minSpeed;

  // Write outputs
  digitalWrite (IBT_2_ENABLE, runEnable);
  analogWrite(FWD_PWM_OUTPUT, forwardPWM);
}

