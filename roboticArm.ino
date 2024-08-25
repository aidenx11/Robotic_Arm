/**
 * Code to control the servos and potentiometers, and their communication, for my robotic arm.
 * Some of the code is modified from the AdaFruit PWM servo libraries sample "servo" code, notably 
 * the setServoPulse function.
 * 
 * @author Aiden Schroeder
 * @author Limor Fried
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Using default address 0x41, can be changed by soldering pins on the PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  130 // Minimum pulse length count (out of 4096)
#define SERVOMAX  500 // Maximum pulse length count (out of 4096)
#define MINTIME  0.6 // Minimum time for the PWM pulse (0 degrees)
#define MAXTIME 2.4 // Maximum time for the PWM pulse (180 degrees)
#define SERVO_FREQ 50 // My 9g micro servos run at 50Hz

double potVal = 0; // Variable to store potentiometer value, ranges from 0-1023
int potPin = A3; // Input pin of the current potentiometer, cycles through in the loop function

double prevPulse; // Value of the previous pulse

uint8_t servonum = 0; // Which servo is currently being modified

/**
 * Reads the current potentiometer (from potPin) and translates the value to amount of milliseconds to pulse the servo, 
 * so that it aligns in relation to the potentiometer
 */
double readPot() {
  potVal = analogRead(potPin);
  double factor = 0.00176; // Calculated as the time between max and min pulse, divided by the potentiometers max value (1023)
  double normalized = (potVal * factor) + MINTIME; // Translates the potentiometer value to a millisecond value to be used by setServoPulse()
  return roundToThree(normalized);
}

/**
 * Rounds the given number to three decimal places
 */
float roundToThree(double num) {
  int temp = round(num * 100);
  float rounded = temp / 100.0;
  return rounded;
}

void setup() {
  
  Serial.begin(9600);
  prevPulse = 0;
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Sample oscillator frequency that works for my chip, since I don't have an oscilloscope!
  pwm.setPWMFreq(SERVO_FREQ);  

  delay(10);
}

/**
 * Sets the pulse length given pulse as milliseconds with n being the servo number to set. 
 * Modified from the sample servo code from AdaFruit's PWM servo library.
 */
void setServoPulse(uint8_t n, double pulse) {
  
  double pulselength = 1000000; // 1 second -> 1000000 microseconds
  pulse /= 1000; // Translates given milliseconds to seconds
  
  pulselength /= SERVO_FREQ;
  pulselength /= 4096;  // 12 bits of resolution
  
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  
  pwm.setPWM(n, 0, pulse);
}

/**
 * Sets the servo pulse of the current servo to its corresponding potentiometer, then cycles both the servo and potentiometer.
 */
void loop() {
  
  setServoPulse(servonum, readPot());
  delay(3);
  potPin--;
  servonum++;
  if (potPin < A0) {
    potPin = A3;
  }
  if (servonum > 3) {
    servonum = 0;
  }
  
}
