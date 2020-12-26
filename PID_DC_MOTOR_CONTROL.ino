
/*
 *  Simple arduino PID motor position control
 *  Fine tune for better results
 *  Implemented on ASlONG DC Motor with a magnetic encoder (JGB3-520B-12v-178RPM)
 */

#include <PIDController.h>

PIDController pid;

volatile int degree = 0;
int encoderSignalAPin = 2;    // Signal A from 1st Hall sensor 
int encoderSignalBPin = 3;    // Signal B from 2nd Hall sensor

int motorOP1Pin = 9;          // Motor output1 (OP1) connect to the IN1 of L293D motor driver shield
int motorOP2Pin = 10;         // Motor output2 (OP2) connect to the IN2 of L293D motor driver shield
int motorDirection = 0;       // Motor direction

int encoderReversePulseCount = 0;
int encoderForwardPulseCount = 0;

void setup()
{
  Serial.begin(115200);      // Begin serial communication
  pinMode(encoderSignalAPin, INPUT); 
  pinMode(encoderSignalBPin, INPUT); 
  pinMode(motorOP1Pin, OUTPUT);      
  pinMode(motorOP2Pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderSignalAPin), detectReverseISR, RISING);
  pid.begin();               // Initialize the PID instance
  pid.tune(15, 3, 3000);     // Tune the PID, arguments: kP, kI, kD (Fine tune for better responses)
  pid.limit(-255, 255);      // Limit the PID output between 0 and 255, this is important to get rid of integral windup!

}

void loop()
{
  init:
  
  if (Serial.available())
  {
    int setTargetPositon = Serial.parseInt();
    if (setTargetPositon <= 360 && setTargetPositon >= -360 )
    {
      Serial.print("Target position: ");
      Serial.println(setTargetPositon);
      pid.setpoint(setTargetPositon);   // Set the "goal" the PID controller tries to "reach"
      Serial.println("Position reached");
    }
    else 
    {
      Serial.println("Out of range value:");
      goto init;
    }
  }

  degree = encoderReversePulseCount / 1.72222;  // Approximate pulses convertion to degrees
  int outputSignal = pid.compute(degree);       // PID compute the value and returns the optimal output
  if (outputSignal > 0) 
  {
    RotateCounterRotateClockwise(outputSignal); // Rotate the motor to counter clockwise
  }
  else 
  {
    RotateClockwise(abs(outputSignal));         // Rotate the motor to clockwise
  } 
  delay(10);                                    // delay is important to tune

}

/*
 * ISR to detect reverse direction when the encoder is moved
 */
void detectReverseISR()
{
  motorDirection = digitalRead(encoderSignalBPin);
  if (!motorDirection) 
  {
    encoderReversePulseCount += 1;
    if (degree >= 360) 
    {
      encoderReversePulseCount = 0;
    }
  }
  else {
    encoderReversePulseCount += -1;
    if (degree <= -360) 
    {
      encoderReversePulseCount = 0;
    }
  }
  attachInterrupt(digitalPinToInterrupt(encoderSignalAPin), detectForwardISR, FALLING); 
}

/*
 * ISR to detect forward direction when the encoder is moved
 */
void detectForwardISR()
{
  motorDirection = digitalRead(encoderSignalBPin); 
  if (motorDirection) 
  {
    encoderForwardPulseCount += 1; 
  }
  else 
  {
    encoderForwardPulseCount += -1; 
  }
  attachInterrupt(digitalPinToInterrupt(encoderSignalAPin), detectReverseISR, RISING);
}

/*
 * To rotate the motor to clockwise direction
 */
void RotateClockwise(int power) 
{
  if (power > 95) 
  {
    analogWrite(motorOP1Pin, power);
    digitalWrite(motorOP2Pin, LOW);
  } 
  else 
  {
    digitalWrite(motorOP1Pin, LOW);
    digitalWrite(motorOP2Pin, LOW);
  }
}

/*
 * To rotate the motor to counter clockwise direction
 */
void RotateCounterRotateClockwise(int power) 
{
  if (power > 95) 
  {
    analogWrite(motorOP2Pin, power);
    digitalWrite(motorOP1Pin, LOW);
  } 
  else 
  {
    digitalWrite(motorOP1Pin, LOW);
    digitalWrite(motorOP2Pin, LOW);
  }
}
