
/*
 *  Synchronising two DC motors using simple arduino PID motor position control
 *  Fine tune for better results
 *  Implemented on ASlONG DC Motor with a magnetic encoder (JGB3-520B-12v-178RPM)
 */

#include <PIDController.h>

PIDController pid;

int encoderSignalAPin = 2;    // Encoder Signal A from 1st Hall sensor 
int encoderSignalBPin = 3;    // Encoder Signal B from 2nd Hall sensor

int encoder1SignalAPin = 20;  // Encoder1 Signal A from 1st Hall sensor
int encoder1SignalBPin = 21;  // Encoder1 Signal B from 1st Hall sensor

int motorOP1Pin = 9;          // Motor output1 (OP1) connect to the IN1 of L293D motor driver shield
int motorOP2Pin = 10;         // Motor output2 (OP2) connect to the IN2 of L293D motor driver shield
int motorDirection = 0;       // Motor direction
int encoder1Direction = 0;    // Encoder1 direction

volatile long int encoderReversePulseCount = 0;
volatile long int encoderForwardPulseCount = 0;

volatile long int encoder1ReversePulseCount = 0;
volatile long int encoder1ForwardPulseCount = 0;

void setup()
{
  Serial.begin(9600);        // Begin serial communication
  pinMode(encoderSignalAPin, INPUT); 
  pinMode(encoderSignalBPin, INPUT); 
  pinMode(encoder1SignalAPin,INPUT_PULLUP);
  pinMode(encoder1SignalBPin,INPUT_PULLUP);
  pinMode(motorOP1Pin, OUTPUT);      
  pinMode(motorOP2Pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderSignalAPin), detectReverseISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1SignalAPin), detectReverseISR1, RISING);
  pid.begin();               // Initialize the PID instance
  pid.tune(14, 2, 3000);     // Tune the PID, arguments: kP, kI, kD (Fine tune for better responses)
  pid.limit(-255, 255);      // Limit the PID output between 0 and 255, this is important to get rid of integral windup!

}

void loop()
{
  pid.setpoint(encoder1ReversePulseCount+encoder1ForwardPulseCount/2);   // Set the "goal" the PID controller tries to "reach"
    
  int outputSignal = pid.compute(encoderReversePulseCount+encoderForwardPulseCount/2);       // PID compute the value and returns the optimal output
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
  }
  else {
    encoderReversePulseCount += -1;
  }
  attachInterrupt(digitalPinToInterrupt(encoderSignalAPin), detectForwardISR, FALLING); 
}

/*
 * ISR1 to detect reverse direction when the encoder is moved
 */
void detectReverseISR1()
{
  encoder1Direction = digitalRead(encoder1SignalBPin);
  if (!encoder1Direction) 
  {
    encoder1ReversePulseCount += 1;
  }
  else {
    encoder1ReversePulseCount += -1;
  }
  attachInterrupt(digitalPinToInterrupt(encoder1SignalAPin), detectForwardISR1, FALLING); 
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
 * ISR to detect forward direction when the encoder is moved
 */
void detectForwardISR1()
{
  encoder1Direction = digitalRead(encoder1SignalBPin); 
  if (encoder1Direction) 
  {
    encoder1ForwardPulseCount += 1; 
  }
  else 
  {
    encoder1ForwardPulseCount += -1; 
  }
  attachInterrupt(digitalPinToInterrupt(encoder1SignalAPin), detectReverseISR1, RISING);
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
