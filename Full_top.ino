#include <Wire.h>
#include <SparkFun_GridEYE_Arduino_Library.h>
#include <Servo.h>

Servo myServoTurret;
Servo myServoFlap;

int servoPin1 = 10;  // Or 6, depending on where you connected the signal wire
int servoPin2 = 11; // 

int E1 = 5;
int M1 = 4;

GridEYE grideye;
float frame[8][8];

const int ROWS = 8;
const int COLS = 8;
const float H_FOV = 60.0; // horizontal field of view in degrees
const float TEMP_THRESHOLD = 30.0;

const float Kp = 1.0;

int servoPosition = 90;  // Starting servo position (center position)
int oscDirection = 1;    // 1 for increasing angle, -1 for decreasing
const int oscStep = 1;   // Oscillation step in degrees

void setup() {
  Wire.begin();
  grideye.begin();
  Serial.begin(9600); // Ensure baud rate matches receiving Arduino

  // Turret
  myServoTurret.attach(servoPin1);
  myServoTurret.write(servoPosition);

  // Flap
  myServoFlap.attach(servoPin2);
  myServoFlap.write(servoPosition);

  pinMode(M1, OUTPUT);

  delay(1000);
}

void loop() {
  float sumTemp = 0.0;
  float sumWeightedX = 0.0;

  // Read all pixels from the Grid-EYE sensor into a 2D array
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      int pixelIndex = row * COLS + col;
      frame[row][col] = grideye.getPixelTemperature(pixelIndex);
    }
  }

  // Process the sensor data: accumulate data for pixels above the threshold.
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      float tempVal = frame[row][col];
      if (tempVal >= TEMP_THRESHOLD) {
        sumWeightedX += col * tempVal;
        sumTemp += tempVal;
      }
    }
  }

  // If a target is detected, use the centroid and P control to adjust servo.
  if (sumTemp > 0) {
    float xCentroid = sumWeightedX / sumTemp;
    // Calculate error relative to the center column of the grid
    float angleError = (xCentroid - (COLS - 1) / 2.0) * (H_FOV / COLS);
    Serial.print("xCentroid: ");
    Serial.print(xCentroid, 2);
    Serial.print("  Angle Error: ");
    Serial.println(angleError, 2);
    
    float adjustment = Kp * angleError;
    servoPosition += adjustment;
    // Constrain servo position to valid range
    if (servoPosition > 180) servoPosition = 180;
    if (servoPosition < 0) servoPosition = 0;
    
    myServoTurret.write(servoPosition);
    Serial.print("New Servo Position: ");
    Serial.println(servoPosition);

    // Yellow motor
    if (abs(angleError) < 10){
       digitalWrite(M1,HIGH);
       analogWrite(E1, 255);
    }
    else{
      digitalWrite(M1, LOW);
      analogWrite(E1, 0);
    }

    // Flap
    if (abs(angleError) < 5){
      for (int angle = 90; angle <= 180; angle++) {
        myServoFlap.write(angle);
        delay(10); // Adjust delay for desired sweep speed
      }
      for (int angle = 180; angle >= 90; angle--) {
        myServoFlap.write(angle);
        delay(10); // Adjust delay for desired sweep speed
      }
    }
  } 
  // Otherwise, oscillate the servo until a target is found.
  else {
    // Stop yellow motor
    digitalWrite(M1, LOW);
    analogWrite(E1, 0);
    
    // Update servo position based on current oscillation direction
    servoPosition += oscDirection * oscStep;
    // Reverse direction if the bounds are reached
    if (servoPosition >= 180) {
      servoPosition = 180;
      oscDirection = -1;
    } 
    else if (servoPosition <= 0) {
      servoPosition = 0;
      oscDirection = 1;
    }
    
    myServoTurret.write(servoPosition);
    Serial.print("Oscillating Servo Position: ");
    Serial.println(servoPosition);
  }
  
  delay(50); // Adjust loop delay as needed
}
