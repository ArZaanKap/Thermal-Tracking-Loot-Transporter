#include <Servo.h>
#include <Motoron.h>
#include <Wire.h>
#include <math.h>  // For fabs()
#include <HX711.h>

// ---------- Pin and Object Definitions ----------
Servo myServo;
const int servoPin = 11;

const int trigPin = 9;
const int echoPin = 10;

MotoronI2C mc;

float prev_distance = 0;

#define calibration_factor -7050.0

float prevMass = 0;
float currMass = 0;

#define DOUT  3
#define CLK  2

HX711 scale;

// ---------- Setup Function ----------
void setup() {
  Serial.begin(9600);
 
  //pinMode(LED_BUILTIN, OUTPUT);/////////////////////////////////////////////////////////

  // Attach the servo
  myServo.attach(servoPin);
 
  // Set ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Load cell setup
  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
  prevMass = scale.get_units();
 
  // Initialize I2C and Motoron controller
  Wire.begin();
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.setMaxAcceleration(1, 140);
  mc.setMaxDeceleration(1, 300);
  mc.setMaxAcceleration(2, 140);
  mc.setMaxDeceleration(2, 300);
}

// ---------- Function to Read Distance ----------
float readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0343) / 2;
 
  // If measured distance is out of range, keep previous value
  if (distance > 400) {
    distance = prev_distance;
  }
  prev_distance = distance;
  return distance;
}

 
// ---------- Function to Perform a Servo Scan and Print Distance Map ----------
float scanDistanceMap() {

  float max_dist = 0;
  float max_dist_angle = 0;
   
  Serial.println("Starting servo scan...");

  // Sweep from 90° to 135°
  for (int angle = 90; angle <= 110; angle++) {
    myServo.write(angle);
    delay(30); // Allow time for servo to move
    float d = readDistance();
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" -> Distance: ");
    Serial.println(d);

    if (d > max_dist){
      max_dist = d;
      max_dist_angle = angle;
    }
  }

  // Sweep from 135° down to 45°
  for (int angle = 110; angle >= 70; angle--) {
    myServo.write(angle);
    delay(30);
    float d = readDistance();
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" -> Distance: ");
    Serial.println(d);

    if (d > max_dist){
      max_dist = d;
      max_dist_angle = angle;
    }
  }

  // Sweep back from 45° to 90°
  for (int angle = 70; angle <= 90; angle++) {
    myServo.write(angle);
    delay(30);
    float d = readDistance();
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" -> Distance: ");
    Serial.println(d);

    if (d > max_dist){
      max_dist = d;
      max_dist_angle = angle;
    }
  }
 
  Serial.println("Servo scan complete.");
  Serial.print("Max Distance: ");
  Serial.println(max_dist);
  Serial.println("");
 
  Serial.print("At Angle: ");
  Serial.println(max_dist_angle);
  Serial.println("");

  return max_dist_angle;
}

// ---------- Main Loop ----------

// Add these global variables
unsigned long turnStartTime;
unsigned long currentTurnDuration = 0;


void loop() {
  // Measure the current distance
  float distance = readDistance();
  Serial.print("Current Distance: ");
  Serial.println(distance);
  delay(100);

  currMass = scale.get_units();
  Serial.println(currMass);

  if (sqrt(pow(currMass - prevMass,2)) > 20){
    //digitalWrite(LED_BUILTIN, HIGH); /////////////////////////////////////////////////////////
    mc.setSpeed(1, 700);
    mc.setSpeed(2, -700);
    delay(500);
    mc.setSpeed(1, -700);
    mc.setSpeed(2, 700);
    delay(500);
    mc.setSpeed(1, 700);
    mc.setSpeed(2, -700);
    delay(500);
    mc.setSpeed(1, -700);
    mc.setSpeed(2, 700);
    delay(500);
  }else{
    //digitalWrite(LED_BUILTIN, LOW); ///////////////////////////////////////////////////////// 
  }
  prevMass = currMass;
  // If an obstacle is too close, perform a turn routine
  if (distance < 10) {
    // Slow down the motors
    mc.setSpeed(1, 300);
    mc.setSpeed(2, 300);
    delay(100);
   
    // Run a servo sweep to produce a distance map and obtain the best turning angle
    float maxAngle = scanDistanceMap();
    float error = maxAngle - 90;
    float msPerDegree = 200;
    currentTurnDuration = fabs(error) * msPerDegree;
   
    Serial.print("Max Angle: ");
    Serial.print(maxAngle);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Turn Duration: ");
    Serial.print(currentTurnDuration);
    Serial.println(" ms");

    const int turnSpeed = 300;
   
    if (error > 0) {
      mc.setSpeed(1, turnSpeed);
      mc.setSpeed(2, 0);
      turnStartTime = millis();
      Serial.println("Turning Right");
      while (millis() - turnStartTime < currentTurnDuration){
          mc.setSpeed(1, turnSpeed);
          mc.setSpeed(2, 0);
        }
    } else if (error < 0) {
      mc.setSpeed(1, 0);
      mc.setSpeed(2, turnSpeed);
      turnStartTime = millis();
      Serial.println("Turning Left");
      while (millis() - turnStartTime < currentTurnDuration){
          mc.setSpeed(1, 0);
          mc.setSpeed(2, turnSpeed);
        }
    } else {
      mc.setSpeed(1, turnSpeed);
      mc.setSpeed(2, -turnSpeed);
      turnStartTime = millis();
      Serial.println("Turning 180 degrees");
      while (millis() - turnStartTime < currentTurnDuration){
          mc.setSpeed(1, turnSpeed);
          mc.setSpeed(2, -turnSpeed); ////////////////////// new
        }
    }
  }
  mc.setSpeed(1, 700);
  mc.setSpeed(2, -700);
  delay(100);
}
