// Include necessary libraries
#include <LiquidCrystal.h>  // Library for interfacing with LCD
#include <SD.h>             // Library for SD card functionality

// LCD setup
const int rs = 31, en = 30, d0 = 21, d1 = 20, d2 = 19, d3 = 18, d4 = 37, d5 = 36, d6 = 35, d7 = 34;
LiquidCrystal lcd(rs, en, d0, d1, d2, d3, d4, d5, d6, d7);  // Initialize LCD object

// Definitions for motor control pins
#define IR_SENSOR_RIGHT 11   // Right IR sensor pin
#define IR_SENSOR_LEFT 12    // Left IR sensor pin
#define MOTOR_SPEED 100      // Default motor speed

int enableRightMotor = 2;    // Motor enable pin for the right motor
int rightMotorPin1 = 4;      // Right motor control pin 1
int rightMotorPin2 = 5;      // Right motor control pin 2

int enableLeftMotor = 3;     // Motor enable pin for the left motor
int leftMotorPin1 = 6;       // Left motor control pin 1
int leftMotorPin2 = 7;       // Left motor control pin 2

// Definitions for analog sensors
#define ANALOG_IN_PIN A0      // Analog input pin for voltage measurement
const int ledPin = 13;        // Pin for an LED

// Definitions for ultrasonic sensor
#define echoPin A2            // Pin connected to the echo output of the ultrasonic sensor
#define trigPin A3            // Pin connected to the trigger input of the ultrasonic sensor

// Variables for voltage and current calculations
float adc_voltage = 0.0;      // Variable to store calculated voltage from analog sensor
float in_voltage = 0.0;       // Variable to store voltage across the resistor
float R1 = 30000.0;           // Resistance R1 value
float R2 = 7500.0;            // Resistance R2 value
float ref_voltage = 4.0;      // Reference voltage for analog sensor
int adc_value = 0;            // Raw ADC value from analog sensor

const int analogchannel = 1;  // Analog channel for current measurement
int sensitivity = 185;        // Sensitivity for current measurement
int adcvalue = 0;             // Raw ADC value for current measurement
int offsetvoltage = 2500;     // Offset voltage for current measurement
double Voltage = 0;           // Calculated voltage for current measurement
double ecurrent = 0;          // Calculated current

// Variables for ultrasonic sensor
long duration;                // Variable to store the duration of the ultrasonic pulse
int distance;                 // Variable to store calculated distance from ultrasonic sensor

// Variables for timed operations
unsigned long previousMillis = 0;  // Variable to store the last time LCD was updated
const long interval = 1000;         // Interval for updating LCD (1 second)

// File for storing path information
File pathFile;

// Setup function
void setup() {
  // Set timer for analogWrite to improve PWM on certain pins
  TCCR0B = TCCR0B & B11111000 | B00000010;

  // Motor control pin configuration
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // IR sensor pin configuration
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

  // Ultrasonic sensor pin configuration
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize motors
  rotateMotor(0, 0);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.print("Line Follower");
  delay(1500);
  lcd.clear();

  // Analog sensor pin configuration
  pinMode(ANALOG_IN_PIN, INPUT);
  pinMode(ledPin, OUTPUT);

  // Serial communication initialization
  Serial.begin(9600);

  // Initialize SD card
  if (SD.begin()) {
    Serial.println("SD card is ready to use.");
    pathFile = SD.open("path.txt", FILE_WRITE);
    if (pathFile) {
      pathFile.println("Sample Path"); // Add your initial path data here
      pathFile.close();
    } else {
      Serial.println("Error opening path file.");
    }
  } else {
    Serial.println("SD card initialization failed.");
  }
}

// Main loop function
void loop() {
  // Get current time
  unsigned long currentMillis = millis();

  // Read analog sensor values
  adc_value = analogRead(ANALOG_IN_PIN);
  adc_voltage = (adc_value * ref_voltage) / 1024.0;
  in_voltage = adc_voltage * (R1 + R2) / R2;

  // Read current sensor values
  adcvalue = analogRead(analogchannel);
  Voltage = (adcvalue / 1024.0) * 3000;
  ecurrent = ((Voltage - offsetvoltage) / sensitivity);

  // Display voltage and current on LCD
  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(in_voltage, 2);
  lcd.print("V");

  lcd.setCursor(9, 1);
  lcd.print("I:");
  lcd.print(ecurrent, 3);
  lcd.print("A");
  lcd.setCursor(0, 0);

  // Trigger ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // Check for obstacles
  if (distance < 20) {
    rotateMotor(0, 0);
    lcd.print("Obstacle detected!");
    delay(3000);
    lcd.clear();
    handleObstacle();
  } else {
    // Read IR sensor values for line following
    int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
    int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

    // Adjust movement based on IR sensor readings
    if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) {
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
      lcd.print("Forward");
    } else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) {
      rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
      lcd.print("Right");
    } else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) {
      rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
      lcd.print("Left");
    } else {
      rotateMotor(0, 0);
      lcd.print("Stop");
    }
  }

  // Display voltage and current every second
  if (currentMillis - previousMillis >= interval) {
    lcd.setCursor(0, 1);
    lcd.print("V:");
    lcd.print(in_voltage, 2);
    lcd.print("V");

    lcd.setCursor(9, 1);
    lcd.print("I:");
    lcd.print(ecurrent, 3);
    lcd.print("A");

    previousMillis = currentMillis;
  }
}

// Function to control motor movement
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  // Control right motor
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  // Control left motor
  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  // Set motor speeds
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}

// Function to handle obstacles
void handleObstacle() {
  // Perform a 360-degree turn
  rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  delay(2000); // Adjust duration for a complete turn

  // Stop after the turn
  rotateMotor(0, 0);

  // Continue following the stored path
  followStoredPath();
}

// Function to follow the stored path
void followStoredPath() {
  lcd.clear();
  lcd.print("Following Path");

  // Open the path file for reading
  pathFile = SD.open("path.txt");
  if (pathFile) {
    while (pathFile.available()) {
      char character = pathFile.read();
      lcd.write(character);
      delay(100); // Adjust delay for path display speed
    }
    pathFile.close();
  } else {
    Serial.println("Error opening path file for reading.");
  }

  delay(3000); // Adjust delay after path display
  lcd.clear();
}
