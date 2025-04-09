//esp=dc_motor+arduino_interface(slave)
#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// I2C Setup
#define I2C_DEV_ADDR 0x08  // Arduino Uno I2C slave address

// Front Driver (Controls M1 and M2)
#define FRONT_M1_IN1 12  // M1 (front-left) direction pin 1
#define FRONT_M1_IN2 13  // M1 (front-left) direction pin 2
#define FRONT_M2_IN1 14  // M2 (front-right) direction pin 1
#define FRONT_M2_IN2 15  // M2 (front-right) direction pin 2

// Rear Driver (Controls M3 and M4)
#define REAR_M3_IN1 25   // M3 (rear-left) direction pin 1
#define REAR_M3_IN2 23   // M3 (rear-left) direction pin 2
#define REAR_M4_IN1 18   // M4 (rear-right) direction pin 1
#define REAR_M4_IN2 19   // M4 (rear-right) direction pin 2

// Function Declarations
void stopMotors();
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void servoCon(char command);

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Controller");

  Wire.begin();  // SDA = GPIO21, SCL = GPIO22

  // Motor pin setup
  pinMode(FRONT_M1_IN1, OUTPUT);
  pinMode(FRONT_M1_IN2, OUTPUT);
  pinMode(FRONT_M2_IN1, OUTPUT);
  pinMode(FRONT_M2_IN2, OUTPUT);
  pinMode(REAR_M3_IN1, OUTPUT);
  pinMode(REAR_M3_IN2, OUTPUT);
  pinMode(REAR_M4_IN1, OUTPUT);
  pinMode(REAR_M4_IN2, OUTPUT);

  stopMotors();
  Serial.println("ESP32 Bluetooth + I2C Motor Control Started");
}

void loop() {
  // Bluetooth Command Processing
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.printf("Command Received: %c\n", command);

    switch (command) {
      case 'F': moveForward(); break;
      case 'B': moveBackward(); break;
      case 'L': turnLeft(); break;
      case 'R': turnRight(); break;
      case 'O':
      case 'X':
        servoCon(command); break;
      case 'S': stopMotors(); break;
      default:
        Serial.println("Invalid Command"); break;
    }
  }

  // Read response from Arduino (if any)
  Wire.requestFrom(I2C_DEV_ADDR, 16);
  while (Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }
  Serial.println();

  delay(200);  // Adjust delay for responsiveness
}

// ---------------------- I2C Servo Command ----------------------

void servoCon(char command) {
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(command);
  uint8_t error = Wire.endTransmission();
  Serial.printf("Sent '%c' via I2C (error: %u)\n", command, error);
}

// ---------------------- Motor Control Functions ----------------------

void moveForward() {
  digitalWrite(FRONT_M1_IN1, LOW);
  digitalWrite(FRONT_M1_IN2, HIGH);
  digitalWrite(FRONT_M2_IN1, LOW);
  digitalWrite(FRONT_M2_IN2, HIGH);
  digitalWrite(REAR_M3_IN1, LOW);
  digitalWrite(REAR_M3_IN2, HIGH);
  digitalWrite(REAR_M4_IN1, LOW);
  digitalWrite(REAR_M4_IN2, HIGH);
  Serial.println("Moving forward");
}

void moveBackward() {
  digitalWrite(FRONT_M1_IN1, HIGH);
  digitalWrite(FRONT_M1_IN2, LOW);
  digitalWrite(FRONT_M2_IN1, HIGH);
  digitalWrite(FRONT_M2_IN2, LOW);
  digitalWrite(REAR_M3_IN1, HIGH);
  digitalWrite(REAR_M3_IN2, LOW);
  digitalWrite(REAR_M4_IN1, HIGH);
  digitalWrite(REAR_M4_IN2, LOW);
  Serial.println("Moving backward");
}

void turnRight() {
  digitalWrite(FRONT_M1_IN1, HIGH);
  digitalWrite(FRONT_M1_IN2, LOW);
  digitalWrite(FRONT_M2_IN1, LOW);
  digitalWrite(FRONT_M2_IN2, HIGH);
  digitalWrite(REAR_M3_IN1, HIGH);
  digitalWrite(REAR_M3_IN2, LOW);
  digitalWrite(REAR_M4_IN1, LOW);
  digitalWrite(REAR_M4_IN2, HIGH);
  Serial.println("Turning right");
}

void turnLeft() {
  digitalWrite(FRONT_M1_IN1, LOW);
  digitalWrite(FRONT_M1_IN2, HIGH);
  digitalWrite(FRONT_M2_IN1, HIGH);
  digitalWrite(FRONT_M2_IN2, LOW);
  digitalWrite(REAR_M3_IN1, LOW);
  digitalWrite(REAR_M3_IN2, HIGH);
  digitalWrite(REAR_M4_IN1, HIGH);
  digitalWrite(REAR_M4_IN2, LOW);
  Serial.println("Turning left");
}

void stopMotors() {
  digitalWrite(FRONT_M1_IN1, LOW);
  digitalWrite(FRONT_M1_IN2, LOW);
  digitalWrite(FRONT_M2_IN1, LOW);
  digitalWrite(FRONT_M2_IN2, LOW);
  digitalWrite(REAR_M3_IN1, LOW);
  digitalWrite(REAR_M3_IN2, LOW);
  digitalWrite(REAR_M4_IN1, LOW);
  digitalWrite(REAR_M4_IN2, LOW);
  Serial.println("Motors stopped");
}