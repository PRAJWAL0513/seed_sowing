//arduino-2(slave)-servo_control(mg995)

#include <Wire.h>
#include <Servo.h>

#define I2C_ADDRESS 0x08
const int servoPin = 6;

Servo myServo;
bool servoOn = false;
char lastCommand = '\0';

void setup() {
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.begin(9600);
  myServo.attach(servoPin);
//  myServo.write(90); // Stop servo initially
}

void loop() {
  if (servoOn) {
    myServo.write(-180); // Rotate in one direction
  } else {
    myServo.write(90); // Stop
  }
}

// Handle command from ESP32
void receiveEvent(int numBytes) {
  if (Wire.available()) {
    char cmd = Wire.read();
    lastCommand = cmd;

    if (cmd == 'O') {
      servoOn = true;
    } else if (cmd == 'X') {
      servoOn = false;
    }

    Serial.print("Received command: ");
    Serial.println(cmd);
  }
}

// Respond when ESP32 requests data
void requestEvent() {
  if (servoOn) {
    Wire.write("Servo: ON ");
  } else {
    Wire.write("Servo: OFF");
  }
}