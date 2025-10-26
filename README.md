/*****************************************************
 * Advanced PID Line Follower Robot
 * Components: Arduino Nano, L298N, 5 IR Sensors,
 * HC-05 Bluetooth Module, Buck Converter
 * Features: Real-time PID tuning via Bluetooth
 *****************************************************/

#include <SoftwareSerial.h>
SoftwareSerial BT(2, 3); // RX | TX (connect to TX | RX of HC-05)

// ---- Motor Pins ----
#define ENA 5
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENB 10

// ---- IR Sensor Pins ----
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4

// ---- PID Variables ----
float Kp = 25.0;
float Ki = 0.0;
float Kd = 15.0;

int lastError = 0;
float integral = 0;
float output = 0;

int baseSpeed = 150; // Adjust based on motor speed

// ---- Bluetooth buffer ----
String inputString = "";

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  Serial.println("PID Line Follower with Bluetooth Control");
  BT.println("Send commands: Kp=xx, Ki=xx, Kd=xx");
}

void loop() {
  readBluetooth(); // Check if Kp, Ki, Kd updated
  int position = readSensors(); // Get line position (-2 to +2)

  int error = position;
  integral += error;
  int derivative = error - lastError;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int leftSpeed = baseSpeed - output;
  int rightSpeed = baseSpeed + output;

  setMotorSpeed(leftSpeed, rightSpeed);

  lastError = error;

  // Debug info
  Serial.print("Err: "); Serial.print(error);
  Serial.print("\tOut: "); Serial.print(output);
  Serial.print("\tKp: "); Serial.print(Kp);
  Serial.print("\tKi: "); Serial.print(Ki);
  Serial.print("\tKd: "); Serial.println(Kd);

  delay(10);
}

// ---- Sensor Reading Function ----
int readSensors() {
  int s[5];
  s[0] = !digitalRead(S1);
  s[1] = !digitalRead(S2);
  s[2] = !digitalRead(S3);
  s[3] = !digitalRead(S4);
  s[4] = !digitalRead(S5);

  // Weighted position calculation
  int weightedSum = (-2)*s[0] + (-1)*s[1] + 0*s[2] + 1*s[3] + 2*s[4];
  int activeSensors = s[0] + s[1] + s[2] + s[3] + s[4];
  
  if (activeSensors == 0)
    return lastError; // Line lost, maintain previous direction

  return weightedSum / activeSensors;
}

// ---- Motor Control ----
void setMotorSpeed(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Left Motor
  if (left >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, left);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -left);
  }

  // Right Motor
  if (right >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, right);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -right);
  }
}

// ---- Bluetooth Control ----
void readBluetooth() {
  while (BT.available()) {
    char c = BT.read();
    if (c == '\n') {
      parseCommand(inputString);
      inputString = "";
    } else {
      inputString += c;
    }
  }
}

// ---- Parse Kp, Ki, Kd from Bluetooth ----
void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("Kp=")) {
    Kp = cmd.substring(3).toFloat();
    BT.println("Updated Kp = " + String(Kp));
  } else if (cmd.startsWith("Ki=")) {
    Ki = cmd.substring(3).toFloat();
    BT.println("Updated Ki = " + String(Ki));
  } else if (cmd.startsWith("Kd=")) {
    Kd = cmd.substring(3).toFloat();
    BT.println("Updated Kd = " + String(Kd));
  } else if (cmd == "show") {
    BT.println("Kp=" + String(Kp) + ", Ki=" + String(Ki) + ", Kd=" + String(Kd));
  } else {
    BT.println("Invalid cmd. Use Kp=xx, Ki=xx, Kd=xx");
  }
}
