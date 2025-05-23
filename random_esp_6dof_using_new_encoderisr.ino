#include <Arduino.h>

// --- Pin Definitions ---
const int dirPins[6] = {19, 26, 27, 14, 12, 13};     // Direction control pins
const int pwmPins[6] = {18, 25, 33, 32, 5, 22};      // PWM control pins
const int encoderAPins[6] = {2, 4, 16, 23, 17, 23};   // Interrupt-capable pins
const int encoderBPins[6] = {15, 34, 35, 21, 22, 0};  // Digital pins

// --- PWM Configuration ---
const int pwmFreq = 1000;      // PWM frequency (Hz)
const int pwmResolution = 8;   // PWM resolution (bits, 8-bit = 0-255)
const int pwmChannels[6] = {0, 1, 2, 3, 4, 5}; // PWM channels for each motor

// --- Encoder Variables ---
volatile long encoderCounts[6] = {0, 0, 0, 0, 0, 0};
volatile int lastEncoded[6] = {0, 0, 0, 0, 0, 0};
bool invertEncoderCounting[6] = {false, false, true, false, false, false};

// Lookup table for quadrature decoding
const int8_t lookupTable[16] = {
  0,  // 0000
  -1,  // 0001
  +1, // 0010
  0,  // 0011
  +1, // 0100
  0,  // 0101
  0,  // 0110
  -1,  // 0111
  -1,  // 1000
  0,  // 1001
  0,  // 1010
  1, // 1011
  0,  // 1100
  1, // 1101
  -1,  // 1110
  0   // 1111
};

// --- PID Control Constants ---
float Kp[6] = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
float Ki[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float Kd[6] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
long previousErrors[6] = {0, 0, 0, 0, 0, 0};
float integrals[6] = {0, 0, 0, 0, 0, 0};

// --- Movement Control Parameters ---
const int maxSpeed = 255;       // Max motor PWM (ESP32 8-bit PWM)
const int minSpeed = 60;        // Minimum speed to overcome inertia
const int stopThreshold = 2;    // How close to target before stopping
const unsigned long moveTimeout = 100000; // Timeout in ms for movements

// Targets for each motor
long motorTargets[6] = {0, 0, 0, 0, 0, 0};
bool motorMoving[6] = {false, false, false, false, false, false};
unsigned long moveStartTimes[6] = {0, 0, 0, 0, 0, 0};

// --- Command Processing ---
String inputString = "";
boolean stringComplete = false;
const int MAX_ARGS = 10;
String cmdArgs[MAX_ARGS];

// --- ISR Functions for Encoders ---
void IRAM_ATTR updateEncoder(int motorIndex) {
  int MSB = digitalRead(encoderAPins[motorIndex]);
  int LSB = digitalRead(encoderBPins[motorIndex]);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded[motorIndex] << 2) | encoded;
  int increment = lookupTable[sum];
  if (invertEncoderCounting[motorIndex]) {
    increment = -increment;
  }
  encoderCounts[motorIndex] += increment;
  lastEncoded[motorIndex] = encoded;
}

void IRAM_ATTR encoderISR0() { updateEncoder(0); }
void IRAM_ATTR encoderISR1() { updateEncoder(1); }
void IRAM_ATTR encoderISR2() { updateEncoder(2); }
void IRAM_ATTR encoderISR3() { updateEncoder(3); }
void IRAM_ATTR encoderISR4() { updateEncoder(4); }
void IRAM_ATTR encoderISR5() { updateEncoder(5); }

typedef void (*ISRFunction)();
ISRFunction encoderISRs[6] = {encoderISR0, encoderISR1, encoderISR2, encoderISR3, encoderISR4, encoderISR5};

// --- Motor Control Functions ---
void stopMotor(int motorIndex) {
  ledcWrite(pwmPins[motorIndex], 0);
  motorMoving[motorIndex] = false;
}

void stopAllMotors() {
  for (int i = 0; i < 6; i++) {
    stopMotor(i);
  }
  Serial.println("All motors stopped");
}

void moveRelative(int motorIndex, long ticks) {
  motorTargets[motorIndex] = encoderCounts[motorIndex] + ticks;
  previousErrors[motorIndex] = 0;
  integrals[motorIndex] = 0;
  moveStartTimes[motorIndex] = millis();
  motorMoving[motorIndex] = true;
  
  Serial.print("Motor ");
  Serial.print(motorIndex);
  Serial.print(" moving relatively by ");
  Serial.print(ticks);
  Serial.print(" ticks to target: ");
  Serial.println(motorTargets[motorIndex]);
}

void updateMotorControl(int motorIndex) {
  if (!motorMoving[motorIndex]) return;
  
  if (millis() - moveStartTimes[motorIndex] > moveTimeout) {
    stopMotor(motorIndex);
    Serial.print("Motor ");
    Serial.print(motorIndex);
    Serial.println(" timeout! Movement failed.");
    return;
  }
  
  long error = motorTargets[motorIndex] - encoderCounts[motorIndex];
  
  if (abs(error) <= stopThreshold) {
    stopMotor(motorIndex);
    delay(50);
    long actualReached = encoderCounts[motorIndex];
    Serial.print("Motor ");
    Serial.print(motorIndex);
    Serial.print(" reached target: ");
    Serial.println(actualReached);
    return;
  }
  
  integrals[motorIndex] += error;
  long derivative = error - previousErrors[motorIndex];
  float output = (Kp[motorIndex] * error) + 
                 (Ki[motorIndex] * integrals[motorIndex]) + 
                 (Kd[motorIndex] * derivative);
  previousErrors[motorIndex] = error;
  
  int outputSpeed = abs(output);
  outputSpeed = constrain(outputSpeed, minSpeed, maxSpeed);
  
  digitalWrite(dirPins[motorIndex], error > 0 ? LOW : HIGH);
  ledcWrite(pwmPins[motorIndex], outputSpeed);
}

// --- Command Processing Functions ---
void parseCommand(String command) {
  int argCount = 0;
  int lastSpace = -1;
  command.trim();
  
  if (command.length() == 0) return;
  
  unsigned int cmdLength = command.length();
  for (unsigned int i = 0; i <= cmdLength; i++) {
    if (i == cmdLength || command.charAt(i) == ' ') {
      if (i > (unsigned int)(lastSpace + 1)) {
        cmdArgs[argCount] = command.substring(lastSpace + 1, i);
        argCount++;
        if (argCount >= MAX_ARGS) break;
      }
      lastSpace = i;
    }
  }
  
  if (argCount > 0) {
    String cmd = cmdArgs[0];
    
    if (cmd.equalsIgnoreCase("MOVE") && argCount > 2) {
      int motorIndex = cmdArgs[1].toInt();
      long absolutePosition = cmdArgs[2].toInt();
      
      if (motorIndex >= 0 && motorIndex < 6) {
        long currentPos = encoderCounts[motorIndex];
        long relativeMove = absolutePosition - currentPos;
        moveRelative(motorIndex, relativeMove);
      } else {
        Serial.println("Invalid motor index (0-5)");
      }
    }
    else if (cmd.equalsIgnoreCase("MOVEREL") && argCount > 2) {
      int motorIndex = cmdArgs[1].toInt();
      long ticks = cmdArgs[2].toInt();
      
      if (motorIndex >= 0 && motorIndex < 6) {
        moveRelative(motorIndex, ticks);
      } else {
        Serial.println("Invalid motor index (0-5)");
      }
    }
    else if (cmd.equalsIgnoreCase("MOVEABS") && argCount > 2) {
      int motorIndex = cmdArgs[1].toInt();
      long position = cmdArgs[2].toInt();
      
      if (motorIndex >= 0 && motorIndex < 6) {
        moveRelative(motorIndex, position - encoderCounts[motorIndex]);
      } else {
        Serial.println("Invalid motor index (0-5)");
      }
    }
    else if (cmd.equalsIgnoreCase("MOVEALL") && argCount == 7) {
      for (int i = 0; i < 6; i++) {
        long absoluteTarget = cmdArgs[i + 1].toInt();
        long currentPos = encoderCounts[i];
        long relativeMove = absoluteTarget - currentPos;
        moveRelative(i, relativeMove);
      }
    }
    else if (cmd.equalsIgnoreCase("MOVEALLREL") && argCount == 7) {
      for (int i = 0; i < 6; i++) {
        moveRelative(i, cmdArgs[i+1].toInt());
      }
    }
    else if (cmd.equalsIgnoreCase("MOVEALLABS") && argCount == 7) {
      for (int i = 0; i < 6; i++) {
        moveRelative(i, cmdArgs[i+1].toInt() - encoderCounts[i]);
      }
    }
    else if (cmd.equalsIgnoreCase("STOP")) {
      if (argCount > 1) {
        int motorIndex = cmdArgs[1].toInt();
        if (motorIndex >= 0 && motorIndex < 6) {
          stopMotor(motorIndex);
          Serial.print("Motor ");
          Serial.print(motorIndex);
          Serial.println(" stopped");
        }
      } else {
        stopAllMotors();
      }
    }
    else if (cmd.equalsIgnoreCase("ENC")) {
      for (int i = 0; i < 6; i++) {
        Serial.print(encoderCounts[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    else if (cmd.equalsIgnoreCase("RESET")) {
      if (argCount > 1) {
        int motorIndex = cmdArgs[1].toInt();
        if (motorIndex >= 0 && motorIndex < 6) {
          encoderCounts[motorIndex] = 0;
          lastEncoded[motorIndex] = (digitalRead(encoderAPins[motorIndex]) << 1) | digitalRead(encoderBPins[motorIndex]);
          Serial.print("Motor ");
          Serial.print(motorIndex);
          Serial.println(" encoder reset to 0");
        }
      } else {
        for (int i = 0; i < 6; i++) {
          encoderCounts[i] = 0;
          lastEncoded[i] = (digitalRead(encoderAPins[i]) << 1) | digitalRead(encoderBPins[i]);
        }
        Serial.println("All encoders reset to 0");
      }
    }
    else if (cmd.equalsIgnoreCase("SET") && argCount >= 7) {
      for (int i = 0; i < 6; i++) {
        int pwmValue = cmdArgs[i+1].toInt();
        Serial.print("Motor "); Serial.print(i);
        Serial.print(": Dir="); Serial.print(pwmValue >= 0 ? "HIGH" : "LOW");
        Serial.print(", PWM="); Serial.println(constrain(abs(pwmValue), 0, maxSpeed));
        digitalWrite(dirPins[i], pwmValue >= 0 ? HIGH : LOW);
        ledcWrite(pwmPins[i], constrain(abs(pwmValue), 0, maxSpeed));
        motorMoving[i] = false;
      }
      Serial.println("OK");
    }
    else if (cmd.equalsIgnoreCase("STATUS")) {
      for (int i = 0; i < 6; i++) {
        Serial.print("Motor ");
        Serial.print(i);
        Serial.print(": Pos=");
        Serial.print(encoderCounts[i]);
        Serial.print(", Moving=");
        Serial.print(motorMoving[i] ? "yes" : "no");
        if (motorMoving[i]) {
          Serial.print(", Target=");
          Serial.print(motorTargets[i]);
          Serial.print(", Error=");
          Serial.print(motorTargets[i] - encoderCounts[i]);
        }
        Serial.println();
      }
    }
    else if (cmd.equalsIgnoreCase("PID") && argCount == 5) {
      int motorIndex = cmdArgs[1].toInt();
      if (motorIndex >= 0 && motorIndex < 6) {
        Kp[motorIndex] = cmdArgs[2].toFloat();
        Ki[motorIndex] = cmdArgs[3].toFloat();
        Kd[motorIndex] = cmdArgs[4].toFloat();
        Serial.print("Motor ");
        Serial.print(motorIndex);
        Serial.print(" PID set to P=");
        Serial.print(Kp[motorIndex]);
        Serial.print(", I=");
        Serial.print(Ki[motorIndex]);
        Serial.print(", D=");
        Serial.println(Kd[motorIndex]);
      }
    }
    else if (cmd.equalsIgnoreCase("HELP")) {
      Serial.println("Available commands:");
      Serial.println("MOVE <motor> <ticks> - Move to absolute position");
      Serial.println("MOVEREL <motor> <ticks> - Explicit relative movement");
      Serial.println("MOVEABS <motor> <ticks> - Absolute position movement");
      Serial.println("MOVEALL <m0> <m1> <m2> <m3> <m4> <m5> - Move all to absolute positions");
      Serial.println("MOVEALLREL <m0> <m1> <m2> <m3> <m4> <m5> - Relative movement for all motors");
      Serial.println("MOVEALLABS <m0> <m1> <m2> <m3> <m4> <m5> - Absolute position movement for all motors");
      Serial.println("STOP [motor_index]");
      Serial.println("ENC - Get all encoder values");
      Serial.println("RESET [motor_index]");
      Serial.println("SET <m0> <m1> <m2> <m3> <m4> <m5> - Set PWM values");
      Serial.println("STATUS - Show motor status");
      Serial.println("PID <motor> <Kp> <Ki> <Kd> - Set PID constants");
      Serial.println("HELP - Show this message");
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      Serial.println("Type HELP for available commands");
    }
  }
}

// --- Setup Function ---
void setup() {
  Serial.begin(115200);
  delay(2000); // Longer delay for stability
  
  while (!Serial) {
    delay(10);
  }
  
  Serial.println();
  Serial.println("ESP32 Six Motor Control System Ready");
  
  // Initialize motor pins and PWM channels
  for (int i = 0; i < 6; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(encoderAPins[i], INPUT_PULLUP);
    pinMode(encoderBPins[i], INPUT_PULLUP);
    
    // Initialize lastEncoded for each encoder
    lastEncoded[i] = (digitalRead(encoderAPins[i]) << 1) | digitalRead(encoderBPins[i]);
    
    // Attach PWM pin with frequency and resolution
    ledcAttach(pwmPins[i], pwmFreq, pwmResolution);
    
    // Attach interrupts for both A and B channels
    attachInterrupt(digitalPinToInterrupt(encoderAPins[i]), encoderISRs[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderBPins[i]), encoderISRs[i], CHANGE);
  }
  
  Serial.println("Type HELP for available commands");
}

// --- Main Loop ---
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      parseCommand(input);
    }
  }
  
  for (int i = 0; i < 6; i++) {
    if (motorMoving[i]) {
      updateMotorControl(i);
    }
  }
  
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 1000) {
    lastStatusTime = millis();
    
    bool anyMotorMoving = false;
    for (int i = 0; i < 6; i++) {
      if (motorMoving[i]) {
        anyMotorMoving = true;
        break;
      }
    }
    
    if (anyMotorMoving) {
      for (int i = 0; i < 6; i++) {
        if (motorMoving[i]) {
          long error = motorTargets[i] - encoderCounts[i];
          Serial.print("Motor ");
          Serial.print(i);
          Serial.print(": Pos=");
          Serial.print(encoderCounts[i]);
          Serial.print(" Target=");
          Serial.print(motorTargets[i]);
          Serial.print(" Error=");
          Serial.print(error);
          Serial.println();
        }
      }
    }
  }
  
  delay(10);
}
