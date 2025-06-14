// ESP32 Six Motor Encoder Control with PID
// Supports interface for external control via serial commands

// --- Pin Definitions ---
// Motor control pins (direction and PWM)
// --- Pin Definitions ---
// Motor control pins (direction and PWM) - Fixed pin assignments
const int dirPins[6] = {19, 26, 27, 14, 12, 13};     // Direction control pins
const int pwmPins[6] = {18, 25, 33, 32, 5, 22};     // PWM control pins - NO CONFLICTS

// Encoder pins - FIXED assignments with no conflicts
// A channels must be interrupt-capable, avoiding input-only pins
const int encoderAPins[6] = {2, 4, 16, 21, 17, 23};   // Interrupt-capable pins
const int encoderBPins[6] = {15, 34, 35, 23, 22, 0}; // Digital pins (34-39 are input-only, that's OK for B pins)


// --- PWM Configuration ---
const int pwmFreq = 1000;      // PWM frequency (Hz)
const int pwmResolution = 8;   // PWM resolution (bits, 8-bit = 0-255)
const int pwmChannels[6] = {0, 1, 2, 3, 4, 5}; // PWM channels for each motor

// --- Motor and Encoder Variables ---
volatile long encoderCounts[6] = {0, 0, 0, 0, 0, 0};
bool invertEncoderCounting[6] = {false, false, true, false, false, false};

// --- PID Control Constants ---
// These should be tuned for your specific motors and application
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
// Motor status flags (1=moving, 0=stopped)
bool motorMoving[6] = {false, false, false, false, false, false};
// Start time for each motor movement
unsigned long moveStartTimes[6] = {0, 0, 0, 0, 0, 0};

// --- Command Processing ---
String inputString = "";        // String to hold incoming data
boolean stringComplete = false; // Whether the string is complete
const int MAX_ARGS = 10;        // Maximum number of arguments in a command
String cmdArgs[MAX_ARGS];       // Array to hold command arguments

// --- Forward Declarations ---
void IRAM_ATTR updateEncoder(int motorIndex);

// --- ISR Functions for Encoders ---
void IRAM_ATTR encoderISR0() { updateEncoder(0); }
void IRAM_ATTR encoderISR1() { updateEncoder(1); }
void IRAM_ATTR encoderISR2() { updateEncoder(2); }
void IRAM_ATTR encoderISR3() { updateEncoder(3); }
void IRAM_ATTR encoderISR4() { updateEncoder(4); }
void IRAM_ATTR encoderISR5() { updateEncoder(5); }

void IRAM_ATTR updateEncoder(int motorIndex) {
  bool countUp = digitalRead(encoderBPins[motorIndex]) == HIGH;
  if (invertEncoderCounting[motorIndex]) countUp = !countUp;
  encoderCounts[motorIndex] += countUp ? -1 : 1;
}

// Function pointer array for ISRs
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

// Modified function to handle relative movements
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
  
  // Check for timeout
  if (millis() - moveStartTimes[motorIndex] > moveTimeout) {
    stopMotor(motorIndex);
    Serial.print("Motor ");
    Serial.print(motorIndex);
    Serial.println(" timeout! Movement failed.");
    return;
  }
  
  long error = motorTargets[motorIndex] - encoderCounts[motorIndex];
  
  // Check if we've reached the target
  if (abs(error) <= stopThreshold) {
    stopMotor(motorIndex);
    delay(50);  // Allow physical system to settle
    long actualReached = encoderCounts[motorIndex];
    Serial.print("Motor ");
    Serial.print(motorIndex);
    Serial.print(" reached target: ");
    Serial.println(actualReached);
    return;
  }
  
  // PID calculations
  integrals[motorIndex] += error;
  long derivative = error - previousErrors[motorIndex];
  float output = (Kp[motorIndex] * error) + 
                 (Ki[motorIndex] * integrals[motorIndex]) + 
                 (Kd[motorIndex] * derivative);
  previousErrors[motorIndex] = error;
  
  // Clamp output speed
  int outputSpeed = abs(output);
  outputSpeed = constrain(outputSpeed, minSpeed, maxSpeed);
  
  // Set direction
  digitalWrite(dirPins[motorIndex], error > 0 ? LOW : HIGH);
  
  // Apply speed using ESP32's ledcWrite
  ledcWrite(pwmPins[motorIndex], outputSpeed);
}

// --- Command Processing Functions ---
void parseCommand(String command) {
  // Split the command into space-separated arguments
  int argCount = 0;
  int lastSpace = -1;
  
  command.trim();
  
  // Handle empty command
  if (command.length() == 0) return;
  
  // Split command into arguments
  unsigned int cmdLength = command.length();
  for (unsigned int i = 0; i <= cmdLength; i++) {
    if (i == cmdLength || command.charAt(i) == ' ') {
      if (i > (unsigned int)(lastSpace + 1)) {  // Avoid empty arguments
        cmdArgs[argCount] = command.substring(lastSpace + 1, i);
        argCount++;
        if (argCount >= MAX_ARGS) break;
      }
      lastSpace = i;
    }
  }
  
  // Process commands
  if (argCount > 0) {
    String cmd = cmdArgs[0];
    
    if (cmd.equalsIgnoreCase("MOVE") && argCount > 2) {
      // MOVE <motor_index> <position>
      int motorIndex = cmdArgs[1].toInt();
      long absolutePosition = cmdArgs[2].toInt();
      
      if (motorIndex >= 0 && motorIndex < 6) {
        long currentPos = encoderCounts[motorIndex];  // current encoder count
        long relativeMove = absolutePosition - currentPos;  // compute difference

        moveRelative(motorIndex, relativeMove);  // move by difference
      } else {
        Serial.println("Invalid motor index (0-5)");
      }
    }
    else if (cmd.equalsIgnoreCase("MOVEREL") && argCount > 2) {
      // MOVEREL <motor_index> <ticks> - Explicit relative movement
      int motorIndex = cmdArgs[1].toInt();
      long ticks = cmdArgs[2].toInt();
      
      if (motorIndex >= 0 && motorIndex < 6) {
        moveRelative(motorIndex, ticks);
      } else {
        Serial.println("Invalid motor index (0-5)");
      }
    }
    else if (cmd.equalsIgnoreCase("MOVEABS") && argCount > 2) {
      // MOVEABS <motor_index> <position> - Explicit absolute movement
      int motorIndex = cmdArgs[1].toInt();
      long position = cmdArgs[2].toInt();
      
      if (motorIndex >= 0 && motorIndex < 6) {
        moveRelative(motorIndex, position - encoderCounts[motorIndex]); // Move to absolute position
      } else {
        Serial.println("Invalid motor index (0-5)");
      }
    }
    else if (cmd.equalsIgnoreCase("MOVEALL") && argCount == 7) {
      // MOVEALL <pos1> <pos2> <pos3> <pos4> <pos5> <pos6>
      for (int i = 0; i < 6; i++) {
        long absoluteTarget = cmdArgs[i + 1].toInt();  // Desired absolute position for joint i
        long currentPos = encoderCounts[i];            // Current encoder count for joint i
        long relativeMove = absoluteTarget - currentPos; // Calculate relative movement required

        moveRelative(i, relativeMove);                  // Move joint i relatively by difference
      }
    }
    else if (cmd.equalsIgnoreCase("MOVEALLREL") && argCount == 7) {
      // MOVEALLREL <ticks1> <ticks2> <ticks3> <ticks4> <ticks5> <ticks6>
      for (int i = 0; i < 6; i++) {
        moveRelative(i, cmdArgs[i+1].toInt());
      }
    }
    else if (cmd.equalsIgnoreCase("MOVEALLABS") && argCount == 7) {
      // MOVEALLABS <pos1> <pos2> <pos3> <pos4> <pos5> <pos6>
      for (int i = 0; i < 6; i++) {
        moveRelative(i, cmdArgs[i+1].toInt() - encoderCounts[i]); // Move to absolute position
      }
    }
    else if (cmd.equalsIgnoreCase("STOP")) {
      if (argCount > 1) {
        // STOP <motor_index>
        int motorIndex = cmdArgs[1].toInt();
        if (motorIndex >= 0 && motorIndex < 6) {
          stopMotor(motorIndex);
          Serial.print("Motor ");
          Serial.print(motorIndex);
          Serial.println(" stopped");
        }
      } else {
        // STOP (all motors)
        stopAllMotors();
      }
    }
    else if (cmd.equalsIgnoreCase("ENC")) {
      // Return all encoder values
      for (int i = 0; i < 6; i++) {
        Serial.print(encoderCounts[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    else if (cmd.equalsIgnoreCase("RESET")) {
      if (argCount > 1) {
        // RESET <motor_index>
        int motorIndex = cmdArgs[1].toInt();
        if (motorIndex >= 0 && motorIndex < 6) {
          encoderCounts[motorIndex] = 0;
          Serial.print("Motor ");
          Serial.print(motorIndex);
          Serial.println(" encoder reset to 0");
        }
      } else {
        // RESET (all encoders)
        for (int i = 0; i < 6; i++) {
          encoderCounts[i] = 0;
        }
        Serial.println("All encoders reset to 0");
      }
    }
    else if (cmd.equalsIgnoreCase("SET") && argCount >= 7) {
  for (int i = 0; i < 6; i++) {
    int pwmValue = cmdArgs[i+1].toInt();
    Serial.print("Motor "); Serial.print(i); // Debug
    Serial.print(": Dir="); Serial.print(pwmValue >= 0 ? "HIGH" : "LOW");
    Serial.print(", PWM="); Serial.println(constrain(abs(pwmValue), 0, maxSpeed));
    digitalWrite(dirPins[i], pwmValue >= 0 ? HIGH : LOW);
    ledcWrite(pwmPins[i], constrain(abs(pwmValue), 0, maxSpeed));
    motorMoving[i] = false;
  }
  Serial.println("OK");
}
    else if (cmd.equalsIgnoreCase("STATUS")) {
      // Print status information for all motors
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
      // PID <motor_index> <Kp> <Ki> <Kd>
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
      // Print available commands
      Serial.println("Available commands:");
      Serial.println("MOVE <motor_index> <position> - Move to absolute position");
      Serial.println("MOVEREL <motor_index> <ticks> - Explicit relative movement");
      Serial.println("MOVEABS <motor_index> <position> - Absolute position movement");
      Serial.println("MOVEALL <pos1> <pos2> <pos3> <pos4> <pos5> <pos6> - Move all to absolute positions");
      Serial.println("MOVEALLREL <ticks1> <ticks2> <ticks3> <ticks4> <ticks5> <ticks6> - Relative movement for all motors");
      Serial.println("MOVEALLABS <pos1> <pos2> <pos3> <pos4> <pos5> <pos6> - Absolute position movement for all motors");
      Serial.println("STOP [motor_index]");
      Serial.println("ENC - Get all encoder values");
      Serial.println("RESET [motor_index]");
      Serial.println("SET <pwm1> <pwm2> <pwm3> <pwm4> <pwm5> <pwm6>");
      Serial.println("STATUS - Show motor status");
      Serial.println("PID <motor_index> <Kp> <Ki> <Kd>");
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
  delay(2000);  // Longer delay for stability
  
  // Wait for serial port to connect
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
    
    // Attach PWM pin with frequency and resolution
    ledcAttach(pwmPins[i], pwmFreq, pwmResolution);
    
    // Attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(encoderAPins[i]), encoderISRs[i], RISING);
  }
  
  Serial.println("Type HELP for available commands");
}

// --- Main Loop ---
void loop() {
  // Check for serial input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      parseCommand(input);
    }
  }
  
  // Update motor control for all motors
  for (int i = 0; i < 6; i++) {
    if (motorMoving[i]) {
      updateMotorControl(i);
    }
  }
  
  // Print debug information every 1000ms
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 1000) {
    lastStatusTime = millis();
    
    // Only print status if at least one motor is moving
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
  
  delay(10); // Control loop timing
}
