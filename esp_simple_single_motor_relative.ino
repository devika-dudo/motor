// ESP32 Motor Control with PID and Encoder
// Motor control pins
const int dirPin = 33; // Motor direction pin
const int pwmPin = 25; // PWM output pin
const int pwmFreq = 1000; // PWM frequency (Hz)
const int pwmResolution = 8; // PWM resolution (bits, 8-bit = 0-255)

// Encoder pins (must be interrupt-capable)
const int encoderPinA = 34; 
const int encoderPinB = 35;

// Encoder variables
volatile long encoderCount = 0;

// PID control constants (tune these!)
float Kp = 1.5;
float Ki = 0.0;
float Kd = 0.5;
long previousError = 0;
float integral = 0;

// Movement control
const int maxSpeed = 255; // Max motor PWM
const int minSpeed = 60; // Minimum speed to overcome inertia
const int stopThreshold = 2; // How close to target before stopping
const bool INVERT_ENCODER_COUNTING = false;

// Improved encoder ISR - using same logic as Teensy code
void IRAM_ATTR encoderISR() {
  bool countUp = digitalRead(encoderPinB) == HIGH;
  if (INVERT_ENCODER_COUNTING) countUp = !countUp;
  encoderCount += countUp ? 1 : -1;
}

void stopMotor() {
  ledcWrite(pwmPin, 0);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Motor Control with PID Ready");
  
  // Motor pins setup
  pinMode(dirPin, OUTPUT);
  
  // Attach PWM pin with frequency and resolution
  ledcAttach(pwmPin, pwmFreq, pwmResolution);
  
  // Encoder pins setup with pullups
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  // Attach interrupt for encoder A pin on RISING edge (like Teensy code)
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);
  
  Serial.println("Commands: move <value>, reset, encode, stop");
  Serial.println("Example: move 100, move -50");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("move ")) {
      long steps = input.substring(5).toInt();
      if (steps != 0) {
        moveTicks(steps);
      } else {
        Serial.println("Invalid move value.");
      }
    } else if (input == "stop") {
      stopMotor();
      Serial.println("Motor stopped");
    } else if (input == "encode") {
      Serial.print("Encoder count: ");
      Serial.println(encoderCount);
    } else if (input == "reset") {
      encoderCount = 0;
      Serial.println("Encoder count reset to 0");
    } else if (input == "f") {
      // Keep original forward command for compatibility
      digitalWrite(dirPin, HIGH);
      ledcWrite(pwmPin, 255);
      Serial.println("Moving Forward (manual)");
    } else if (input == "b") {
      // Keep original backward command for compatibility
      digitalWrite(dirPin, LOW);
      ledcWrite(pwmPin, 255);
      Serial.println("Moving Backward (manual)");
    } else if (input == "s") {
      // Keep original stop command for compatibility
      stopMotor();
      Serial.println("Stopped (manual)");
    } else {
      Serial.println("Commands: move <value>, reset, encode, stop, f, b, s");
    }
  }
  
  // Print encoder count every 500ms (when not in PID control)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Current encoder count: ");
    Serial.println(encoderCount);
    lastPrint = millis();
  }
}

void moveTicks(long ticks) {
  long target = encoderCount + ticks;
  previousError = 0;
  integral = 0;
  
  Serial.print("Moving to: ");
  Serial.println(target);
  
  unsigned long startTime = millis();
  const unsigned long moveTimeout = 10000; // 10 second timeout
  
  while (true) {
    // Timeout check
    if (millis() - startTime > moveTimeout) {
      stopMotor();
      Serial.println("Timeout! Movement failed.");
      break;
    }
    
    long error = target - encoderCount;
    
    // Check if we've reached the target
    if (abs(error) <= stopThreshold) {
      stopMotor();
      Serial.print("Reached target: ");
      Serial.println(encoderCount);
      break;
    }
    
    // PID calculations
    integral += error;
    long derivative = error - previousError;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previousError = error;
    
    // Clamp output speed
    int outputSpeed = abs(output);
    outputSpeed = constrain(outputSpeed, minSpeed, maxSpeed);
    
    // Set direction
    digitalWrite(dirPin, error > 0 ? HIGH : LOW);
    
    // Apply speed using ESP32's ledcWrite
    ledcWrite(pwmPin, outputSpeed);
    
    // Debug info every 200ms
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 200) {
      Serial.print("Pos: ");
      Serial.print(encoderCount);
      Serial.print(" | Target: ");
      Serial.print(target);
      Serial.print(" | Error: ");
      Serial.print(error);
      Serial.print(" | PWM: ");
      Serial.print(outputSpeed);
      Serial.print(" | Output: ");
      Serial.print(output);
      Serial.print(" | Integral: ");
      Serial.print(integral);
      Serial.print(" | Derivative: ");
      Serial.println(derivative);
      lastDebugPrint = millis();
    }
    
    delay(10); // Control loop timing
  }
}
