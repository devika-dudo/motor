// ESP32 Motor Control with PID and Encoder (using Quadrature Lookup Table Logic)

// Motor control pins
const int dirPin = 14; // Motor direction pin
const int pwmPin = 25; // PWM output pin
const int pwmFreq = 1000; // PWM frequency (Hz)
const int pwmResolution = 8; // PWM resolution (bits, 8-bit = 0-255)

// Encoder pins (must be interrupt-capable)
const int encoderPinA = 21; 
const int encoderPinB = 23;

// Encoder variables
volatile long encoderCount = 0;
volatile int lastEncoded = 0;

// PID control constants (tune these!)
float Kp = 1.5;
float Ki = 0.0;
float Kd = 0.5;
long previousError = 0;
float integral = 0;

// Movement control
const int maxSpeed = 255;
const int minSpeed = 60;
const int stopThreshold = 2;

// Lookup table for quadrature decoding
const int8_t lookupTable[16] = {
   0, -1, +1, 0,
  1, 0, 0, +1,
   -1, 0, 0, +1,
   0, 1, -1, 0
};

void IRAM_ATTR encoderISR() {
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  encoderCount += lookupTable[sum];
  lastEncoded = encoded;
}

void stopMotor() {
  ledcWrite(pwmPin, 0);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Motor Control with PID and Encoder Ready");

  pinMode(dirPin, OUTPUT);
  ledcAttach(pwmPin, pwmFreq, pwmResolution);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // Initial read of encoder
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);
  lastEncoded = (MSB << 1) | LSB;

  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISR, CHANGE);

  Serial.println("Commands: move <abs_pos>, reset, encode, stop, f, b, s");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("move ")) {
      long target = input.substring(5).toInt();
      if (abs(encoderCount - target) <= stopThreshold) {
        Serial.print("Already at target position: ");
        Serial.println(target);
      } else {
        moveTicks(target);
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
      digitalWrite(dirPin, HIGH);
      ledcWrite(pwmPin, 255);
      Serial.println("Moving Forward (manual)");
    } else if (input == "b") {
      digitalWrite(dirPin, LOW);
      ledcWrite(pwmPin, 255);
      Serial.println("Moving Backward (manual)");
    } else if (input == "s") {
      stopMotor();
      Serial.println("Stopped (manual)");
    } else {
      Serial.println("Commands: move <abs_pos>, reset, encode, stop, f, b, s");
    }
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Current encoder count: ");
    Serial.println(encoderCount);
    lastPrint = millis();
  }
}

void moveTicks(long target) {
  previousError = 0;
  integral = 0;

  Serial.print("Moving to position: ");
  Serial.println(target);

  unsigned long startTime = millis();
  const unsigned long moveTimeout = 10000;

  while (true) {
    if (millis() - startTime > moveTimeout) {
      stopMotor();
      Serial.println("Timeout! Movement failed.");
      break;
    }

    long error = target - encoderCount;
    if (abs(error) <= stopThreshold) {
      stopMotor();
      Serial.print("Reached target: ");
      Serial.println(encoderCount);
      break;
    }

    integral += error;
    long derivative = error - previousError;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previousError = error;

    int speed = constrain(abs(output), minSpeed, maxSpeed);
    digitalWrite(dirPin, error > 0 ? LOW : HIGH);
    ledcWrite(pwmPin, speed);

    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 200) {
      Serial.print("Pos: "); Serial.print(encoderCount);
      Serial.print(" | Target: "); Serial.print(target);
      Serial.print(" | Error: "); Serial.print(error);
      Serial.print(" | PWM: "); Serial.println(speed);
      lastDebug = millis();
    }

    delay(10);
  }
}
