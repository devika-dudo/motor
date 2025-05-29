// --- Pin Definitions ---
// Motor control pins (direction and PWM)
const int dirPins[6] = {1, 3, 5, 7, 9, 11};  // Direction control pins
const int pwmPins[6] = {2, 4, 6, 8, 10, 12}; // PWM control pins  

// Encoder pins (A channels must be interrupt-capable)
const int encoderAPins[6] = {27, 25, 40, 13, 15, 17}; // Interrupt pins (must be interrupt-capable pins)
const int encoderBPins[6] = {24, 26, 39, 41, 14, 16}; // Digital pins

volatile long encoderCounts[6] = {0, 0, 0, 0, 0, 0};
volatile int lastEncoded[6] = {0, 0, 0, 0, 0, 0};

const int8_t lookupTable[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

void updateEncoder(int motorIndex) {
    int MSB = digitalRead(encoderAPins[motorIndex]);
    int LSB = digitalRead(encoderBPins[motorIndex]);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded[motorIndex] << 2) | encoded;
    int increment = lookupTable[sum];
    encoderCounts[motorIndex] += increment;
    lastEncoded[motorIndex] = encoded;
}

void encoderISR0() { updateEncoder(0); }
void encoderISR1() { updateEncoder(1); }
void encoderISR2() { updateEncoder(2); }
void encoderISR3() { updateEncoder(3); }
void encoderISR4() { updateEncoder(4); }
void encoderISR5() { updateEncoder(5); }

typedef void (*ISRFunction)();
ISRFunction encoderISRs[6] = {encoderISR0, encoderISR1, encoderISR2, encoderISR3, encoderISR4, encoderISR5};

// Function to move all motors with different PWM values
void moveAllMotors(int pwmValues[6]) {
  for (int i = 0; i < 6; i++) {
    digitalWrite(dirPins[i], pwmValues[i] >= 0 ? HIGH : LOW);
    analogWrite(pwmPins[i], abs(pwmValues[i])); // Use absolute value for PWM
  }
}

void stopAllMotors() {
  for (int i = 0; i < 6; i++) {
    analogWrite(pwmPins[i], 0);
  }
}

void printEncoders() {
  Serial.print("Encoders: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(encoderCounts[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  for (int i=0; i<6; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(pwmPins[i], OUTPUT);
    pinMode(encoderAPins[i], INPUT_PULLUP);
    pinMode(encoderBPins[i], INPUT_PULLUP);
    lastEncoded[i] = (digitalRead(encoderAPins[i]) << 1) | digitalRead(encoderBPins[i]);
    attachInterrupt(digitalPinToInterrupt(encoderAPins[i]), encoderISRs[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderBPins[i]), encoderISRs[i], CHANGE);
  }
}

void loop() {
  int forwardPwmValues[6] = {100, 100, 100, 100, 50, 50}; // Different PWM values for forward movement
  moveAllMotors(forwardPwmValues);   // Move forward with specified PWM values
  unsigned long start = millis();
  while (millis() - start < 1000) {  // Run for 1 second
    printEncoders(); 
    delay(100);
  }

  int backwardPwmValues[6] = {-100, -100, -100, -100, -50, -50}; // Different PWM values for backward movement
  moveAllMotors(backwardPwmValues);  // Move backward with specified PWM values
  start = millis();
  while (millis() - start < 1000) {  // Run for 1 second
    printEncoders();
    delay(100);
  }

  stopAllMotors();
  while(1) {
    printEncoders();
    delay(1000);  // print encoder positions every second while stopped
  }
}
