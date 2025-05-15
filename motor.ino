// Simple Encoder Motor Control for Teensy with PID
// Motor and Encoder Pins for Teensy
const int dir1 = 9; // Direction control pin
const int pwm1 = 10; // Speed control pin
const int encoderA = 1; // Interrupt pin
const int encoderB = 2; // Any digital pin
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
// ISR for encoder
void encoderISR() {
bool countUp = digitalRead(encoderB) == HIGH;
if (INVERT_ENCODER_COUNTING) countUp = !countUp;
 encoderCount += countUp ? 1 : -1;
}
void stopMotor() {
analogWrite(pwm1, 0);
}
void setup() {
pinMode(dir1, OUTPUT);
pinMode(pwm1, OUTPUT);
pinMode(encoderA, INPUT_PULLUP);
pinMode(encoderB, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, RISING);
Serial.begin(9600);
while (!Serial);
Serial.println("Motor Control System Ready");
Serial.println("Commands: move <value>, reset, encode, stop");
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
} else {
Serial.println("Commands: move <value>, reset, encode, stop");
}
}
}
void moveTicks(long ticks) {
long target = encoderCount + ticks;
 previousError = 0;
 integral = 0;
Serial.print("Moving to: ");
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
 // PID calculations
 integral += error;
long derivative = error - previousError;
float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
 previousError = error;
 // Clamp output speed
int outputSpeed = abs(output);
 outputSpeed = constrain(outputSpeed, minSpeed, maxSpeed);
 // Set direction
digitalWrite(dir1, error > 0 ? HIGH : LOW);
 // Apply speed
analogWrite(pwm1, outputSpeed);
 // Debug info every 200ms
static unsigned long lastPrint = 0;
if (millis() - lastPrint > 200) {
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

 lastPrint = millis();
}
delay(10); // Control loop timing
}
}
