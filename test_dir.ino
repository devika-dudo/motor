// --- Pin Definitions ---
const int dirPins[6]    = {1, 3, 5, 7, 9, 11};      // Direction pins
const int pwmPins[6]    = {2, 4, 6, 8, 10, 12};     // PWM pins

const int testSpeed = 150;  // Set PWM speed (0 to 255)
const int runTime = 2000;   // Run time in ms (2 seconds)

void setup() {
  Serial.begin(9600);

  // Set all pins as OUTPUT
  for (int i = 0; i < 6; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(pwmPins[i], OUTPUT);
  }

  Serial.println("Starting motor direction test...");
}

void loop() {
  for (int i = 0; i < 6; i++) {
    Serial.print("Testing Motor ");
    Serial.println(i);

    // Forward direction (e.g., LOW on dir pin)
    Serial.println("Forward");
    digitalWrite(dirPins[i], LOW);
    analogWrite(pwmPins[i], testSpeed);
    delay(runTime);

    // Stop
    analogWrite(pwmPins[i], 0);
    delay(500);

    // Reverse direction (e.g., HIGH on dir pin)
    Serial.println("Reverse");
    digitalWrite(dirPins[i], HIGH);
    analogWrite(pwmPins[i], testSpeed);
    delay(runTime);

    // Stop
    analogWrite(pwmPins[i], 0);
    delay(1000);  // Pause before next motor
  }

  // Stop looping
  while (true);  // Stop after one full cycle
}