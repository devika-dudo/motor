// --- Pin Definitions ---
// Motor control pins (direction and PWM)
const int dirPins[6] = {1, 3, 5, 7, 9, 11};  // Direction control pins
const int pwmPins[6] = {2, 4, 6, 8, 10, 12}; // PWM control pins  

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  for (int i = 0; i < 6; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(pwmPins[i], OUTPUT);
  }
}

void loop() {
  // Check if data is available in the serial buffer
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Read until newline
    if (input.startsWith("SET")) {
      // Parse the PWM values
      int pwmValues[6] = {0, 0, 0, 0, 0, 0}; // Initialize PWM values
      int idx = 0;
      char* ptr = strtok((char*)input.c_str() + 4, " "); // Skip "SET " and tokenize
      while (ptr != NULL && idx < 6) {
        pwmValues[idx] = atoi(ptr); // Convert to integer
        ptr = strtok(NULL, " ");
        idx++;
      }

      // Control motors based on pwmValues
      for (int i = 0; i < 6; i++) {
        // Set direction based on PWM value
        if (pwmValues[i] >= 0) {
          digitalWrite(dirPins[i], HIGH); // Forward
        } else {
          digitalWrite(dirPins[i], LOW);  // Backward
        }
        // Set PWM value (constrain to 0-255)
        int pwmVal = constrain(abs(pwmValues[i]), 0, 255);
        analogWrite(pwmPins[i], pwmVal);
      }
    }
  }
}
