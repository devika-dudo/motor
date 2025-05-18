const int dirPins[6] = {9, 11, 24, 28, 37, 35};  // Direction control pins
const int pwmPins[6] = {10, 12, 25, 29, 36, 33}; // PWM control pins

float jointVals[6];

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 6; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(pwmPins[i], OUTPUT);
  }
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int idx = 0;
    char* ptr = strtok((char*)input.c_str(), " ");
    while (ptr != NULL && idx < 6) {
      jointVals[idx] = atof(ptr);
      ptr = strtok(NULL, " ");
      idx++;
    }

    // Control motors based on jointVals
    for (int i = 0; i < 6; i++) {
      if (jointVals[i] >= 0) {
        digitalWrite(dirPins[i], HIGH);
      } else {
        digitalWrite(dirPins[i], LOW);
      }
      int pwmVal = (int)(fabs(jointVals[i]) * 255);
      analogWrite(pwmPins[i], pwmVal);
    }
  }
}
