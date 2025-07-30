// --- Pin Definitions ---
// Motor control pins (direction and PWM)
const int dirPins[6] = {2, 4, 16, 17, 18, 25};  // Direction control pins
const int pwmPins[6] = {5, 23, 25, 26, 32, 26}; // PWM control pins

// PWM settings
const int pwmFreq = 1000;     // PWM frequency in Hz
const int pwmResolution = 8;  // 8-bit resolution (0-255)

void setup() {
  delay(2000); // Wait for serial to stabilize
  Serial.begin(115200); // Higher baud rate for ESP32
  while (!Serial) {
    delay(100); // Wait for serial connection
  }
  
  // Initialize direction pins and PWM
  for (int i = 0; i < 6; i++) {
    pinMode(dirPins[i], OUTPUT);
    
    // Attach PWM pin with frequency and resolution
    ledcAttach(pwmPins[i], pwmFreq, pwmResolution);
    
    // Initialize motors to stopped state
    digitalWrite(dirPins[i], HIGH);
    ledcWrite(pwmPins[i], 0);
  }
  
  Serial.println("ESP32 6-Motor Controller Ready");
  Serial.println("Send commands like: SET 100 -150 200 0 -75 255");
}

void loop() {
  // Check if data is available in the serial buffer
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Read until newline
    input.trim(); // Remove whitespace
    
    if (input.startsWith("SET")) {
      // Parse the PWM values
      int pwmValues[6] = {0, 0, 0, 0, 0, 0}; // Initialize PWM values
      int idx = 0;
      
      // Skip "SET " and tokenize
      int startPos = input.indexOf(' ') + 1;
      String values = input.substring(startPos);
      
      // Parse space-separated values
      int pos = 0;
      while (pos < values.length() && idx < 6) {
        int nextSpace = values.indexOf(' ', pos);
        if (nextSpace == -1) nextSpace = values.length();
        
        String valueStr = values.substring(pos, nextSpace);
        pwmValues[idx] = valueStr.toInt();
        
        pos = nextSpace + 1;
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
        ledcWrite(pwmPins[i], pwmVal);
        
        // Debug output
        if (pwmVal > 0) {
          Serial.print("Motor ");
          Serial.print(i);
          Serial.print(" - DIR pin ");
          Serial.print(dirPins[i]);
          Serial.print(": ");
          Serial.print(pwmValues[i] >= 0 ? "HIGH" : "LOW");
          Serial.print(", PWM pin ");
          Serial.print(pwmPins[i]);
          Serial.print(": ");
          Serial.println(pwmVal);
        }
      }
      
      // Send confirmation
      Serial.print("Motors set to: ");
      for (int i = 0; i < 6; i++) {
        Serial.print(pwmValues[i]);
        if (i < 5) Serial.print(" ");
      }
      Serial.println();
    }
    else if (input.equals("f")) {
      // Test forward - motor 5 (index 5) at full speed
      digitalWrite(dirPins[5], HIGH);
      ledcWrite(pwmPins[5], 255);
      Serial.println("Motor 5 forward at full speed (test)");
    }
    else if (input.equals("b")) {
      // Test backward - motor 5 (index 5) at full speed  
      digitalWrite(dirPins[5], LOW);
      ledcWrite(pwmPins[5], 255);
      Serial.println("Motor 5 backward at full speed (test)");
    }
    else if (input.equals("s")) {
      // Test stop - motor 5 (index 5)
      ledcWrite(pwmPins[5], 0);
      Serial.println("Motor 5 stopped (test)");
    }
    else if (input.equals("STOP")) {
      // Emergency stop all motors
      for (int i = 0; i < 6; i++) {
        ledcWrite(pwmPins[i], 0);
      }
      Serial.println("All motors stopped");
    }
    else if (input.equals("STATUS")) {
      // Print current status
      Serial.println("ESP32 6-Motor Controller Status: OK");
    }
    else {
      Serial.println("Unknown command. Use SET, STOP, or STATUS");
    }
  }
}
