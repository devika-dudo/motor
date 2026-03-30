#include <Encoder.h>

// ================== PIN DEFINITIONS ==================
#define ENC_A 17
#define ENC_B 18

Encoder encoder(ENC_A, ENC_B);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Encoder ready. Rotate motor.");
}

void loop() {
  long ticks = encoder.read();
  Serial.println(ticks);
  delay(50);
}
