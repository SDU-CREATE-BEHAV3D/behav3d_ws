#include <Controllino.h>

unsigned long lastPrintMs = 0;
int lastDi0State = -1;

void setup() {
  pinMode(CONTROLLINO_DI0, INPUT);
  Serial.begin(115200);
  delay(500);
  Serial.println("CONTROLLINO smoke test boot");
  Serial.println("Monitoring DI0");
}

void loop() {
  unsigned long now = millis();
  int di0State = digitalRead(CONTROLLINO_DI0);

  if (di0State != lastDi0State) {
    lastDi0State = di0State;
    Serial.print("DI0 changed -> ");
    Serial.println(di0State == HIGH ? "HIGH" : "LOW");
  }

  if (now - lastPrintMs >= 1000UL) {
    lastPrintMs = now;
    Serial.print("alive ms=");
    Serial.print(now);
    Serial.print(" DI0=");
    Serial.println(di0State == HIGH ? "HIGH" : "LOW");
  }
}
