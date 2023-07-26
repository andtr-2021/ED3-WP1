#define INTERNAL_LED 12
#include <Arduino.h>

void setup() {
  // Set pin mode
  pinMode(INTERNAL_LED,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(INTERNAL_LED,HIGH);
  Serial.print("LED ON");
  delay(1000);
  digitalWrite(INTERNAL_LED,LOW);
  Serial.print("LED OFF");
  delay(1000);
} 