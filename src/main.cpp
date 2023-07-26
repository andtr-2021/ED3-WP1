#include <Arduino.h>
#include <HX711.h>

const int LOADCELL_DOUT_PIN = 16; // Change this to your ESP32 GPIO pin for HX711 data (DT)
const int LOADCELL_SCK_PIN = 4;  // Change this to your ESP32 GPIO pin for HX711 clock (SCK)

HX711 scale;

void setup() {
  Serial.begin(9600);
  Serial.println("HX711 Weight Measurement");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(350);   // Set this to the calibration factor of your load cell (read below for calibration)
  scale.tare();
}

void loop() {
  float weight = scale.get_units(); // Get weight in units (adjust with calibration factor)
  Serial.print("Weight: ");
  Serial.print(weight);
  Serial.println(" grams");

  delay(1000); // Wait for a second before reading again
}