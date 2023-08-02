#include <Arduino.h>
#include <HX711_ADC.h>

#define LOADCELL_DT_PIN 16
#define LOADCELL_SCK_PIN 4

HX711_ADC scale(LOADCELL_DT_PIN, LOADCELL_SCK_PIN);

#define calibration_factor 460 // Depends on the load cell

volatile boolean newDataReady;

float loadcell_data; // measurement from the loadcell in gr

// serial configuration parameters
unsigned long curr_time = 0, prev_time = 0, dt = 50000; // time interval in us

// functions declarations
void dataReadyISR();
void SerialDataWrite();

void setup()
{
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Loadcell calibration, please wait.");
  scale.begin();
  scale.start(2000, true);
  scale.setCalFactor(calibration_factor);
  Serial.println("Loadcell calibrated. Press any key to start the main program.");
  while (!Serial.available());
  Serial.read();
  pinMode(LOADCELL_DT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LOADCELL_DT_PIN), dataReadyISR, FALLING);
}

void loop()
{
  if (newDataReady)
  {
    newDataReady = 0;
    loadcell_data = scale.getData();
  }

  curr_time = micros();
  if (curr_time - prev_time >= dt)
  {
    prev_time += dt;
    SerialDataWrite();
  }
}

// fuctions definitions

void dataReadyISR()
{
  if (scale.update())
    newDataReady = 1;
}

void SerialDataWrite()
{
  Serial.print(curr_time / 1000);
  Serial.print(",");
  Serial.print(loadcell_data);
  Serial.println("");
}