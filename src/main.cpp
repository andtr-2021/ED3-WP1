/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-servo-motor-controlled-by-potentiometer
 */

// <Required>
// Include ESP32Servo
// Include HX711_ADC

#include <ESP32Servo.h>
#include <HX711_ADC.h>
#include <Arduino.h>

#define PIN_POTENTIOMETER 36 // ESP32 pin GPIO36 (ADC0) onnected to potentiometer
#define PIN_ESC         26 // ESP32 pin GPIO26 onnected to servo motor

#define V_SENSOR_PIN  34 // voltage sensor pin
#define C_SENSOR_PIN  32 // current sensor pin 


Servo esc;  // create servo object to control a servo

// Floats for ADC voltage & Input voltage
float adc_voltage = 0.0;
float in_voltage = 0.0;

 
// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0; 
 
// Float for Reference Voltage
// float ref_voltage = 25.2; //voltage from the battery
// default reference value of the sensor is 5V but I can modify it to 3.3 when I connnect with it
 float ref_voltage = 5;
// Integer for ADC value
int adc_value = 0;


#define LOADCELL_DT_PIN 16
#define LOADCELL_SCK_PIN 4

HX711_ADC scale(LOADCELL_DT_PIN, LOADCELL_SCK_PIN);

#define calibration_factor 430 // Depends on the load cell

volatile boolean newDataReady;

float loadcell_data; // measurement from the loadcell in gr

// serial configuration parameters
unsigned long curr_time = 0, prev_time = 0, dt = 50000; // time interval in us

// functions declarations
void dataReadyISR();
void SerialDataWrite();


void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  pinMode(V_SENSOR_PIN, INPUT);
  //Sensor setup
  

  // ESC & motor setup
  esc.attach(PIN_ESC); // (pin, min pulse width, max pulse width in microseconds)
  esc.writeMicroseconds(1000); // Initialize the ESC at minimum throttle (adjust if needed)
  delay(2000);          // Delay to allow the ESC to recognize the minimum throttle position


  while (!Serial);
  // Serial.println("Starting...");
  // Serial.println("Loadcell calibration, please wait.");
  scale.begin();
  scale.start(2000, true);
  scale.setCalFactor(calibration_factor);
  // Serial.println("Loadcell calibrated. Press any key to start the main program.");
  // while (!Serial.available());
  // Serial.read();
  pinMode(LOADCELL_DT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LOADCELL_DT_PIN), dataReadyISR, FALLING);

}

void loop() {
  // potentiometer control
  // reads the value of the potentiometer (value between 0 and 4095)
  int analogValue = analogRead(PIN_POTENTIOMETER);
  // scales it to use it with the servo (value between 0 and 180) // max: 2000 but We set it 1350 as for safety reason
  int throttleValue = map(analogValue, 0, 4095, 1160, 1350);
  // angle of the potentiometer
  int angle = map(analogValue, 0, 4095, 0, 270);
  // sets the servo position according to the scaled value
  esc.writeMicroseconds(throttleValue);

  // motor value display
  // print out the value
  Serial.print(" Analog value: ");
  Serial.print(analogValue);
  Serial.print(" => angle: ");
  Serial.print(angle);
  Serial.print(" => throttleValue: ");
  Serial.println(throttleValue);


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

  // Voltage Sensor control
  adc_value = analogRead(V_SENSOR_PIN);   // read the state of the the input pin:
  adc_voltage  = (adc_value * ref_voltage) / 4096.0;  // Determine voltage at ADC input // analog read resolution 10 bi(1025)? 12bit(4096)? 
  in_voltage = adc_voltage / (R2/(R1+R2)) ;    // Calculate voltage at divider input

  Serial.print("adc Voltage = ");
  Serial.print(adc_value);
   
  //Print results to Serial Monitor to 2 decimal places
  Serial.print(" Input Voltage = ");
  Serial.println(in_voltage, 2);
   delay(1000);
}

void dataReadyISR()
{
  if (scale.update())
    newDataReady = 1;
}

void SerialDataWrite()
{
  Serial.print("Loadcell value: ");
  Serial.print(curr_time / 1000);
  Serial.print(",");
  Serial.print(loadcell_data);
  Serial.println("");
}