/**
 * DSC_v2: UI and control systems for prototype DSC system
 * Copyright (C) 2020  Christian Kunis
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>
 *
 * You may contact the author at ckunis.contact@gmail.com
 */

#include <Adafruit_NeoPixel.h>

// NeoPixel parameters
#define LED_PIN 8
#define LED_COUNT 1
Adafruit_NeoPixel neopixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Predefined colors for NeoPixel
const uint32_t black = neopixel.Color(0, 0, 0);
const uint32_t white = neopixel.Color(255, 255, 255);
const uint32_t magenta = neopixel.Color(255, 0, 255);
const uint32_t red = neopixel.Color(255, 0, 0);
const uint32_t yellow = neopixel.Color(255, 255, 0);
const uint32_t green = neopixel.Color(0, 255, 0);
const uint32_t cyan = neopixel.Color(0, 255, 255);
const uint32_t blue = neopixel.Color(0, 0, 255);

// Feather M0 Express board pinouts
#define REF_TEMP_PROBE_PIN A1
#define REF_CURRENT_SENS_PIN A2
#define SAMP_CURRENT_SENS_PIN A3
#define SAMP_TEMP_PROBE_PIN A4
#define Ref_Heater_PIN 11
#define Samp_Heater_PIN 10

/**
 * Number of samples to average the reading over. Change this to make the
 * reading smoother... but beware of buffer overflows!
 *
 * ! Must NOT exceed 2^(32 - 2*ANALOG_RESOLUTION) to prevent overflow error
 * during summation
 *
 * For 10-bit analog res, this max is 4096 samples.
 *
 * For 12-bit analog res, this max is 256 samples.
 */
#define AVG_SAMPLES 200

// Wait 2 milliseconds before the next loop for the analog-to-digital converter
// to settle after the last reading
#define AVG_SAMPLE_DELAY 2 // Sample delay in milliseconds

// global variable for holding the raw analog sensor values
unsigned long sensorValues[4];

// The max voltage of analog input readings
#define ANALOG_REF_VOLTAGE 3.3
// The sample resolution of the analogRead() output
#define ANALOG_RESOLUTION 12
const unsigned long analogMidpoint = pow(2, ANALOG_RESOLUTION - 1);
// Analog signal to voltage conversion factor
const double byteToVolts = (ANALOG_REF_VOLTAGE / pow(2, ANALOG_RESOLUTION));
const double byteToMillivolts = 1000.0 * byteToVolts;

// Thermocouple amplifier conversion constants
#define AMPLIFIER_VOLTAGE_OFFSET 1250.0 // 1250 mV = 1.25 V
#define AMPLIFIER_CONVERSION_FACTOR 5.0 // 5 mV/C = 0.005 V/C

// Current sensor conversion constants
#define CURRENT_SENSOR_SENS 0.4 // Sensitivity (Sens) 100mA per 250mV = 0.4
// #define CURRENT_SENSOR_VREF 1650.0 // Output voltage with no current: ~ 1650mV or 1.65V
#define CURRENT_SENSOR_VREF 0.0 // VRef = {Output voltate with no current} - ANALOG_REF/2
// VREF is now accounted for by `sensorValue - analogMidpoint` during the RMS
// calculation
double Ref_Current_Sensor_Sens = 0.4, Samp_Current_Sensor_Sens = 0.4;
double Ref_Current_Sensor_VRef = 0.0, Samp_Current_Sensor_VRef = 0.0;

// The constant voltage supplied to the heating coils
#define HEATING_COIL_VOLTAGE 24.0 // Theoretical 24 VAC
// I recommend measuring the real-world voltage across the resistor and
// adjusting this value accordingly

// The resistance of the heating coil circuit (Ohms, not kilo-Ohms)
#define HEATING_COIL_RESISTANCE 49.0
// For best accuracy during sensor calibration, this value should be measured as
// the total resistance around the heating coil circuit, not just the resistance
// of the heating components alone.

// Max allowable temperature. If the either temperature exceeds this value, the
// PWM duty cycle will be set set to zero
#define MAX_TEMPERATURE 300

// target temperature and temp control parameters
double targetTemp;

unsigned long elapsedTime; // tracks clock time

// global variables for holding temperature and current sensor readings
double refTemperature, sampTemperature;
double refCurrent, sampCurrent;
double refHeatFlow, sampHeatFlow;

// The mass (in grams) of each of the material samples
double refMass = 1, sampMass = 1;

/**
 * Reads the values from each of the sensor pins and computes to average of
 * multiple measurements for each pin
 */
void readSensorValues()
{
  unsigned long latestSampleTime;

  // Zero the array before taking samples
  memset(sensorValues, 0, sizeof(sensorValues));

  // Read the analog signals from the sensors
  for (int i = 0; i < AVG_SAMPLES; i++)
  {
    latestSampleTime = millis();

    sensorValues[0] += analogRead(REF_TEMP_PROBE_PIN);
    sensorValues[1] += analogRead(SAMP_TEMP_PROBE_PIN);

    // Current sensor values are squared for RMS calculation
    sensorValues[2] += sq(analogRead(REF_CURRENT_SENS_PIN) - analogMidpoint);
    sensorValues[3] += sq(analogRead(SAMP_CURRENT_SENS_PIN) - analogMidpoint);

    // Wait 2 milliseconds before the next loop for the analog-to-digital
    // converter to settle after the last reading
    while ((millis() - latestSampleTime) < AVG_SAMPLE_DELAY)
      ;
  }

  // Calculate the average of the samples
  for (int i = 0; i < 4; i++)
  {
    sensorValues[i] = sensorValues[i] / AVG_SAMPLES;
  }

  // Calculate the RMS for current sensors
  for (int i = 2; i < 4; i++)
  {
    sensorValues[i] = sqrt(sensorValues[i]);
  }
}

/**
 * Reads the values from each of the sensor pins and converts them to the
 * appropriate units, storing the result in the global variables
 */
void updateSensorData()
{
  readSensorValues();

  // The voltage is in millivolts
  double refTempVoltage = sensorValues[0] * byteToMillivolts;
  double sampTempVoltage = sensorValues[1] * byteToMillivolts;
  double refCurrentVoltage = sensorValues[2] * byteToMillivolts;
  double sampCurrentVoltage = sensorValues[3] * byteToMillivolts;

  // Convert the voltage readings into appropriate units. This will calculate
  // the temperature (in Celcius)
  refTemperature = (refTempVoltage - AMPLIFIER_VOLTAGE_OFFSET) / AMPLIFIER_CONVERSION_FACTOR;
  sampTemperature = (sampTempVoltage - AMPLIFIER_VOLTAGE_OFFSET) / AMPLIFIER_CONVERSION_FACTOR;
  // This will calculate the actual current (in mA). Using the ~~Vref~~ and
  // sensitivity settings you configure
  refCurrent = (refCurrentVoltage - CURRENT_SENSOR_VREF) * CURRENT_SENSOR_SENS;
  sampCurrent = (sampCurrentVoltage - CURRENT_SENSOR_VREF) * CURRENT_SENSOR_SENS;
}

/**
 * Calcutate the heat flow
 */
void calculateHeatFlow()
{
  // Convert current from milliAmps to Amps
  double refCurrentAmps = refCurrent / 1000.0;
  double sampCurrentAmps = sampCurrent / 1000.0;
  // Calculate the heat flow as milliWatts per gram
  refHeatFlow = refCurrent * HEATING_COIL_VOLTAGE / refMass;
  sampHeatFlow = refCurrent * HEATING_COIL_VOLTAGE / sampMass;
}

/**
 * Send the latest measurement data via the serial bus
 */
void sendData()
{
  // Send the char 'd' to indicate the start of data set
  Serial.println('d');

  Serial.println("ElapsedTime(ms),TargetTemp(C),RefTemp(C),SampTemp(C),RefCurrent(mA),SampCurrent(mA),RefHeatFlow(),SampHeatFlow(),RefDutyCycle(%),SampDutyCycle(%)");

  // Send each value in the expected order, separated by commas
  Serial.print(elapsedTime);
  Serial.print(",");
  Serial.print(targetTemp);
  Serial.print(",");

  Serial.print(refTemperature);
  Serial.print(",");
  Serial.print(sampTemperature);
  Serial.print(",");

  Serial.print(refCurrent);
  Serial.print(",");
  Serial.print(sampCurrent);
  Serial.print(",");

  Serial.print(refHeatFlow);
  Serial.print(",");
  Serial.print(sampHeatFlow);
  Serial.print(",");

  Serial.print(0);
  Serial.print(",");
  Serial.println(0);
}

void setup()
{
  Serial.begin(9600);

  pinMode(13, OUTPUT);

  analogReadResolution(ANALOG_RESOLUTION);

  // Set the temperature and current sensor pins
  pinMode(REF_TEMP_PROBE_PIN, INPUT);
  pinMode(SAMP_TEMP_PROBE_PIN, INPUT);
  pinMode(REF_CURRENT_SENS_PIN, INPUT);
  pinMode(SAMP_CURRENT_SENS_PIN, INPUT);

  // Set the relay output pins
  pinMode(Ref_Heater_PIN, OUTPUT);
  pinMode(Samp_Heater_PIN, OUTPUT);

  // NeoPixel initialization
  neopixel.begin();
  neopixel.show(); // Initialize all pixels to 'off'

  targetTemp = 300;
}

void loop()
{
  digitalWrite(13, HIGH); // Blink the LED

  // Record the time
  elapsedTime = 0;

  // Read the measurements from the sensors
  updateSensorData();

  // Calcutate the heat flow
  calculateHeatFlow();

  // Set placeholder target temp
  targetTemp = 300;

  digitalWrite(13, LOW); // Blink the LED

  int maxTempCounter = 0;

  if (refTemperature < MAX_TEMPERATURE)
  {
    digitalWrite(Ref_Heater_PIN, HIGH);
  }
  else
  {
    digitalWrite(Ref_Heater_PIN, LOW);
    maxTempCounter++;
  }

  if (sampTemperature < MAX_TEMPERATURE)
  {
    digitalWrite(Samp_Heater_PIN, HIGH);
  }
  else
  {
    digitalWrite(Samp_Heater_PIN, LOW);
    maxTempCounter++;
  }

  switch (maxTempCounter)
  {
  case 0:
    neopixel.fill(green);
    neopixel.show();
    break;
  case 1:
    neopixel.fill(yellow);
    neopixel.show();
    break;
  case 2:
  default:
    neopixel.fill(red);
    neopixel.show();
    break;
  }

  // Send data out via Serial bus
  sendData();
}
