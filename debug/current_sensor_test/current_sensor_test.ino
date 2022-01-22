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
#include <INA219_WE.h>
#include <Wire.h>

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
#define SAMP_TEMP_PROBE_PIN A4
#define Ref_Heater_PIN 11
#define Samp_Heater_PIN 10
#define REF_CURRENT_I2C 0x41
#define SAMP_CURRENT_I2C 0x40

// INA219 Current Sensor Devices
INA219_WE REF_INA219 = INA219_WE(REF_CURRENT_I2C);   // ref heater ina219 board
INA219_WE SAMP_INA219 = INA219_WE(SAMP_CURRENT_I2C); // sample heater ina219 board

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
#define AVG_SAMPLE_DELAY 2000UL // Sample delay in microseconds

// global variable for holding the raw analog sensor values
unsigned long sensorValues[2];

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

#define REF_TEMP_CALIBRATION_OFFSET 6.666666666666666
#define SAMP_TEMP_CALIBRATION_OFFSET 3.000000000000000

// Max allowable temperature. If the either temperature exceeds this value, the
// PWM duty cycle will be set set to zero
#define MAX_TEMPERATURE 200.0

// target temperature and temp control parameters
double targetTemp;

unsigned long elapsedTime; // tracks clock time

// global variables for holding temperature and current sensor readings
double refTemperature, sampTemperature;

double refShuntVoltage_mV, sampShuntVoltage_mV;
double refBusVoltage_V, sampBusVoltage_V;
double refLoadVoltage_V, sampLoadVoltage_V;
double refCurrent_mA, sampCurrent_mA;
double refPower, sampPower;
bool refOverflow = false, sampOverflow = false;

double refHeatFlow, sampHeatFlow;

// The mass (in grams) of each of the material samples
double refMass = 1, sampMass = 1;

/**
 * Reads the values from each of the sensor pins and computes to average of
 * multiple measurements for each pin
 */
void readTempSensorValues()
{
  // Zero the array before taking samples
  memset(sensorValues, 0, sizeof(sensorValues));

  // Read the analog signals from the sensors
  unsigned long latestSampleTime = micros();
  for (int i = 0; i < AVG_SAMPLES; i++)
  {
    sensorValues[0] += analogRead(REF_TEMP_PROBE_PIN);
    sensorValues[1] += analogRead(SAMP_TEMP_PROBE_PIN);

    // Wait 2 milliseconds before the next loop for the analog-to-digital
    // converter to settle after the last reading
    while ((micros() - latestSampleTime) < AVG_SAMPLE_DELAY)
      ; // busy wait

    latestSampleTime += AVG_SAMPLE_DELAY;
  }

  // Calculate the average of the samples
  for (int i = 0; i < 2; i++)
  {
    sensorValues[i] = sensorValues[i] / AVG_SAMPLES;
  }
}

/**
 * Reads the values from each of the sensor pins and converts them to the
 * appropriate units, storing the result in the global variables
 */
void updateSensorData()
{
  readTempSensorValues();

  // The voltage is in millivolts
  double refTempVoltage = sensorValues[0] * byteToMillivolts;
  double sampTempVoltage = sensorValues[1] * byteToMillivolts;

  // Convert the voltage readings into appropriate units. This will calculate
  // the temperature (in Celsius)
  refTemperature = (refTempVoltage - AMPLIFIER_VOLTAGE_OFFSET) / AMPLIFIER_CONVERSION_FACTOR - REF_TEMP_CALIBRATION_OFFSET;
  sampTemperature = (sampTempVoltage - AMPLIFIER_VOLTAGE_OFFSET) / AMPLIFIER_CONVERSION_FACTOR - SAMP_TEMP_CALIBRATION_OFFSET;

  REF_INA219.startSingleMeasurement();
  refShuntVoltage_mV = REF_INA219.getShuntVoltage_mV();
  refBusVoltage_V = REF_INA219.getBusVoltage_V();
  refCurrent_mA = REF_INA219.getCurrent_mA();
  refPower = REF_INA219.getBusPower();
  refLoadVoltage_V = refBusVoltage_V + (refShuntVoltage_mV / 1000);
  refOverflow = REF_INA219.getOverflow();

  SAMP_INA219.startSingleMeasurement();
  sampShuntVoltage_mV = SAMP_INA219.getShuntVoltage_mV();
  sampBusVoltage_V = SAMP_INA219.getBusVoltage_V();
  sampCurrent_mA = SAMP_INA219.getCurrent_mA();
  sampPower = SAMP_INA219.getBusPower();
  sampLoadVoltage_V = sampBusVoltage_V + (sampShuntVoltage_mV / 1000);
  sampOverflow = SAMP_INA219.getOverflow();
}

/**
 * Calcutate the heat flow
 */
void calculateHeatFlow()
{
  // // Convert current from milliAmps to Amps
  // double refCurrentAmps = refCurrent_mA / 1000.0;
  // double sampCurrentAmps = sampCurrent_mA / 1000.0;

  // Calculate the heat flow as Watts per gram
  refHeatFlow = (refPower / 1000.0) / refMass;
  sampHeatFlow = (sampPower / 1000.0) / sampMass;
}

/**
 * Send the latest measurement data via the serial bus
 */
void sendData()
{
  Serial.println("ElapsedTime(ms),TargetTemp(C),RefTemp(C),SampTemp(C),RefShuntVoltage(mV),SampShuntVoltage(mV),RefBusVoltage(V),SampBusVoltage(V),RefLoadVoltage(V),SampLoadVoltage(V),RefCurrent(mA),SampCurrent(mA),RefBusPower(mW),SampBusPower(mW),RefHeatFlow(),SampHeatFlow(),RefDutyCycle(%),SampDutyCycle(%)");

  // Send each value in the expected order, separated by commas
  Serial.print(elapsedTime);
  Serial.print(",");
  Serial.print(targetTemp);
  Serial.print(",");

  Serial.print(refTemperature);
  Serial.print(",");
  Serial.print(sampTemperature);
  Serial.print(",");

  Serial.print(refShuntVoltage_mV);
  Serial.print(",");
  Serial.print(sampShuntVoltage_mV);
  Serial.print(",");

  Serial.print(refBusVoltage_V);
  Serial.print(",");
  Serial.print(sampBusVoltage_V);
  Serial.print(",");

  Serial.print(refLoadVoltage_V);
  Serial.print(",");
  Serial.print(sampLoadVoltage_V);
  Serial.print(",");

  Serial.print(refCurrent_mA);
  Serial.print(",");
  Serial.print(sampCurrent_mA);
  Serial.print(",");

  Serial.print(refPower);
  Serial.print(",");
  Serial.print(sampPower);
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
  while (!Serial)
  {
    // will pause Zero, Leonardo, etc until serial console opens
    delay(1);
  }

  // Initialize INA219 boards.
  Wire.begin();
  if (!REF_INA219.init())
  {
    Serial.println("Failed to find INA219 chip for reference heater");
    while (1)
    {
      delay(10);
    }
  }
  if (!SAMP_INA219.init())
  {
    Serial.println("Failed to find INA219 chip for sample heater");
    while (1)
    {
      delay(10);
    }
  }

  pinMode(13, OUTPUT);

  analogReadResolution(ANALOG_RESOLUTION);

  // Set up ADC mode for INA219 boards
  // ADC automatic hw averaging is available with SAMPLE_MODE_XX, using single sample here
  REF_INA219.setADCMode(BIT_MODE_12);
  SAMP_INA219.setADCMode(BIT_MODE_12);

  // Set measurement mode to single-shot triggered
  REF_INA219.setMeasureMode(TRIGGERED);
  SAMP_INA219.setMeasureMode(TRIGGERED);

  // Set the temperature and current sensor pins
  pinMode(REF_TEMP_PROBE_PIN, INPUT);
  pinMode(SAMP_TEMP_PROBE_PIN, INPUT);

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

  // Send data out via Serial bus
  sendData();

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
}
