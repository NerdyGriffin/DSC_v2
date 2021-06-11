/*
   DSC_v2: UI and control systems for prototype DSC system
   Copyright (C) 2020  Christian Kunis

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <https://www.gnu.org/licenses/>

   You may contact the author at ckunis.contact@gmail.com
*/

#include <Adafruit_NeoPixel.h>
#include <AutoPID.h>

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

// Number of samples to average the reading over
// Change this to make the reading smoother... but beware of buffer overflows!
const int avgSamples = 200;
//! Must NOT exceed 2^(32 - 2*ANALOG_RESOLUTION)
//! to prevent overflow error during summation
// For 10-bit analog res, this max is 4096 samples
// For 12-bit analog res, this max is 256 samples

// global variable for holding the raw analog sensor values
unsigned long sensorValues[4];

// PID settings and gains
#define PULSE_WIDTH 100 // Pulse width in milliseconds
double Kp = 0.01;
double Ki = 0;
double Kd = 0;
#define BANG_RANGE 4
#define PID_UPDATE_INTERVAL PULSE_WIDTH

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
//#define CURRENT_SENSOR_VREF 1650.0 // Output voltage with no current: ~ 1650mV or 1.65V
//#define CURRENT_SENSOR_VREF 2500.0 // Output voltage with no current: ~ 2500mV or 2.5V

// The constant voltage supplied to the heating coils
#define HEATING_COIL_VOLTAGE 23.0 // Theoretical 24 VAC
// I recommend measuring the real-world voltage across the resistor and adjusting this value accordingly

// The resistance of the heating coil circuit (Ohms, not kilo-Ohms)
#define HEATING_COIL_RESISTANCE 49.0
// For best accuracy during sensor calibration, this value should be measured as the
// total resistance around the heating coil circuit, not just the resistance of the
// heating components alone.

// Max allowable temperature.
// If the either temperature exceeds this value, the PWM duty cycle will be set
// set to zero
#define MAX_TEMPERATURE 300

// The minimum acceptable error between the sample temperatures and the target
// temperature. The error for both samples must be less than this value before
// the stage controller will continue to the next stage. Units: [degrees C]
#define MINIMUM_ACCEPTABLE_ERROR 5

// The number of the consecutive samples within the
// MINIMUM_ACCEPTABLE_ERROR that are required before the program
// considers the target to be satisfied
#define TARGET_COUNTER_THRESHOLD 100

// target temperature and temp control parameters
double targetTemp;
double startTemp;
double endTemp;
double rampUpRate;
double holdTime;

unsigned long latestSampleTime;
unsigned long latestTime, elapsedTime; // tracks clock time
unsigned long rampUpStartTime;

// Counter used to hold samples at start temperature before beginning to ramp up
// the target temperature
unsigned long startCounter;

// global variables for holding temperature and current sensor readings
double refTemperature, sampTemperature;
double refCurrent, sampCurrent;
double refHeatFlow, sampHeatFlow;

// PID Relay output values
bool refRelayState, sampRelayState;

//input/output variables passed by reference, so they are updated automatically
AutoPIDRelay refPID(&refTemperature, &targetTemp, &refRelayState, PULSE_WIDTH, Kp, Ki, Kd);
AutoPIDRelay sampPID(&sampTemperature, &targetTemp, &sampRelayState, PULSE_WIDTH, Kp, Ki, Kd);

// The mass (in grams) of each of the material samples
double refMass = 1, sampMass = 1;

// Debug variables used to override the control loop end conditions
bool debugMode = false;
#define DEBUG_TIME_LIMIT 100000

/*
   Send the PID gain constants via the serial bus
*/
void sendPIDGains()
{
  // Send the char 'k' to indicate the start of the PID data set
  Serial.println('k');

  // Send each value in the expected order, separated by newlines
  Serial.println(Kp);
  Serial.println(Ki);
  Serial.println(Kd);
}

/*
   Receive the PID gain constants via the serial bus
*/
void receivePIDGains()
{
  // Read the incoming data as a float
  Kp = Serial.parseFloat();
  Ki = Serial.parseFloat();
  Kd = Serial.parseFloat();

  // Update the PID gains
  refPID.setGains(Kp, Ki, Kd);
  sampPID.setGains(Kp, Ki, Kd);
}

/*
   Send the temperature control parameters via the serial bus
*/
void sendControlParameters()
{
  // Send the char 'c' to indicate the start of config data set
  Serial.println('c');

  // Send each value in the expected order, separated by newlines
  Serial.println(startTemp);
  Serial.println(endTemp);
  Serial.println(rampUpRate);
  Serial.println(holdTime);
}

/*
   Receive the temperature control parameters via the serial bus
*/
void receiveControlParameters()
{
  // Read the incoming data as a float
  startTemp = Serial.parseFloat();
  endTemp = Serial.parseFloat();
  rampUpRate = Serial.parseFloat();
  holdTime = Serial.parseFloat();
}

/*
   Reads the values from each of the sensor pins and computes to average of multiple measurements for each pin
*/
void getSensorValues()
{
  // Zero the array before taking samples
  memset(sensorValues, 0, sizeof(sensorValues));

  // Read the analog signals from the sensors
  for (int i = 0; i < avgSamples; i++)
  {
    latestSampleTime = millis();

    sensorValues[0] += analogRead(REF_TEMP_PROBE_PIN);
    sensorValues[1] += analogRead(SAMP_TEMP_PROBE_PIN);

    // Current sensor values are squared for RMS calculation
    sensorValues[2] += sq(analogRead(REF_CURRENT_SENS_PIN) - analogMidpoint);
    sensorValues[3] += sq(analogRead(SAMP_CURRENT_SENS_PIN) - analogMidpoint);

    // Wait 2 milliseconds before the next loop for the analog-to-digital
    // converter to settle after the last reading
    while ((millis() - latestSampleTime) < 2)
      ;
  }

  // Calculate the average of the samples
  for (int i = 0; i < 4; i++)
  {
    sensorValues[i] = sensorValues[i] / avgSamples;
  }

  // Calculate the RMS for current sensors
  for (int i = 2; i < 4; i++)
  {
    sensorValues[i] = sqrt(sensorValues[i]);
  }
}

/*
   ! -- Experimental Feature -- !

   Calculate the current sensor offset and sensitivity values

   (This function is not yet completed/functional)
*/
/*
  void calibrateCurrentSensors(double *refCurrentSensorVref, double *sampCurrentSensorVref, double *refCurrentSensorSens, double *sampCurrentSensorSens)
  {
  // Set the heater circuit to OFF to measure the baseline (current sensors measure 0 mA)
  digitalWrite(Ref_Heater_PIN, LOW);
  digitalWrite(Samp_Heater_PIN, LOW);

  // Take a measurement of the sensor with expected current reading 0 mA
  getSensorValues();

  // The current sensor voltage is in millivolts
  double ref_VREF = sensorValues[2] * byteToMillivolts;
  double samp_VREF = sensorValues[3] * byteToMillivolts;
  // Calculate the ideal VREF using the zero current measurement
  refCurrentSensorVref = ref_VREF;
  sampCurrentSensorVref = samp_VREF;

  // Set the heater circuit to ON to measure the sensitivity (current sensors measure V_AC / R_heater)
  digitalWrite(Ref_Heater_PIN, HIGH);
  digitalWrite(Samp_Heater_PIN, HIGH);

  // Take a measurement of the sensor with expected maximum current
  getSensorValues();

  // The current sensor voltage is in millivolts
  double refCurrentVoltage = sensorValues[2] * byteToMillivolts;
  double sampCurrentVoltage = sensorValues[3] * byteToMillivolts;

  // Calculate the expected current through the heating coils (in mA)
  double idealHeatingCoilCurrent = (HEATING_COIL_VOLTAGE / HEATING_COIL_RESISTANCE) * 1000;

  // Calculate the ideal SENS using the max current measurement
  refCurrentSensorSens = idealHeatingCoilCurrent / (refCurrentVoltage - ref_VREF);
  sampCurrentSensorSens = idealHeatingCoilCurrent / (sampCurrentVoltage - samp_VREF);

  // Reset the heater circuit to OFF to at the end of calibration
  digitalWrite(Ref_Heater_PIN, LOW);
  digitalWrite(Samp_Heater_PIN, LOW);
  }
*/

/*
   Reads the values from each of the sensor pins and converts them to the
   appropriate units, storing the result in the global variables
*/
void readSensors(double *refTemperature, double *sampTemperature, double *refCurrent, double *sampCurrent)
{
  getSensorValues();

  // The voltage is in millivolts
  double refTempVoltage = sensorValues[0] * byteToMillivolts;
  double sampTempVoltage = sensorValues[1] * byteToMillivolts;
  double refCurrentVoltage = sensorValues[2] * byteToMillivolts;
  double sampCurrentVoltage = sensorValues[3] * byteToMillivolts;

  // Convert the voltage readings into appropriate units.
  // This will calculate the temperature (in Celcius)
  *refTemperature = (refTempVoltage - AMPLIFIER_VOLTAGE_OFFSET) / AMPLIFIER_CONVERSION_FACTOR;
  *sampTemperature = (sampTempVoltage - AMPLIFIER_VOLTAGE_OFFSET) / AMPLIFIER_CONVERSION_FACTOR;
  // This will calculate the actual current (in mA)
  // Using the ~~Vref~~ and sensitivity settings you configure
  //? (Vref is automatically accounted for during RMS calculation)
  *refCurrent = (refCurrentVoltage)*CURRENT_SENSOR_SENS;
  *sampCurrent = (sampCurrentVoltage)*CURRENT_SENSOR_SENS;
}

/*
   Calcutate the heat flow
*/
void calculateHeatFlow(double *refHeatFlow, double *sampHeatFlow, double refCurrent, double sampCurrent)
{
  // Convert current from milliAmps to Amps
  double refCurrentAmps = refCurrent / 1000.0;
  double sampCurrentAmps = sampCurrent / 1000.0;
  // Calculate the heat flow as Watts per gram
  *refHeatFlow = refCurrentAmps * HEATING_COIL_VOLTAGE / refMass;
  *sampHeatFlow = sampCurrentAmps * HEATING_COIL_VOLTAGE / sampMass;
}

/*
   Calcutate the target temperature
*/
void updateTargetTemperature(double *targetTemp, double startTemp, double endTemp, double rampUpRate, double latestTime)
{
  if (startCounter < TARGET_COUNTER_THRESHOLD)
  {
    *targetTemp = startTemp;
    if ((refPID.atSetPoint(MINIMUM_ACCEPTABLE_ERROR)) &&
        (sampPID.atSetPoint(MINIMUM_ACCEPTABLE_ERROR)))
    {
      startCounter++;
    }
    else
    {
      startCounter = 0;
    }
    rampUpStartTime = millis();
  }
  else if ((*targetTemp < endTemp) && (endTemp > startTemp))
  {
    *targetTemp = startTemp + (rampUpRate / (60 * 1000)) * (latestTime - rampUpStartTime);
  }
  else if ((*targetTemp > endTemp) && (endTemp < startTemp))
  {
    *targetTemp = startTemp - (rampUpRate / (60 * 1000)) * (latestTime - rampUpStartTime);
  }
  else
  {
    *targetTemp = endTemp;
  }
}

/*
   Send the latest measurement data via the serial bus
*/
void sendData()
{
  // Send the char 'd' to indicate the start of data set
  Serial.println('d');

  // Send each value in the expected order, separated by newlines
  Serial.println(elapsedTime);
  Serial.println(targetTemp);

  Serial.println(refTemperature);
  Serial.println(sampTemperature);

  Serial.println(refCurrent);
  Serial.println(sampCurrent);

  Serial.println(refHeatFlow);
  Serial.println(sampHeatFlow);

  Serial.println(refPID.getPulseValue());
  Serial.println(sampPID.getPulseValue());
}

/*
   Temperature control loop
*/
void controlLoop()
{
  // Send the char 's' to indicate the start of control loop
  Serial.println('s');

  unsigned long startTime = millis();
  unsigned long holdStartTime = millis();
  startCounter = 0;
  unsigned long targetCounter = 0;
  bool controlLoopState = true;
  while (controlLoopState)
  {
    // Check for interupts from the UI
    if (Serial.available())
    {
      digitalWrite(13, HIGH);

      // Read the incoming data as a char
      int inByte = Serial.read();

      switch (inByte)
      {
      case 'x':
        // Received stop commmand

        // Stop PID calculations and reset internal PID calculation values
        refPID.stop();
        sampPID.stop();

        // Turn off the PWM Relay output
        digitalWrite(Ref_Heater_PIN, LOW);
        digitalWrite(Samp_Heater_PIN, LOW);

        neopixel.fill(red);
        neopixel.show();
        // Confirm by sending the same command back
        Serial.println('x');
        return;
        break;
      default:
        break;
      }
    }

    digitalWrite(13, LOW);

    neopixel.fill(green);
    neopixel.show();

    // Record the time
    latestTime = millis();
    elapsedTime = latestTime - startTime;

    // Read the measurements from the sensors
    readSensors(&refTemperature, &sampTemperature, &refCurrent, &sampCurrent);

    // Calcutate the heat flow
    calculateHeatFlow(&refHeatFlow, &sampHeatFlow, refCurrent, sampCurrent);

    // Calculate the new target temperature
    updateTargetTemperature(&targetTemp, startTemp, endTemp, rampUpRate, latestTime);

    // Run the PID algorithm
    refPID.run();
    sampPID.run();

    // Update the PWM Relay output
    if (refTemperature < MAX_TEMPERATURE)
      digitalWrite(Ref_Heater_PIN, refRelayState);
    else
      digitalWrite(Ref_Heater_PIN, LOW);

    if (sampTemperature < MAX_TEMPERATURE)
      digitalWrite(Samp_Heater_PIN, sampRelayState);
    else
      digitalWrite(Samp_Heater_PIN, LOW);

    // Send data out via Serial bus
    sendData();

    // Check loop exit conditions
    if (targetCounter < TARGET_COUNTER_THRESHOLD)
    {
      if ((targetTemp == endTemp) &&
          (refPID.atSetPoint(MINIMUM_ACCEPTABLE_ERROR)) &&
          (sampPID.atSetPoint(MINIMUM_ACCEPTABLE_ERROR)))
      {
        targetCounter++;
      }
      else
      {
        targetCounter = 0;
      }
      holdStartTime = millis();
    }
    else if ((latestTime - holdStartTime) > (holdTime / 1000))
    {
      controlLoopState = false;
    }

    if (debugMode && (elapsedTime > DEBUG_TIME_LIMIT))
    {
      controlLoopState = false;
    }

    //delay(1);
  }

  // Stop PID calculations and reset internal PID calculation values
  refPID.stop();
  sampPID.stop();

  // Turn off the PWM Relay output
  digitalWrite(Ref_Heater_PIN, LOW);
  digitalWrite(Samp_Heater_PIN, LOW);

  neopixel.fill(magenta);
  neopixel.show();
  // Send the char 'x' to indicate the end of the control loop
  Serial.println("x");
}

void setup()
{
  Serial.begin(9600);

  pinMode(13, OUTPUT);

  analogReadResolution(ANALOG_RESOLUTION);
  //  analogReference(AR_EXTERNAL);

  // Set the temperature and current sensor pins
  pinMode(REF_TEMP_PROBE_PIN, INPUT);
  pinMode(SAMP_TEMP_PROBE_PIN, INPUT);
  pinMode(REF_CURRENT_SENS_PIN, INPUT);
  pinMode(SAMP_CURRENT_SENS_PIN, INPUT);

  // Set the relay output pins
  pinMode(Ref_Heater_PIN, OUTPUT);
  pinMode(Samp_Heater_PIN, OUTPUT);

  // if temperature is more than 4 degrees below or above setpoint, OUTPUT
  // will be set to min or max respectively
  refPID.setBangBang(BANG_RANGE);
  sampPID.setBangBang(BANG_RANGE);
  //set PID update interval
  refPID.setTimeStep(PID_UPDATE_INTERVAL);
  sampPID.setTimeStep(PID_UPDATE_INTERVAL);

  // NeoPixel initialization
  neopixel.begin();
  neopixel.show(); // Initialize all pixels to 'off'

  // Set PID gain constants to default values
  Kp = 0.1;
  Ki = 0;
  Kd = 0;

  // Update the PID gains
  refPID.setGains(Kp, Ki, Kd);
  sampPID.setGains(Kp, Ki, Kd);

  // Set temperature control parameters to default values
  startTemp = 30;
  targetTemp = startTemp;
  endTemp = 120;
  rampUpRate = 1;
  holdTime = 0;

  // Set the sample masses to default values
  refMass = 1.0;
  sampMass = 1.0;
}

void loop()
{
  digitalWrite(13, LOW); // Blink the LED
  delay(500);
  digitalWrite(13, HIGH);
  delay(500);

  neopixel.clear();
  neopixel.show();

  if (Serial.available())
  {
    digitalWrite(13, HIGH);

    // Read the incoming data as a char
    int inByte = Serial.read();

    switch (inByte)
    {
    case 'i':
      // Received initialization command
      neopixel.fill(cyan);
      neopixel.show();
      // Send the PID gain constants via the serial bus
      sendPIDGains();
      // Send the temperature control parameters via the serial bus
      sendControlParameters();
      break;
    case 'l':
      // Received load control parameters commmand
      neopixel.fill(cyan);
      neopixel.show();
      // Receive the temperature control parameters via the serial bus
      receiveControlParameters();
      // Send the temperature control parameters to confirm that the
      // values were received properly
      sendControlParameters();
      break;
    case 'p':
      // Received load PID gains commmand
      neopixel.fill(cyan);
      neopixel.show();
      // Receive the PID gain constants via the serial bus
      receivePIDGains();
      // Send the PID gain constants to confirm that the values were
      // received properly
      sendPIDGains();
      break;
    case 's':
      // Received start commmand
      neopixel.fill(green);
      neopixel.show();
      // Run the temperature control loop
      controlLoop();
      break;
    case 10:
      // Received newline char
      neopixel.fill(blue);
      neopixel.show();
      break;
    default:
      break;
    }
  }
}
