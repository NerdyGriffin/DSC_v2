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

// PID settings and gains
#define PULSE_WIDTH 100 // Pulse width in milliseconds
double Kp = 1;
double Ki = 0;
double Kd = 0;
#define BANG_RANGE 10
// When the temperature is less than {TargetTemp - BANG_RANGE}, the PID control
// is deactivated, and the output is set to max
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

// The minimum acceptable error between the sample temperatures and the target
// temperature. The error for both samples must be less than this value before
// the stage controller will continue to the next stage. Units: [degrees C]
#define MINIMUM_ACCEPTABLE_ERROR 5

// The number of the consecutive samples within the MINIMUM_ACCEPTABLE_ERROR
// that are required before the program considers the target to be satisfied
#define TARGET_COUNTER_THRESHOLD 100

// target temperature and temp control parameters
double targetTemp;
double startTemp;
double endTemp;
double rampUpRate;
double holdTime;

unsigned long latestTime, elapsedTime; // tracks clock time
unsigned long rampUpStartTime;
unsigned long holdStartTime;

// Counter used to hold samples at start temperature before beginning to ramp up
// the target temperature
unsigned long standbyCounter;
unsigned long startCounter;
unsigned long endCounter;

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
#define DEBUG_MODE false
#define DEBUG_TIME_LIMIT 300000

/**
 * Send the PID gain constants via the serial bus
 */
void sendPIDGains()
{
  // Send the char 'k' to indicate the start of the PID data set
  Serial.println('k');

  Serial.println("Kp,Ki,Kd");

  // Send each value in the expected order, separated by newlines
  Serial.print(Kp);
  Serial.print(",");
  Serial.print(Ki);
  Serial.print(",");
  Serial.println(Kd);
}

/**
 * Receive the PID gain constants via the serial bus
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

/**
 * Send the temperature control parameters via the serial bus
 */
void sendControlParameters()
{
  // Send the char 'c' to indicate the start of config data set
  Serial.println('c');

  Serial.println("StartTemp(C),EndTemp(C),RampUpRate(C/min),HoldTime(sec)");

  // Send each value in the expected order, separated by newlines
  Serial.print(startTemp);
  Serial.print(",");
  Serial.print(endTemp);
  Serial.print(",");
  Serial.print(rampUpRate);
  Serial.print(",");
  Serial.println(holdTime);
}

/**
 * Receive the temperature control parameters via the serial bus
 */
void receiveControlParameters()
{
  // Read the incoming data as a float
  startTemp = Serial.parseFloat();
  endTemp = Serial.parseFloat();
  rampUpRate = Serial.parseFloat();
  holdTime = Serial.parseFloat();
}

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

    // Run the PID algorithm
    refPID.run();
    sampPID.run();

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
 * ! -- Experimental Feature -- !
 *
 * Calculate the current sensor offset and sensitivity values
 *
 * (This function is not yet completed/functional)
 */
void calibrateCurrentSensors()
{
  // Set the heater circuit to OFF to measure the baseline (current sensors
  // measure 0 mA)
  digitalWrite(Ref_Heater_PIN, LOW);
  digitalWrite(Samp_Heater_PIN, LOW);

  delay(100);

  // Take a measurement of the sensor with expected current reading 0 mA
  readSensorValues();

  // Calculate the ideal VREF using the zero current measurement. The voltage is
  // in millivolts
  Ref_Current_Sensor_VRef = sensorValues[2] * byteToMillivolts;
  Samp_Current_Sensor_VRef = sensorValues[3] * byteToMillivolts;

  // Set the heater circuit to ON to measure the sensitivity (current sensors
  // measure V_{AC} / R_{heater})
  digitalWrite(Ref_Heater_PIN, HIGH);
  digitalWrite(Samp_Heater_PIN, HIGH);

  delay(100);

  // Take a measurement of the sensor with expected maximum current
  readSensorValues();

  // The current sensor voltage is in millivolts
  double refCurrentVoltage = sensorValues[2] * byteToMillivolts;
  double sampCurrentVoltage = sensorValues[3] * byteToMillivolts;

  // Calculate the expected current through the heating coils (in mA)
  double idealHeatingCoilCurrent = (HEATING_COIL_VOLTAGE / HEATING_COIL_RESISTANCE) * 1000;

  // Calculate the ideal SENS using the max current measurement
  Ref_Current_Sensor_Sens = idealHeatingCoilCurrent / (refCurrentVoltage - Ref_Current_Sensor_VRef);
  Samp_Current_Sensor_Sens = idealHeatingCoilCurrent / (sampCurrentVoltage - Samp_Current_Sensor_VRef);

  // Reset the heater circuit to OFF to at the end of calibration
  digitalWrite(Ref_Heater_PIN, LOW);
  digitalWrite(Samp_Heater_PIN, LOW);
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
  // Calculate the heat flow as Watts per gram
  refHeatFlow = refCurrent * HEATING_COIL_VOLTAGE / refMass;
  sampHeatFlow = sampCurrent * HEATING_COIL_VOLTAGE / sampMass;
}

/**
 * Calcutate the target temperature
 */
void updateTargetTemperature()
{
  if (startCounter < TARGET_COUNTER_THRESHOLD)
  {
    targetTemp = startTemp;
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
  else
  {
    if (endTemp > startTemp)
    {
      targetTemp = startTemp + (latestTime - rampUpStartTime) * rampUpRate / 60000.0;
      if (targetTemp > endTemp)
      {
        targetTemp = endTemp;
      }
    }
    else if (endTemp < startTemp)
    {
      targetTemp = startTemp - (latestTime - rampUpStartTime) * rampUpRate / 60000.0;
      if (targetTemp < endTemp)
      {
        targetTemp = endTemp;
      }
    }
    else
    {
      targetTemp = endTemp;
    }
    //! DEBUG: Ramp up has been disabled for PID tuning
    targetTemp = endTemp;
  }
}

/**
 * Send the latest measurement data via the serial bus
 */
void sendData()
{
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

  Serial.print(refPID.getPulseValue());
  Serial.print(",");
  Serial.println(sampPID.getPulseValue());
}

/**
 * Temperature control loop
 */
void controlLoop()
{
  // Send the char 's' to indicate the start of control loop
  Serial.println('s');

  unsigned long startTime = millis();
  rampUpStartTime = millis();
  holdStartTime = millis();
  startCounter = 0;
  endCounter = 0;
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

    // Read the measurements from the sensors
    updateSensorData();

    // Calcutate the heat flow
    calculateHeatFlow();

    // Record the time
    latestTime = millis();
    elapsedTime = latestTime - startTime;

    // Calculate the new target temperature
    updateTargetTemperature();

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
    if (targetTemp == endTemp)
    {
      if (endCounter < TARGET_COUNTER_THRESHOLD)
      {
        if ((refPID.atSetPoint(MINIMUM_ACCEPTABLE_ERROR)) &&
            (sampPID.atSetPoint(MINIMUM_ACCEPTABLE_ERROR)))
        {
          endCounter++;
        }
        else
        {
          endCounter = 0;
        }
        holdStartTime = millis();
      }
      else if ((latestTime - holdStartTime) > (holdTime * 1000))
      {
        controlLoopState = false;
      }
    }

    if (DEBUG_MODE && (elapsedTime > DEBUG_TIME_LIMIT))
    {
      controlLoopState = false;
    }
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

/**
 * Passive sensor measurements
 */
void standbyData()
{
  digitalWrite(13, LOW); // Blink the LED

  // Read the measurements from the sensors
  updateSensorData();

  // Calcutate the heat flow
  calculateHeatFlow();

  // Zero the time during standby mode
  elapsedTime = 0;

  // Set standby target temp
  targetTemp = 20;

  // Stop PID calculations and reset internal PID calculation values
  refPID.stop();
  sampPID.stop();

  // Turn off the PWM Relay output
  digitalWrite(Ref_Heater_PIN, LOW);
  digitalWrite(Samp_Heater_PIN, LOW);

  // Send data out via Serial bus
  sendData();
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

  // if temperature is more than 4 degrees below or above setpoint, OUTPUT will
  // be set to min or max respectively
  refPID.setBangBang(BANG_RANGE);
  sampPID.setBangBang(BANG_RANGE);
  // set PID update interval
  refPID.setTimeStep(PID_UPDATE_INTERVAL);
  sampPID.setTimeStep(PID_UPDATE_INTERVAL);

  // NeoPixel initialization
  neopixel.begin();
  neopixel.show(); // Initialize all pixels to 'off'

  // Set PID gain constants to default values
  Kp = 0.01;
  Ki = 0;
  Kd = 0;

  // Update the PID gains
  refPID.setGains(Kp, Ki, Kd);
  sampPID.setGains(Kp, Ki, Kd);

  // Set temperature control parameters to default values
  startTemp = 40;
  targetTemp = startTemp;
  endTemp = 42;      // 120;
  rampUpRate = 2000; // 20;
  holdTime = 240;    // 0;

  standbyCounter = 0;

  // Set the sample masses to default values
  refMass = 1.0;
  sampMass = 1.0;
}

void loop()
{
  digitalWrite(13, LOW); // Blink the LED
  delay(500);
  digitalWrite(13, HIGH); // Blink the LED
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
      // Send the temperature control parameters to confirm that the values were
      // received properly
      sendControlParameters();
      break;
    case 'p':
      // Received load PID gains commmand
      neopixel.fill(cyan);
      neopixel.show();
      // Receive the PID gain constants via the serial bus
      receivePIDGains();
      // Send the PID gain constants to confirm that the values were received
      // properly
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
  else if (standbyCounter % 10 == 0)
  {
    standbyCounter++;
  }
  else
  {
    standbyCounter = 0;
    standbyData();
  }
}
