/*  DSC_v2: UI and control systems for prototype DSC system
 *      Copyright (C) 2019  Christian Kunis
 *
 *      This program is free software: you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation, either version 3 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program. If not, see <https://www.gnu.org/licenses/>
 *
 *      You may contact the author at ckunis.contact@gmail.com
 */

#include <Adafruit_NeoPixel.h>
#include <AutoPID.h>

// Feather M0 Express board pinouts
#define REF_TEMP_PROBE_PIN A1
#define REF_CURRENT_SENS_PIN A2
#define SAMP_CURRENT_SENS_PIN A3
#define SAMP_TEMP_PROBE_PIN A4
#define Ref_Heater_PIN 11
#define Samp_Heater_PIN 10

// NeoPixel parameters
#define LED_PIN 8
#define LED_COUNT 1
Adafruit_NeoPixel neopixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Predefined colors for NeoPixel
uint32_t black = neopixel.Color(0, 0, 0);
uint32_t white = neopixel.Color(255, 255, 255);
uint32_t magenta = neopixel.Color(255, 0, 255);
uint32_t red = neopixel.Color(255, 0, 0);
uint32_t yellow = neopixel.Color(255, 255, 0);
uint32_t green = neopixel.Color(0, 255, 0);
uint32_t cyan = neopixel.Color(0, 255, 255);
uint32_t blue = neopixel.Color(0, 0, 255);

// PID settings and gains
#define PULSE_WIDTH 100
#define KP 0.02
#define KI 0.0001
#define KD 0.002
#define PID_THRESHOLD 4
#define PID_UPDATE_INTERVAL 1000

// The source voltage supplied to the sensor boards
#define SENSOR_VOLTAGE 5.0
// The sample resolution of the analogRead() output
#define PWM_RESOLUTION 1024.0
// Analog signal to voltage conversion factor
double byte2volts = (SENSOR_VOLTAGE / PWM_RESOLUTION);

// Thermocouple amplifier conversion constants
#define AMPLIFIER_VOLTAGE_OFFSET -1.25
#define AMPLIFIER_CONVERSION_FACTOR 0.005 // 5 mV/C = 0.005 V/C

// Current sensor conversion constant
#define CURRENT_SENSOR_SENS 0.1 // Sensitivity (Sens) 100 mV/A = 0.1 V/A

// The constant voltage supplied to the heating coils
#define HEATING_COIL_VOLTAGE 24

// Max allowable temperature
// If the either temperature exceeds this value, the PWM duty cycle will be set
// set to zero
#define MAX_TEMPERATURE 300

// The minimum acceptable error between the sample temperatures and
// the target temperature. The error for both samples must be less
// than this value before the stage controller will continue to the
// next stage. Units: [\Delta degrees C]
#define MINIMUM_ACCEPTABLE_ERROR 1.5

// The number of the consecutive samples within the
// MINIMUM_ACCEPTABLE_ERROR that are required before the program
// considers the target to be satisfied
#define TARGET_COUNTER_THRESHOLD 1000

// target temperature and temp control parameters
double targetTemp;
double startTemp;
double endTemp;
double rampUpRate;
double holdTime;

unsigned long startTime, latestTime, elapsedTime; // tracks clock time
unsigned long rampUpStartTime, holdStartTime;

// global variables for holding temperature and current sensor readings
double refTemperature, sampTemperature;
double refCurrent, sampCurrent;
double refHeatFlow, sampHeatFlow;
double refPWMDutyCycle, sampPWMDutyCycle;

// PID Relay output values
bool refRelayState, sampRelayState;

//input/output variables passed by reference, so they are updated automatically
AutoPIDRelay refPID(&refTemperature, &targetTemp, &refRelayState, PULSE_WIDTH, KP, KI, KD);
AutoPIDRelay sampPID(&sampTemperature, &targetTemp, &sampRelayState, PULSE_WIDTH, KP, KI, KD);

// The mass (in grams) of each of the material samples
double refMass = 1, sampMass = 1;

// Counter used to hold samples at start temperature before beginning to ramp up
// the target temperature
int startCounter;

// Debug variables used to override the control loop end conditions
bool debugMode = true;
#define DEBUG_TIME_LIMIT 10000

/*
 * Send the PID gain constants via the serial bus
 */
void sendPIDGains()
{
  // Send the char 'k' to indicate the start of the PID data set
  Serial.println('k');

  // Send each value in the expected order, separated by newlines
  Serial.println(KP);
  Serial.println(KI);
  Serial.println(KD);
}

/*
 * Receive the PID gain constants via the serial bus
 */
void receivePIDGains()
{
}

/*
 * Send the temperature control parameters via the serial bus
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

/*
 * Reads the values from each of the sensor pins and converts them to the
 * appropriate units, storing the result in the global variables
 */
void readSensors(double *refTemperature, double *sampTemperature, double *refCurrent, double *sampCurrent)
{
  int numSamples = 100;

  // Initialize counters to hold the sum of several samples of analog signals
  int refTempSignalSum = 0;
  int sampTempSignalSum = 0;
  int refCurrentSignalSum = 0;
  int sampCurrentSignalSum = 0;

  // Read the analog signals from the sensors
  for (int i = 0; i < numSamples; i++)
  {
    refTempSignalSum += analogRead(REF_TEMP_PROBE_PIN);
    sampTempSignalSum += analogRead(SAMP_TEMP_PROBE_PIN);
    refCurrentSignalSum += analogRead(REF_CURRENT_SENS_PIN);
    sampCurrentSignalSum += analogRead(SAMP_CURRENT_SENS_PIN);
  }

  // Calculate the average of the samples
  double refTempSignalAverage = refTempSignalSum / numSamples;
  double sampTempSignalAverage = sampTempSignalSum / numSamples;
  double refCurrentSignalAverage = refCurrentSignalSum / numSamples;
  double sampCurrentSignalAverage = sampCurrentSignalSum / numSamples;

  // Convert the analog values into voltages
  double refTempVoltage = refTempSignalAverage * byte2volts;
  double sampTempVoltage = sampTempSignalAverage * byte2volts;
  double refCurrentVoltage = refCurrentSignalAverage * byte2volts;
  double sampCurrentVoltage = sampCurrentSignalAverage * byte2volts;

  // Convert the voltage readings into appropriate units
  *refTemperature = (refTempVoltage + AMPLIFIER_VOLTAGE_OFFSET) / AMPLIFIER_CONVERSION_FACTOR;
  *sampTemperature = (sampTempVoltage + AMPLIFIER_VOLTAGE_OFFSET) / AMPLIFIER_CONVERSION_FACTOR;
  *refCurrent = refCurrentVoltage / CURRENT_SENSOR_SENS;
  *sampCurrent = sampCurrentVoltage / CURRENT_SENSOR_SENS;
}

/*
 * Calcutate the heat flow
 */
void calculateHeatFlow(double *refHeatFlow, double *sampHeatFlow, double refCurrent, double sampCurrent)
{
  *refHeatFlow = refCurrent * HEATING_COIL_VOLTAGE / refMass;
  *sampHeatFlow = sampCurrent * HEATING_COIL_VOLTAGE / sampMass;
}

/*
 * Calcutate the target temperature
 */
void updateTargetTemperature(double *targetTemp, double startTemp, double endTemp, double rampUpRate, double latestTime)
{
  if (startCounter < TARGET_COUNTER_THRESHOLD)
  {
    *targetTemp = startTemp;
    if ((abs(*targetTemp - refTemperature) < MINIMUM_ACCEPTABLE_ERROR) &&
        (abs(*targetTemp - sampTemperature) < MINIMUM_ACCEPTABLE_ERROR))
    {
      startCounter++;
    }
    else
    {
      startCounter = 0;
    }
    rampUpStartTime = millis();
  }
  else if (((*targetTemp < endTemp) && (endTemp > startTemp)) ||
           ((*targetTemp > endTemp) && (endTemp < startTemp)))
  {
    *targetTemp = startTemp + (rampUpRate / (60 * 1000)) * (latestTime - rampUpStartTime);
  }
  else
  {
    *targetTemp = endTemp;
  }
}

/*
 * Send the latest measurement data via the serial bus
 */
void sendData()
{
  // Send the char 'd' to indicate the start of data set
  Serial.println('d');

  // Send each value in the expected order, separated by newlines
  Serial.println(startTime);
  Serial.println(latestTime);
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
 * Temperature control loop
 */
void controlLoop()
{
  // Send the char 's' to indicate the start of control loop
  Serial.println('s');

  startTime = millis();
  startCounter = 0;
  int targetCounter = 0;
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
      case 'p':
        // Received pause command
        neopixel.fill(yellow);
        neopixel.show();
        // Confirm by sending the same command back
        Serial.println('p');
        break;
      case 'x':
        // Received stop commmand
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
          (abs(targetTemp - refTemperature) < MINIMUM_ACCEPTABLE_ERROR) &&
          (abs(targetTemp - sampTemperature) < MINIMUM_ACCEPTABLE_ERROR))
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

    delay(1);
  }

  neopixel.fill(magenta);
  neopixel.show();
  // Send the char 'x' to indicate the end of the control loop
  Serial.println("x");
}

void setup()
{
  Serial.begin(9600);

  pinMode(13, OUTPUT);

  // Set the temperature and current sensor pins
  pinMode(REF_TEMP_PROBE_PIN, INPUT);
  pinMode(SAMP_TEMP_PROBE_PIN, INPUT);
  pinMode(REF_CURRENT_SENS_PIN, INPUT);
  pinMode(SAMP_CURRENT_SENS_PIN, INPUT);

  // Set the relay output pins
  pinMode(Ref_Heater_PIN, OUTPUT);
  pinMode(Samp_Heater_PIN, OUTPUT);

  //if temperature is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  refPID.setBangBang(PID_THRESHOLD);
  sampPID.setBangBang(PID_THRESHOLD);
  //set PID update interval
  refPID.setTimeStep(PID_UPDATE_INTERVAL);
  sampPID.setTimeStep(PID_UPDATE_INTERVAL);

  // NeoPixel initialization
  neopixel.begin();
  neopixel.show(); // Initialize all pixels to 'off'

  targetTemp = 25;
  startTemp = 25;
  endTemp = 30;
  rampUpRate = 1;
  holdTime = 0;
}

void loop()
{
  digitalWrite(13, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(500);             // wait for a second
  digitalWrite(13, LOW);  // turn the LED off by making the voltage LOW
  delay(500);             // wait for a second

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
      // Send the temperature control parameters via the serial bus
      sendControlParameters();
      break;
    case 'l':
      // Received load control parameters commmand
      neopixel.fill(cyan);
      neopixel.show();
      // Receive the temperature control parameters via the serial bus
      receiveControlParameters();
      // Send the temperature control parameters to confirm that the values
      // were received properly
      sendControlParameters();
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
