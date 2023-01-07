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
#include <INA219_WE.h>
#include <pidautotuner.h>
#include <Wire.h>

// RTC clock on Adalogger featherwing
#include "RTClib.h"
RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
unsigned long rtcStamp; // rtc timestamp in seconds

// SD card from Adalogger featherwing
#include <SPI.h>
#include <SD.h>
const int chipSelect = 10; // GPIO pin for SD card chip select on featherwing adalogger

// Declare a global file for logging DSC data
File dataFile;

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
#define REF_HEATER_PIN 12
#define SAMP_HEATER_PIN 11
#define REF_CURRENT_I2C 0x41
#define SAMP_CURRENT_I2C 0x40

// INA219 Current Sensor Devices
INA219_WE REF_INA219 = INA219_WE(REF_CURRENT_I2C);   // ref heater ina219 board
INA219_WE SAMP_INA219 = INA219_WE(SAMP_CURRENT_I2C); // sample heater ina219 board

const unsigned long MAX_SERIAL_WAIT_TIME = 10000000UL; // microseconds

const unsigned long STANDBY_LOOP_INTERVAL = 500UL; // milliseconds

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
#define AVG_SAMPLES 128

// Wait 2 milliseconds before the next loop for the analog-to-digital converter
// to settle after the last reading
#define AVG_SAMPLE_DELAY 2000UL // Sample delay in microseconds

/**
 * Loop interval in microseconds.
 * 250,000 microseconds = 0.25 seconds
 */
#define LOOP_INTERVAL 250000UL

// The max voltage of analog input readings
#define ANALOG_REF_VOLTAGE 3.3
// The sample resolution of the analogRead() output
#define ANALOG_RESOLUTION 12
// Analog signal to voltage conversion factor
const double byteToVolts = (ANALOG_REF_VOLTAGE / pow(2, ANALOG_RESOLUTION));
const double byteToMillivolts = 1000.0 * byteToVolts;

// global variable for holding the raw analog sensor values
unsigned long sensorValues[2];

// PID settings and gains
const unsigned int OUTPUT_MIN = 0;
const unsigned int OUTPUT_MAX = pow(2, ANALOG_RESOLUTION) - 1;
// These placeholder values are overwritten during the setup() function
double Kp = 1.0000;
double Ki = 0.0000;
double Kd = 0.0000;

/**
 * When the temperature is less than {TargetTemp - BANG_RANGE}, the PID control
 * is deactivated, and the output is set to max
 */
#define BANG_RANGE 10.0
#define PID_UPDATE_INTERVAL 100UL // Interval in milliseconds (Default is 1000)

// Thermocouple amplifier conversion constants
#define AMPLIFIER_VOLTAGE_OFFSET 1250.0 // 1250 mV = 1.25 V
#define AMPLIFIER_CONVERSION_FACTOR 5.0 // 5 mV/C = 0.005 V/C

#define REF_TEMP_CALIBRATION_OFFSET 0.000000000000000  // 6.666666666666666
#define SAMP_TEMP_CALIBRATION_OFFSET 0.000000000000000 // 3.000000000000000

// Max allowable temperature. If the either temperature exceeds this value, the
// PWM duty cycle will be set set to zero
#define MAX_TEMPERATURE 200.0

// The minimum acceptable error between the sample temperatures and the target
// temperature. The error for both samples must be less than this value before
// the stage controller will continue to the next stage. Units: [degrees C]
#define MINIMUM_ACCEPTABLE_ERROR 1.0

// The number of the consecutive samples within the MINIMUM_ACCEPTABLE_ERROR
// that are required before the program considers the target to be satisfied
#define TARGET_COUNTER_THRESHOLD 100 // Default: 100

// The maximum number of sets of temp control parameters that may be saved
#define MAX_PARAMETERS 8

// target temperature and temp control parameters
double targetTemp;
double startTemp[MAX_PARAMETERS] = {};
double endTemp[MAX_PARAMETERS] = {};
double rampUpRate[MAX_PARAMETERS] = {};
double holdTime[MAX_PARAMETERS] = {};
int numParams;

#define MIN_TO_MICROS 60000000UL
#define SEC_TO_MICROS 1000000.0

// tracks clock time in microseconds
unsigned long microseconds, rampUpStartTime, holdStartTime, standbyTime;

unsigned long standbyCounter;
int startCounter, endCounter;

// global variables for holding temperature and current sensor readings
double elapsedTime; // Time in seconds
double refTemperature, sampTemperature;

double refShuntVoltage_mV, sampShuntVoltage_mV;
double refBusVoltage_V, sampBusVoltage_V;
double refLoadVoltage_V, sampLoadVoltage_V;
double refCurrent_mA, sampCurrent_mA;
double refPower, sampPower;
bool refOverflow = false, sampOverflow = false;

double refHeatFlow, sampHeatFlow;

double refDutyCycle, sampDutyCycle;

// Analog signal to percentage conversion factor
const double byteToPercent = (100.0 / pow(2, ANALOG_RESOLUTION));

double refPIDOutput, sampPIDOutput;

// input/output variables passed by reference, so they are updated automatically
AutoPID refPID(&refTemperature, &targetTemp, &refPIDOutput, OUTPUT_MIN, OUTPUT_MAX, Kp, Ki, Kd);
AutoPID sampPID(&sampTemperature, &targetTemp, &sampPIDOutput, OUTPUT_MIN, OUTPUT_MAX, Kp, Ki, Kd);

// Variables used to simulate the implementation of AutoPIDRelay during PID
// autotuning
bool tuner_relayState;
unsigned long tuner_lastPulseTime;
double tuner_pulseValue;
bool autotuneInProgress;

// The mass (in grams) of each of the material samples
double refMass = 1, sampMass = 1;

/**
 * Returns true if anything violates the safety limits
 */
bool checkSafetyLimits()
{
  bool exceededTempLimit = (refTemperature > MAX_TEMPERATURE) || (sampTemperature > MAX_TEMPERATURE);
  // More complicated checks will be added in the future
  return exceededTempLimit;
}

String csvHeader = "RTC(sec), Time(sec), Ttar(C), Tref(C), Tsam(C), Vrl(V), Vsl(V), Iref(mA), Isam(mA), Pref(mW), Psam(mW), DCref(%), DCsam(%)";

// Variables used when receiving/parsing CSV data from the serial bus
const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
char tempChars[numChars];     // temporary array for use when parsing
boolean newData = false;

/**
 * @brief Receive a full line of serial data as a character array
 *
 * Based off the code examples here: https://forum.arduino.cc/t/serial-input-basics-updated/382007/3
 */
void recvSerialData()
{
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (rc != endMarker)
    {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars)
      {
        ndx = numChars - 1;
      }
    }
    else
    {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

/**
 * Send the PID gain constants via the serial bus
 */
void sendPIDGains()
{
  // Send the char 'k' to indicate the start of the PID data set
  Serial.println('k');

  Serial.println("Kp,Ki,Kd");

  // Send each value in the expected order, separated by newlines
  Serial.print(Kp, 4);
  Serial.print(",");
  Serial.print(Ki, 4);
  Serial.print(",");
  Serial.println(Kd, 4);
}

/**
 * Parse the PID gain constants from the CSV data received via the serial bus
 */
void parsePIDGains()
{
  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ","); // get the first part - Kp
  Kp = atof(strtokIndx);               // copy it to the global variable

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  Ki = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  Kd = atof(strtokIndx);

  // Update the PID gains
  refPID.setGains(Kp, Ki, Kd);
  sampPID.setGains(Kp, Ki, Kd);
}

/**
 * Receive the PID gain constants via the serial bus
 */
void recvPIDGains()
{
  unsigned long startTime = micros();
  while (!newData && (micros() - startTime) < MAX_SERIAL_WAIT_TIME)
  {
    recvSerialData();

    if (newData)
    {
      // this temporary copy is necessary to protect the original data
      //   because strtok() used in parseData() replaces the commas with \0
      strcpy(tempChars, receivedChars);
      parsePIDGains();
    }
  }
  // Send the PID gain constants to confirm that the values were received
  // properly
  sendPIDGains();
  newData = false;
}

/**
 * Run the PID algorithm and update the PWM outputs
 */
void refreshPID()
{
  if (!autotuneInProgress)
  {
    // Run the PID algorithm
    refPID.run();
    // Update the PWM output
    analogWrite(REF_HEATER_PIN, refPIDOutput);
    // Store the latest duty cycle
    refDutyCycle = refPIDOutput * byteToPercent;

    // Run the PID algorithm
    sampPID.run();
    // Update the PWM output
    analogWrite(SAMP_HEATER_PIN, sampPIDOutput);
    // Store the latest duty cycle
    sampDutyCycle = sampPIDOutput * byteToPercent;
  }
}

void stopPID(uint32_t color)
{
  // Stop PID calculations and reset internal PID calculation values
  refPID.stop();
  sampPID.stop();

  // Reset the target temp to the standby state
  targetTemp = 20;

  // Set duty cycle to zero
  refPIDOutput = sampPIDOutput = 0;
  refDutyCycle = sampDutyCycle = 0;

  // Turn off the PWM output
  analogWrite(REF_HEATER_PIN, OUTPUT_MIN);
  analogWrite(SAMP_HEATER_PIN, OUTPUT_MIN);

  neopixel.fill(color);
  neopixel.show();

  // Send the char 'x' to indicate that PID was stopped
  Serial.println("x");

  // Send the PID gain constants via the serial bus
  sendPIDGains();
  // Send the temperature control parameters via the serial bus
  sendControlParameters();
}

void endAutotune(PIDAutotuner *tuner, uint32_t color)
{
  stopPID(color);

  // Force the tuner to stop on the next
  tuner->setTuningCycles(1);

  // Get PID gains - set your PID controller's gains to these
  Kp = tuner->getKp();
  Ki = tuner->getKi();
  Kd = tuner->getKd();

  // Send the old PID gain constants via the serial bus
  sendPIDGains();

  autotuneInProgress = false;
}

/**
 * Simulate the temperature control loop while running the PID autotuner
 */
void autotunePID()
{
  // Send the char 'a' to indicate the start of autotune
  Serial.println('a');

  char fileName[15];
  strcpy(fileName, "/TUNE00.CSV");
  for (uint8_t i = 0; i < 100; i++) {
    fileName[5] = '0' + i/10;
    fileName[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(fileName)) {
      break;
    }
  }

  // Open a file for the data
  dataFile = SD.open(fileName, FILE_WRITE);

  // Check if the file exists - if not, then write a header line for the data
  if (!dataFile.size())
  {
    dataFile.println(csvHeader);
    dataFile.flush();
  }

  // Stop PID calculations and reset internal PID calculation values
  refPID.stop();
  sampPID.stop();

  // Set the target temperature for PID tuning
  targetTemp = 120;

  PIDAutotuner tuner = PIDAutotuner();

  // Set the target value to tune to
  // This will depend on what you are tuning. This should be set to a value within
  // the usual range of the setpoint. For low-inertia systems, values at the lower
  // end of this range usually give better results. For anything else, start with a
  // value at the middle of the range.
  tuner.setTargetInputValue(targetTemp);

  // Set the loop interval in microseconds
  // This must be the same as the interval the PID control loop will run at
  tuner.setLoopInterval(LOOP_INTERVAL);

  // Set the output range
  // These are the minimum and maximum possible output values of whatever you are
  // using to control the system (Arduino analogWrite, for example, is 0-255)
  tuner.setOutputRange(OUTPUT_MIN, OUTPUT_MAX);

  // Set the Ziegler-Nichols tuning mode
  // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
  // or PIDAutotuner::ZNModeNoOvershoot. Defaults to ZNModeNoOvershoot as it is the
  // safest option.
  tuner.setZNMode(PIDAutotuner::ZNModeNoOvershoot);

  // This must be called immediately before the tuning loop
  // Must be called with the current time in microseconds
  tuner.startTuningLoop(micros());

  // Run a loop until tuner.isFinished() returns true
  unsigned long startTime = microseconds = micros();
  autotuneInProgress = true;
  while (autotuneInProgress)
  {
    digitalWrite(13, LOW);

    neopixel.fill(blue);
    neopixel.show();

    // Take an rtc time measurement
    DateTime now = rtc.now();
    rtcStamp = now.secondstime();

    // Record the time (convert microseconds to seconds)
    elapsedTime = (microseconds - startTime) / SEC_TO_MICROS;

    // Read the measurements from the sensors
    updateSensorData();

    // Calcutate the heat flow
    calculateHeatFlow();

    // Call tunePID() with the input value and current time in microseconds
    double tunerOutput = tuner.tunePID(refTemperature, microseconds);
    refPIDOutput = tunerOutput;
    sampPIDOutput = OUTPUT_MIN;
    analogWrite(REF_HEATER_PIN, refPIDOutput);
    analogWrite(SAMP_HEATER_PIN, sampPIDOutput);
    refDutyCycle = refPIDOutput * byteToPercent;
    sampDutyCycle = sampPIDOutput * byteToPercent;

    // Generate a CSV string of all the experiment data
    String csvString = generateCSVString();

    // Send data out via Serial bus
    sendData(csvString);

    // Write data out to SD card file
    writeToFile(&dataFile, fileName, csvString);

    if (tuner.isFinished())
      endAutotune(&tuner, green);

    if (checkSafetyLimits())
      endAutotune(&tuner, yellow);

    // Check for interupts from the UI
    if (Serial.available())
    {
      digitalWrite(13, HIGH);

      // Read the incoming data as a char
      int inByte = Serial.read();

      switch (inByte)
      {
      case 'x':
        // Received stop command
        endAutotune(&tuner, red);
        break;
      default:
        break;
      }
    }

    // This loop must run at the same speed as the PID control loop being tuned
    while ((micros() - microseconds) < LOOP_INTERVAL)
      ; // busy wait

    unsigned long prevMicroseconds = microseconds;
    microseconds += LOOP_INTERVAL;

    if ((microseconds - startTime) < (prevMicroseconds - startTime))
    {
      // Time overflow error, experiment exceeded 70 minutes
      endAutotune(&tuner, red);
    }
  }

  // Close the file after the end of the loop
  dataFile.close();
}

/**
 * Reset the temperature control parameters to the default values
 */
void resetControlParameters()
{
  startTemp[0] = 30;
  endTemp[0] = 35;
  rampUpRate[0] = 5;
  holdTime[0] = 180;
  for (int i = 1; i < MAX_PARAMETERS; i++)
  {
    startTemp[i] = 0;
    endTemp[i] = 0;
    rampUpRate[i] = 0;
    holdTime[i] = 0;
  }
  numParams = 0;
}

/**
 * Send the temperature control parameters via the serial bus
 */
void sendControlParameters()
{
  // Send the char 'c' to indicate the start of config data set
  Serial.println('c');

  Serial.println("StartTemp(C),EndTemp(C),RampUpRate(C/min),HoldTime(sec)");

  for (int i = 0; i < numParams; i++)
  {
    // Send each value in the expected order, separated by newlines
    Serial.print(startTemp[i]);
    Serial.print(",");
    Serial.print(endTemp[i]);
    Serial.print(",");
    Serial.print(rampUpRate[i]);
    Serial.print(",");
    Serial.println(holdTime[i]);
  }
}

/**
 * Parse the temperature control parameters from the CSV data received via the
 * serial bus
 */
void parseControlParameters()
{
  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");     // get the first part - startTemp
  startTemp[numParams] = atof(strtokIndx); // copy it to the global variable

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  endTemp[numParams] = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  rampUpRate[numParams] = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  holdTime[numParams] = atof(strtokIndx);

  if (numParams < MAX_PARAMETERS)
  {
    numParams++;
  }
}

/**
 * Receive the temperature control parameters via the serial bus
 */
void recvControlParameters()
{
  unsigned long startTime = micros();
  while (!newData && (micros() - startTime) < MAX_SERIAL_WAIT_TIME)
  {
    recvSerialData();

    if (newData)
    {
      // this temporary copy is necessary to protect the original data
      //   because strtok() used in parseData() replaces the commas with \0
      strcpy(tempChars, receivedChars);
      parseControlParameters();
    }
  }
  // Send the temperature control parameters to confirm that the values were
  // received properly
  sendControlParameters();
  newData = false;
}

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

    // Refresh the PID calculations and PWM output
    refreshPID();

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

  // Refresh the PID calculations and PWM output
  refreshPID();
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

  // Refresh the PID calculations and PWM output
  refreshPID();
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

  // Refresh the PID calculations and PWM output
  refreshPID();
}

/**
 * Calcutate the target temperature
 */
void updateTargetTemperature(int cycleCounter)
{
  if (startCounter < TARGET_COUNTER_THRESHOLD)
  {
    targetTemp = startTemp[cycleCounter];
    if ((refPID.atSetPoint(MINIMUM_ACCEPTABLE_ERROR)) &&
        (sampPID.atSetPoint(MINIMUM_ACCEPTABLE_ERROR)))
    {
      startCounter++;
    }
    else
    {
      startCounter = 0;
    }
    rampUpStartTime = microseconds;
    if (startCounter == TARGET_COUNTER_THRESHOLD)
    {
      // Reset the PID when transitioning from constant target temperature to
      // ramp-up heating
      refPID.reset();
      sampPID.reset();
    }
  }
  else
  {
    if (endTemp[cycleCounter] > startTemp[cycleCounter])
    {
      targetTemp = startTemp[cycleCounter] + (microseconds - rampUpStartTime) * rampUpRate[cycleCounter] / MIN_TO_MICROS;
      if (targetTemp > endTemp[cycleCounter])
      {
        targetTemp = endTemp[cycleCounter];
      }
    }
    else if (endTemp[cycleCounter] < startTemp[cycleCounter])
    {
      targetTemp = startTemp[cycleCounter] - (microseconds - rampUpStartTime) * rampUpRate[cycleCounter] / MIN_TO_MICROS;
      if (targetTemp < endTemp[cycleCounter])
      {
        targetTemp = endTemp[cycleCounter];
      }
    }
    else
    {
      targetTemp = endTemp[cycleCounter];
    }
  }

  // Prevent the target temp from exceeding the maximum
  if (targetTemp > MAX_TEMPERATURE)
  {
    targetTemp = MAX_TEMPERATURE;
  }

  // Refresh the PID calculations and PWM output
  refreshPID();
}

/**
 * @brief Generate a CSV string of all the experiment data
 *
 */
String generateCSVString()
{
  // Create a string listing each value in the expected order, separated by commas
  String csvString = "";

  csvString.concat(rtcStamp);
  csvString.concat(",    ");

  csvString.concat(elapsedTime);
  csvString.concat(",    ");
  csvString.concat(targetTemp);
  csvString.concat(",    ");

  csvString.concat(refTemperature);
  csvString.concat(",    ");
  csvString.concat(sampTemperature);
  csvString.concat(",    ");
  /*
    csvString.concat(refShuntVoltage_mV);
    csvString.concat(",    ");
    csvString.concat(sampShuntVoltage_mV);
    csvString.concat(",    ");

    csvString.concat(refBusVoltage_V);
    csvString.concat(",    ");
    csvString.concat(sampBusVoltage_V);
    csvString.concat(",    ");
  */
  csvString.concat(refLoadVoltage_V);
  csvString.concat(",    ");
  csvString.concat(sampLoadVoltage_V);
  csvString.concat(",    ");

  csvString.concat(refCurrent_mA);
  csvString.concat(",    ");
  csvString.concat(sampCurrent_mA);
  csvString.concat(",    ");

  csvString.concat(refPower);
  csvString.concat(",    ");
  csvString.concat(sampPower);
  csvString.concat(",    ");
  /*
    csvString.concat(refHeatFlow);
    csvString.concat(",    ");
    csvString.concat(sampHeatFlow);
    csvString.concat(",    ");
  */
  csvString.concat(refDutyCycle);
  csvString.concat(",    ");
  csvString.concat(sampDutyCycle);

  // Return the CSV data string
  return csvString;
}

/**
 * Send the latest measurement data via the serial bus
 */
void sendData(String csvString)
{
  // Send a line of CSV column headers
  Serial.println(csvHeader);
  // Send the line of CSV data
  Serial.println(csvString);
}

// Write all the relevant data to an SD card file
void writeToFile(File *dataFile, String fileName, String csvString)
{
  unsigned long writeStartTime, writeEndTime; //! DEBUG

  writeStartTime = millis(); //! DEBUG

  // if the file opened okay, we can write to it:
  if (dataFile)
  {
    Serial.print("Writing to " + fileName + " ...");
    dataFile->println(csvString);
    dataFile->flush();
    Serial.println("done.");
  }
  else
  {
    Serial.println("Error opening " + fileName + " file");
  }

  writeEndTime = millis(); //! DEBUG

  double writeDuration = ((writeEndTime - writeStartTime) / 1000.0);          //! DEBUG
  Serial.println("File write duration: " + String(writeDuration) + " (sec)"); //! DEBUG
}

/**
 * Temperature control loop
 */
void controlLoop()
{
  // Send the char 's' to indicate the start of control loop
  Serial.println('s');

  char fileName[15];
  strcpy(fileName, "/DATA00.CSV");
  for (uint8_t i = 0; i < 100; i++) {
    fileName[5] = '0' + i/10;
    fileName[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(fileName)) {
      break;
    }
  }

  // Open a file for the data
  dataFile = SD.open(fileName, FILE_WRITE);

  // Check if the file exists - if not, then write a header line for the data
  if (!dataFile.size())
  {
    dataFile.println(csvHeader);
    dataFile.flush();
  }

  refPID.reset();
  sampPID.reset();

  unsigned long startTime = rampUpStartTime = holdStartTime = microseconds = micros();
  bool emergencyStop = false;
  for (int cycleCounter = 0; cycleCounter < numParams; cycleCounter++)
  {
    startCounter = endCounter = 0;
    bool controlLoopState = true;
    while (controlLoopState && !emergencyStop)
    {
      digitalWrite(13, LOW);

      neopixel.fill(blue);
      neopixel.show();

      // Take an rtc time measurement
      DateTime now = rtc.now();
      rtcStamp = now.secondstime();

      // Record the time (convert to seconds)
      elapsedTime = (microseconds - startTime) / SEC_TO_MICROS;

      // Read the measurements from the sensors
      updateSensorData();

      // Calcutate the heat flow
      calculateHeatFlow();

      // Calculate the new target temperature
      updateTargetTemperature(cycleCounter);

      // Generate a CSV string of all the experiment data
      String csvString = generateCSVString();

      // Send data out via Serial bus
      sendData(csvString);

      // Write data to SD card file
      writeToFile(&dataFile, fileName, csvString);

      // Check loop exit conditions
      if (targetTemp == endTemp[cycleCounter])
      {
        if (endCounter < TARGET_COUNTER_THRESHOLD)
        {
          // if (endCounter == 0)
          // {
          //   // Reset the PID when transitioning from ramp-up heating to constant
          //   // target temperature
          //   refPID.reset();
          //   sampPID.reset();
          // }
          if ((refPID.atSetPoint(MINIMUM_ACCEPTABLE_ERROR)) &&
              (sampPID.atSetPoint(MINIMUM_ACCEPTABLE_ERROR)))
          {
            endCounter++;
          }
          else
          {
            endCounter = 1;
          }
          holdStartTime = microseconds;
        }
        else if ((microseconds - holdStartTime) > (holdTime[cycleCounter] * SEC_TO_MICROS))
        {
          stopPID(green);
          controlLoopState = false;
        }
      }

      if (checkSafetyLimits())
      {
        stopPID(yellow);
        emergencyStop = true;
      }

      // Check for interupts from the UI
      if (Serial.available())
      {
        digitalWrite(13, HIGH);

        // Read the incoming data as a char
        int inByte = Serial.read();

        switch (inByte)
        {
        case 'x':
          // Received stop command
          stopPID(red);
          emergencyStop = true;
          break;
        default:
          break;
        }
      }

      while ((micros() - microseconds) < LOOP_INTERVAL)
      {
        // Refresh the PID calculations and PWM output
        refreshPID();
      } // busy wait

      unsigned long prevMicroseconds = microseconds;
      microseconds += LOOP_INTERVAL;

      if ((microseconds - startTime) < (prevMicroseconds - startTime))
      {
        // Time overflow error, experiment exceeded 70 minutes
        stopPID(red);
        emergencyStop = true;
      }
    }
  }

  // Close the file after the end of the loop
  dataFile.close();
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

  // Turn off the PWM output
  analogWrite(REF_HEATER_PIN, OUTPUT_MIN);
  analogWrite(SAMP_HEATER_PIN, OUTPUT_MIN);

  // Set duty cycle to zero
  refPIDOutput = sampPIDOutput = 0;
  refDutyCycle = sampDutyCycle = 0;

  // Generate a CSV string of all the experiment data
  String csvString = generateCSVString();

  // Send data out via Serial bus
  sendData(csvString);
}

void setup()
{
  // Set the relay output pins
  pinMode(REF_HEATER_PIN, OUTPUT);
  pinMode(SAMP_HEATER_PIN, OUTPUT);

  analogWrite(REF_HEATER_PIN, OUTPUT_MIN);
  analogWrite(SAMP_HEATER_PIN, OUTPUT_MIN);

  Serial.begin(9600);
  while (!Serial)
  {
    // will pause Zero, Leonardo, etc until serial console opens
    delay(1);
  }

  // Initialize the SD card.
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1)
      ;
  }
  Serial.println("card initialized.");

  // Set up RTC
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1)
      delay(10);
  }
  if (!rtc.initialized() || rtc.lostPower())
  {
    Serial.println("RTC is NOT initialized, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  rtc.start();

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
  analogWriteResolution(ANALOG_RESOLUTION);

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

  // if temperature is more than 4 degrees below or above setpoint, OUTPUT will
  // be set to min or max respectively
  refPID.setBangBang(BANG_RANGE);
  sampPID.setBangBang(BANG_RANGE);
  // set PID update interval (Default is 1000)
  refPID.setTimeStep(PID_UPDATE_INTERVAL);
  sampPID.setTimeStep(PID_UPDATE_INTERVAL);

  // NeoPixel initialization
  neopixel.begin();
  neopixel.show(); // Initialize all pixels to 'off'

  // Set PID gain constants to default values
  Kp = 350.0000;
  Ki = 4.0000;
  Kd = 5.0000;

  // Update the PID gains
  refPID.setGains(Kp, Ki, Kd);
  sampPID.setGains(Kp, Ki, Kd);

  // Set temperature control parameters to default values
  resetControlParameters();

  targetTemp = startTemp[0];

  standbyCounter = 0;

  // Set the sample masses to default values
  refMass = 1.0;
  sampMass = 1.0;

  standbyTime = 0;
}

void loop()
{
  digitalWrite(13, HIGH); // Blink the LED
  delay(STANDBY_LOOP_INTERVAL / 2);

  neopixel.clear();
  neopixel.show();

  if (Serial.available())
  {
    digitalWrite(13, HIGH);

    // Read the incoming data as a char
    int inByte = Serial.read();

    switch (inByte)
    {
    case 'a':
      // Received autotuner command
      neopixel.fill(blue);
      neopixel.show();
      // Run the PID autotuner
      autotuneInProgress = true;
      autotunePID();
      autotuneInProgress = false;
      break;
    case 'i':
      // Received initialization command
      neopixel.fill(cyan);
      neopixel.show();
      // Reset the parameters to the default values
      resetControlParameters();
      // Send the PID gain constants via the serial bus
      sendPIDGains();
      // Send the temperature control parameters via the serial bus
      sendControlParameters();
      break;
    case 'l':
      // Received load control parameters command
      neopixel.fill(cyan);
      neopixel.show();
      // Receive the temperature control parameters via the serial bus
      recvControlParameters();
      break;
    case 'p':
      // Received load PID gains command
      neopixel.fill(cyan);
      neopixel.show();
      // Receive the PID gain constants via the serial bus
      recvPIDGains();
      break;
    case 's':
      // Received start command
      neopixel.fill(blue);
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
    standbyCounter = 0;
  }
  else if ((standbyCounter % 10UL) == 0)
  {
    standbyData();
    standbyCounter = 0;
  }
  standbyCounter++;

  digitalWrite(13, LOW); // Blink the LED

  while ((millis() - standbyTime) < STANDBY_LOOP_INTERVAL)
    ; // busy wait

  standbyTime += STANDBY_LOOP_INTERVAL;
}
