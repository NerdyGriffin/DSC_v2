#include <AutoPID.h>

// Feather M0 Express board pinouts
#define REF_TEMP_PROBE_PIN A1
#define REF_CURRENT_SENS_PIN A2
#define SAMP_CURRENT_SENS_PIN A3
#define SAMP_TEMP_PROBE_PIN A4
#define Ref_Heater_PIN 11
#define Samp_Heater_PIN 10

// PID settings and gains
#define PULSE_WIDTH 100
#define KP 0.02
#define KI 0.0001
#define KD 0.002

// Analog signal to voltage conversion factor
#define SENSOR_VOLTAGE 5
#define PWM_RESOLUTION 255.0
double byte2volts = (SENSOR_VOLTAGE / PWM_RESOLUTION);

// The constant voltage supplied by the output of the transformers
#define HEATING_COIL_VOLTAGE 24

// Max allowable temperature
// If the either temperature exceeds this value, the PWM duty cycle will be set
// set to zero
#define MAX_TEMPERATURE 300

// target temperature and temp control parameters
double targetTemp = 25;
double startTemp = 25;
double endTemp = 30;
double rampUpRate = 5;
double holdTime = 0;

unsigned long startTime, latestTime, elapsedTime; // tracks clock time
unsigned long rampUpStartTime;

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

int matlabData;

/*
 * Reads the values from each of the sensor pins and converts them to the
 * appropriate units, storing the result in the global variables
 */
void readSensors(double *refTemperature, double *sampTemperature, double *refCurrent, double *sampCurrent)
{
  // Read the encoded voltage signals from the sensors
  int refTempSignal = analogRead(REF_TEMP_PROBE_PIN);
  int sampTempSignal = analogRead(SAMP_TEMP_PROBE_PIN);
  int refCurrentSignal = analogRead(REF_CURRENT_SENS_PIN);
  int sampCurrentSignal = analogRead(SAMP_CURRENT_SENS_PIN);

  // Convert the analog values into voltages
  double refTempVoltage = refTempSignal * byte2volts;
  double sampTempVoltage = sampTempSignal * byte2volts;
  double refCurrentVoltage = refCurrentSignal * byte2volts;
  double sampCurrentVoltage = sampCurrentSignal * byte2volts;

  // Convert the voltage reading into appropriate units
  *refTemperature = 0; //TODO: Put in the conversion equations
  *sampTemperature = 0;
  *refCurrent = 0;
  *sampCurrent = 0;
}

////void calculateDutyCycle(double* refPWMDutyCycle, double* sampPWMDutyCycle, )

void calculateHeatFlow(double *refHeatFlow, double *sampHeatFlow, double refCurrent, double sampCurrent)
{
  *refHeatFlow = refCurrent * HEATING_COIL_VOLTAGE / refMass;
  *sampHeatFlow = sampCurrent * HEATING_COIL_VOLTAGE / sampMass;
}

void updateTargetTemperature(double *targetTemp, double startTemp, double endTemp, double rampUpRate, double latestTime)
{
  if (((*targetTemp < startTemp) && (startTemp < endTemp)) ||
      ((*targetTemp > startTemp) && (startTemp > endTemp)))
  {
    *targetTemp = startTemp;
    rampUpStartTime = millis();
  }
  else if (((*targetTemp < endTemp) && (endTemp > startTemp)) ||
           ((*targetTemp > endTemp) && (endTemp < startTemp)))
  {
    *targetTemp = startTemp + (rampUpRate / 60) * (latestTime - rampUpStartTime);
  }
  else
  {
    *targetTemp = endTemp;
  }
}

void sendData()
{
  Serial.print("Elapsed time (ms): ");
  Serial.println(elapsedTime);

  Serial.print("Target Temperature (C): ");
  Serial.println(targetTemp);

  Serial.print("Ref Temperature (C): ");
  Serial.println(refTemperature);

  Serial.print("Samp Temperature (C): ");
  Serial.println(sampTemperature);

  Serial.print("Ref Current (A): ");
  Serial.println(refCurrent);

  Serial.print("Ref Current (A): ");
  Serial.println(sampCurrent);

  Serial.print("Ref PWM Duty Cycle: ");
  Serial.println(refPID.getPulseValue() / PULSE_WIDTH);

  Serial.print("Samp PWM Duty Cycle: ");
  Serial.println(sampPID.getPulseValue() / PULSE_WIDTH);
}

void controlLoop()
{
  digitalWrite(13, HIGH);
  startTime = millis();
  while (true)
  {
    digitalWrite(13, HIGH);
    // Check for interupts from the UI
    if (Serial.available())
    {
      // read the incoming data as a string
      matlabData = Serial.read();
      Serial.print("Recieved data: ");
      Serial.println(matlabData);

      switch (matlabData)
      {
      case 5:
        return;
        break;
      default:
        break;
      }
    }

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

    //TODO
    // Check loop exit conditions
  }
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
  refPID.setBangBang(4);
  sampPID.setBangBang(4);
  //set PID update interval to 1000ms
  refPID.setTimeStep(1000);
  sampPID.setTimeStep(1000);
}

void loop()
{
  digitalWrite(13, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(500);             // wait for a second
  digitalWrite(13, LOW);  // turn the LED off by making the voltage LOW
  delay(500);             // wait for a second

  if (Serial.available())
  {
    digitalWrite(13, HIGH);
    
    // read the incoming data as a string
    matlabData = Serial.read();
    Serial.print("Recieved data: ");
    Serial.println(matlabData);

    switch (matlabData)
    {
    case 3:
      //controlLoop;
      break;
    default:
      break;
    }
  }
}
