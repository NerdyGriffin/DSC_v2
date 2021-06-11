// Feather M0 Express board pinouts
#define REF_TEMP_PROBE_PIN A1
#define REF_CURRENT_SENS_PIN A2
#define SAMP_CURRENT_SENS_PIN A3
#define SAMP_TEMP_PROBE_PIN A4
#define Ref_Heater_PIN 11
#define Samp_Heater_PIN 10

// Number of samples to average the reading over
// Change this to make the reading smoother... but beware of buffer overflows!
const int avgSamples = 100;
//! Must NOT exceed 2^(32 - 2*ANALOG_RESOLUTION)
//! to prevent overflow error during summation

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
#define CURRENT_SENSOR_SENS 0.4    // Sensitivity (Sens) 100mA per 250mV = 0.4
//#define CURRENT_SENSOR_VREF 1650.0 // Output voltage with no current: ~ 1650mV or 1.65V
//#define CURRENT_SENSOR_VREF 2500.0 // Output voltage with no current: ~ 2500mV or 2.5V

// Max allowable temperature.
// If the either temperature exceeds this value, the PWM duty cycle will be set
// set to zero
#define MAX_TEMPERATURE 300

// global variables for holding temperature and current sensor readings
double refTemperature, sampTemperature;
double refCurrent, sampCurrent;
double refHeatFlow, sampHeatFlow;

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
    sensorValues[0] += analogRead(REF_TEMP_PROBE_PIN);
    sensorValues[1] += analogRead(SAMP_TEMP_PROBE_PIN);
    // Current sensor values are squared for RMS calculation
    sensorValues[2] += sq(analogRead(REF_CURRENT_SENS_PIN) - analogMidpoint);
    sensorValues[3] += sq(analogRead(SAMP_CURRENT_SENS_PIN) - analogMidpoint);

    // Wait 2 milliseconds before the next loop for the analog-to-digital
    // converter to settle after the last reading
    delay(2);
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //  while (!Serial);
  Serial.println("Ref Temp (V),Ref Temp (C),Samp Temp (V),Samp Temp (C),Ref Current (mV),Ref Current (mA),Samp Current (mV),Samp Current (mA)");

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
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(13, HIGH); // turn the LED on (HIGH is the voltage level)
  //  delay(500);             // wait for a second

  // Take a measurement of the sensor with expected current reading 0 mA
  getSensorValues();

  // The voltage is in millivolts
  double refTempVoltage = sensorValues[0] * byteToMillivolts;
  double sampTempVoltage = sensorValues[1] * byteToMillivolts;
  double refCurrentVoltage = sensorValues[2] * byteToMillivolts;
  double sampCurrentVoltage = sensorValues[3] * byteToMillivolts;

  // Convert the voltage readings into appropriate units.
  // This will calculate the temperature (in Celcius)
  refTemperature = (refTempVoltage - AMPLIFIER_VOLTAGE_OFFSET) / AMPLIFIER_CONVERSION_FACTOR;
  sampTemperature = (sampTempVoltage - AMPLIFIER_VOLTAGE_OFFSET) / AMPLIFIER_CONVERSION_FACTOR;
  // This will calculate the actual current (in mA)
  // Using the Vref and sensitivity settings you configure
  refCurrent = (refCurrentVoltage) * CURRENT_SENSOR_SENS;
  sampCurrent = (sampCurrentVoltage) * CURRENT_SENSOR_SENS;

  //  Serial.println("RefTemp(V),RefTemp(C),SampTemp(V),SampTemp(C),RefCurrent(mV),RefCurrent(mA),SampCurrent(mV),SampCurrent(mA)");
  Serial.println("RefTemp(C),SampTemp(C),RefCurrent(mA),SampCurrent(mA),MaxTemp(C)");

  //  Serial.print(refTempVoltage);
  //  Serial.print(",");
  Serial.print(refTemperature);
  Serial.print(",");
  //  Serial.print(sampTempVoltage);
  //  Serial.print(",");
  Serial.print(sampTemperature);
  Serial.print(",");
  //  Serial.print(refCurrentVoltage*0.001);
  //  Serial.print(",");
  Serial.print(refCurrent);
  Serial.print(",");
  //  Serial.print(sampCurrentVoltage*0.001);
  //  Serial.print(",");
  Serial.print(sampCurrent);
  Serial.print(",");
  Serial.println(300);

  digitalWrite(13, LOW);  // turn the LED off by making the voltage LOW
  if (refTemperature < MAX_TEMPERATURE)
    digitalWrite(Ref_Heater_PIN, HIGH);
  else
    digitalWrite(Ref_Heater_PIN, LOW);

  if (sampTemperature < MAX_TEMPERATURE)
    digitalWrite(Samp_Heater_PIN, HIGH);
  else
    digitalWrite(Samp_Heater_PIN, LOW);
  //  delay(500);             // wait for a second
}
