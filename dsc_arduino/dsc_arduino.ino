#include <AutoPID.h>
#include <VariableTimedAction.h>

#define SENSOR_VOLTAGE 5
#define PWM_RESOLUTION 255.0
double byte2volts = (SENSOR_VOLTAGE / PWM_RESOLUTION);

#define REF_TEMP_PROBE_PIN A1
#define REF_CURRENT_SENS_PIN A2
#define SAMP_CURRENT_SENS_PIN A3
#define SAMP_TEMP_PROBE_PIN A4
#define Ref_Heater_PIN 10
#define Samp_Heater_PIN 9

//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 10
#define KP 0.02
#define KI 0.0001
#define KD 0.002

int timedActionInterval = OUTPUT_MAX;

// Max allowable temperature
// If the either temperature exceeds this value, the PWM duty cycle will be set
// set to zero
#define MAX_TEMPERATURE 300

// global variable for holding temperature and current sensor readings
double refTemperature, sampTemperature, refCurrent, sampCurrent;

// target temperature and temp control parameters
double targetTemp = 25;
double startTemp = 25;
double endTemp = 30;
double rampUpRate = 5;
double holdTime = 0;

// PID output values
double refOutputVal, sampOutputVal;

//input/output variables passed by reference, so they are updated automatically
AutoPID refPID(&refTemperature, &targetTemp, &refOutputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID sampPID(&sampTemperature, &targetTemp, &sampOutputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

unsigned long startTime, latestTime, elapsedTime; // tracks clock time of sensor readings

class HeaterPWM : public VariableTimedAction
{
public:
  HeaterPWM(int pin)
      //sets the pin to be used for the heater
      : pin(pin)
  {
    //sets the pin mode
    pinMode(pin, OUTPUT);
  }

  int duration = 0;

private:
  //stores the pin of this heater
  int pin;

  unsigned long run()
  {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
    //delay(OUTPUT_MAX - duration);
    // The VariableTimedAction interval will account for the time off

    //returns the amount in seconds to wait before executing the next event
    //if 0 is returned, then the previous interval is maintained
    return 0;
  }
};

//The HeaterPWM objects that control the heaters
HeaterPWM refHeaterPWM(Ref_Heater_PIN);
HeaterPWM sampHeaterPWM(Samp_Heater_PIN);

//Temperature controller class is responsible for reading the inputs from the sensors and updating the duty cycle of the heaters
class TemperatureController : public VariableTimedAction
{
public:
  TemperatureController()
  {
    // Set the temperature and current sensor pins
    pinMode(REF_TEMP_PROBE_PIN, INPUT);
    pinMode(SAMP_TEMP_PROBE_PIN, INPUT);
    pinMode(REF_CURRENT_SENS_PIN, INPUT);
    pinMode(SAMP_CURRENT_SENS_PIN, INPUT);

    //if temperature is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
    refPID.setBangBang(4);
    sampPID.setBangBang(4);
    //set PID update interval to 4000ms
    refPID.setTimeStep(4000);
    sampPID.setTimeStep(4000);
  }

private:
  unsigned long run()
  {
    // Record the time
    latestTime = millis();
    elapsedTime = latestTime - startTime;

    // Read the measurements from the sensors
    readSensors();

    // Calculate the error between the latest temperatures and the target temperature

    // Caclutate the heat flow

    // Calculate the new target temperature
    updateTargetTemperature();

    // Run the PID algorithm
    if (refTemperature < MAX_TEMPERATURE)
    {
      refPID.run();
    }
    else
    {
      refOutputVal = 0;
    }
    if (sampTemperature < MAX_TEMPERATURE)
    {
      sampPID.run();
    }
    else
    {
      sampOutputVal = 0;
    }

    // Update the PWM
    refHeaterPWM.duration = 10 * refOutputVal;
    sampHeaterPWM.duration = 10 * sampOutputVal;
    /*
    analogWrite(Ref_Heater_PIN, refOutputVal);
    analogWrite(Samp_Heater_PIN, sampOutputVal);
    */

    // Send data out via Serial bus
    sendData();

    return 0;
  }

  /*
   * Reads the values from each of the sensor pins and converts them to the
   * appropriate units, storing the result in the global variables
   */
  void readSensors()
  {
    // Read the encoded voltage signals from the sensors
    int refTempVal = analogRead(REF_TEMP_PROBE_PIN);
    int sampTempVal = analogRead(SAMP_TEMP_PROBE_PIN);
    int refCurrentVal = analogRead(REF_CURRENT_SENS_PIN);
    int sampCurrentVal = analogRead(SAMP_CURRENT_SENS_PIN);

    // Convert the analog values into voltages
    double refTempVoltage = refTempVal * byte2volts;
    double sampTempVoltage = sampTempVal * byte2volts;
    double refCurrentVoltage = refCurrentVal * byte2volts;
    double sampCurrentVoltage = sampCurrentVal * byte2volts;

    // Convert the voltage reading into appropriate units
    refTemperature = 0; //TODO: Put in the conversion equations
  }

  void updateTargetTemperature()
  {
    if (((endTemp > startTemp) && (targetTemp < endTemp)) || ((endTemp < startTemp) && (targetTemp > endTemp)))
    {
      targetTemp = startTemp + (rampUpRate / 60) * (latestTime - startTime);
    }
    else
    {
      targetTemp = endTemp;
    }
  }

  void sendData()
  {
    Serial.print("elapsed time (ms)");
    Serial.println(elapsedTime);

    Serial.print("ref_tmp");
    Serial.println(refTemperature);

    Serial.print("samp_tmp");
    Serial.println(refTemperature);

    Serial.print("ref_pwm");
    Serial.println(refOutputVal);

    Serial.print("samp_pwm");
    Serial.println(refOutputVal);
  }
};

TemperatureController temperatureController;

void controlLoop()
{
  digitalWrite(13, HIGH);

  refHeaterPWM.start(timedActionInterval);
  sampHeaterPWM.start(timedActionInterval);
  temperatureController.start(0.1 * timedActionInterval);

  while (true)
  {
    // Check for interupts from the UI
    if (Serial.available())
    {
      Serial.println(Serial.read());
    }

    VariableTimedAction::updateActions();

    //TODO
    // Check loop exit conditions
  }

  refHeaterPWM.stop();
  sampHeaterPWM.stop();
  temperatureController.stop();
}

void setup()
{
  Serial.begin(9600);

  pinMode(13, OUTPUT);
}

void loop()
{
  digitalWrite(13, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(500);             // wait for a second
  digitalWrite(13, LOW);  // turn the LED off by making the voltage LOW
  delay(500);             // wait for a second

  if (Serial.available())
  {
    Serial.println(Serial.read());

    //switch case when start command, call the control loop function
    //switch Serial.read();
  }
}
