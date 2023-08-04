#include <PID.h> //https://github.com/EricLarueMartin/ArduinioPID
#include <EEPROM.h>
#include <avr/wdt.h>
#include <CmdMessenger.h> //https://github.com/dreamcat4/cmdmessenger
#include <Streaming.h>    //http://arduiniana.org/libraries/streaming/


const int HTRCycleTime = 8192;
const double MaxDutyCycle = 0.5; // limit to 50% power

const int SensorPin = A0;
const int ThermocouplePin = A1;

const int HeaterPin = 9;
const int PowerLED = 5;
const int HeaterLED = 6;

const int LedMin = 100;
const int LedMax = 200;
const int LedPeriod = 2000;

const int InterlockBlinkPeriod = 500; // period in ms for blinking heater LED if thermocouple interlock not met

const double Kp = 0.1;
const double Ki = 0.01;
const double Kd = 0.01;
const double SetPoint = 8.0; // should be about 8 psi

// use linear approximation since we really only need a general idea of temeprature
// temperature = double(iTemperatureADCValue) * slopeADCtoK + offsetADCtoK;
double slopeADCtoK = 1.5/1.024; // deg K / ADC
double offsetADCtoK = -100.0; // offset to ADC * slope for 0 K
double HighTemperatureTrip = 500.0; // don't run heater if temperature rises above this setting in Kelvin.
double HighTemperatureReset = 100.0; // reset high temperature trip if drops below this setting in Kelvin.

bool highTemperatureTripped = true; // should start out below 100 and immediately reset

double MinTemperature = 50.0; // don't run heater if it's colder than this setting in Kelvin.

// strings take up memory, so those that occur in multiple places should refer to a global string
const char PressureMessage[] = {"Measured pressure is "};
const char MilliSecondsMessage[] = {" ms."};
const char PSIGMessage[] = {" psig."};


const double PowerPeriodInMs = 60.0 / 1000.0; // 60 Hz in USA

double slopeADCtoPSIG = 0.04;
double offsetADCtoPSIG = -1.68;
double pressure;
double power = 0.0;
bool heaterOn = false;
unsigned long windowStopTime = 0;
int loopDelay = 10;
double temperature = 300;

bool humanReadable = false; // generate human readable serial outputs

PID HTRPID(&pressure, &power, SetPoint, Kp, Ki, Kd);


char field_separator      = ',';
char command_separator    = ';';
char eol          = '\r';
unsigned short inputMask  = 0x0;
unsigned short oldInputs  = 0x0;
unsigned short controlValue[10]    = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial, field_separator, command_separator);

void SaveToEEPROM( int addrEEPROM = 0 )
{
  EEPROM.put(addrEEPROM, HTRPID.Kp);
  addrEEPROM += sizeof(double);
  EEPROM.put(addrEEPROM, HTRPID.Ki);
  addrEEPROM += sizeof(double);
  EEPROM.put(addrEEPROM, HTRPID.Kd);
  addrEEPROM += sizeof(double);
  EEPROM.put(addrEEPROM, HTRPID.setPoint);
  addrEEPROM += sizeof(double);
  EEPROM.put(addrEEPROM, HTRPID.outMin);
  addrEEPROM += sizeof(double);
  EEPROM.put(addrEEPROM, HTRPID.outMax);
  addrEEPROM += sizeof(double);
  EEPROM.put(addrEEPROM, slopeADCtoPSIG);
  addrEEPROM += sizeof(double);
  EEPROM.put(addrEEPROM, offsetADCtoPSIG);
  addrEEPROM += sizeof(double);
  EEPROM.put(addrEEPROM, HTRPID.sampleTime);
  addrEEPROM += sizeof(long);
  EEPROM.put(addrEEPROM, loopDelay);
}

void LoadFromEEPROM( int addrEEPROM = 0 )
{
  EEPROM.get(addrEEPROM, HTRPID.Kp);
  addrEEPROM += sizeof(double);
  EEPROM.get(addrEEPROM, HTRPID.Ki);
  addrEEPROM += sizeof(double);
  EEPROM.get(addrEEPROM, HTRPID.Kd);
  addrEEPROM += sizeof(double);
  EEPROM.get(addrEEPROM, HTRPID.setPoint);
  addrEEPROM += sizeof(double);
  EEPROM.get(addrEEPROM, HTRPID.outMin);
  addrEEPROM += sizeof(double);
  EEPROM.get(addrEEPROM, HTRPID.outMax);
  addrEEPROM += sizeof(double);
  EEPROM.get(addrEEPROM, slopeADCtoPSIG);
  addrEEPROM += sizeof(double);
  EEPROM.get(addrEEPROM, offsetADCtoPSIG);
  addrEEPROM += sizeof(double);
  EEPROM.get(addrEEPROM, HTRPID.sampleTime);
  addrEEPROM += sizeof(long);
  EEPROM.get(addrEEPROM, loopDelay);
}

void parseSerial()
{
  double fVal;
  int iVal;
  if (Serial.available() < 2) return; // if there's not at least two bytes available there's no valid incoming command
  if (Serial.peek() < '?') return; // if the incoming command is less than '?', it's not one of our human readable commands
  switch (Serial.read()) // read the first byte of the incomming data
  {
    case 'p' : // set Kp
      fVal = Serial.parseFloat();
      HTRPID.Kp = fVal;
      Serial.print(F("Set Kp to ")); // F() stores the string in program storage flash memory, leaving more dynamic memory available
      Serial.println(fVal);
      break;
    case 'i' : // set Ki
      fVal = Serial.parseFloat();
      HTRPID.Ki = fVal;
      Serial.print(F("Set Ki to "));
      Serial.println(fVal);
      break;
    case 'd' : // set Kd
      fVal = Serial.parseFloat();
      HTRPID.Kd = fVal;
      Serial.print(F("Set Kd to "));
      Serial.println(fVal);
      break;
    case 'a' : // toggle auto
      HTRPID.inAuto = !HTRPID.inAuto;
      Serial.print(F("Heater control in ")); // using this extra function call saves a whopping 6 bytes
      Serial.println(HTRPID.inAuto ? F("auto.") : F("manual."));
      break;
    case 's' : // change setpoint, this is in fraction of full scale output of ADC
      fVal = Serial.parseFloat();
      HTRPID.setPoint = fVal;
      Serial.print(F("Setpoint is set to "));
      Serial.println(fVal);
      break;
    case 'o' : // set output, this is fraction of time heater is on
      fVal = Serial.parseFloat();
      HTRPID.SetOutput(fVal);
      Serial.print(F("Set output to "));
      Serial.println(fVal);
      break;
    case 't' : // set iTerm
      fVal = Serial.parseFloat();
      HTRPID.iTerm = fVal;
      Serial.print(F("Set iTerm to "));
      Serial.println(fVal);
      break;
    case 'w' : // set cycle time
      iVal = Serial.parseInt();
      HTRPID.sampleTime = iVal;
      Serial.print(F("Set heater cycle time to "));
      Serial.print(iVal);
      Serial.println(MilliSecondsMessage);
      break;
    case 'L' : // set loop delay
      iVal = Serial.parseInt();
      loopDelay = iVal;
      Serial.print(F("Set loop delay time to "));
      Serial.print(iVal);
      Serial.println(MilliSecondsMessage);
      break;
    case 'm' : // set max output
      fVal = Serial.parseFloat();
      HTRPID.outMax = fVal;
      Serial.print(F("Set maximum output to "));
      Serial.println(fVal);
      break;
    case 'n' : // set minimum output
      fVal = Serial.parseFloat();
      HTRPID.outMin = fVal;
      Serial.print(F("Set minimum output to "));
      Serial.println(fVal);
      break;
    case 'g' : // set slopeADCtoPSIG for sensor calibration
      fVal = Serial.parseFloat();
      slopeADCtoPSIG = fVal;
      Serial.print(F("Set slope of ADC to pressure(psig) conversion to "));
      Serial.println(fVal);
      break;
    case 'z' : // set offsetADCtoPSIG for sensor calibration
      fVal = Serial.parseFloat();
      offsetADCtoPSIG = fVal;
      Serial.print(F("Set offset of ADC to pressure(psig) converstion to "));
      Serial.println(fVal);
      break;
    case 'v' : // save settings
      Serial.println(F("Saving settings to EEPROM."));
      SaveToEEPROM();
      break;
    case 'F' : // restore default settings
      EEPROM.put(0, (long)(-1));
      Serial.println(F("Default settings will be restored next reset."));
      break;

    case '?' : // display settings
      humanReadable = true; // must be someone accessing device through a terminal program, start processing serial commands
      Serial.println(F("Commands are:"));
      Serial.println(F("p # - set Kp"));
      Serial.println(F("i # - set Ki"));
      Serial.println(F("d # - set Kd"));
      Serial.println(F("a - toggle between auto and manual"));
      Serial.println(F("s # - change setpoint"));
      Serial.println(F("o # - set output power"));
      Serial.println(F("t # - set iTerm of PID controller"));
      Serial.println(F("w # - set heater cycle time in milliseconds"));
      Serial.println(F("m # - set maximum PID output"));
      Serial.println(F("n # - set minimum PID output"));
      Serial.println(F("g # - set ADC to pressure slope"));
      Serial.println(F("z # - set ADC to pressure zero offset"));
      Serial.println(F("L # - set loop delay, note that a delay longer than 8000 ms will trigger the watchdog timer"));
      Serial.println(F("v - save settings to EEPROM, will always be in auto after a reset"));
      Serial.println(F("F - restore default settings"));
      Serial.println(F("Note that the PID output is limited by adjusting the iTerm. While the heater power can only be between 0 (always off) to 1 (always on) larger limits on the PID loop ouput are useful for preventing changes to the iTerm when first determining control values."));
      Serial.print(F("Output is "));
      Serial.println(power);
      Serial.print(F("Setpoint is "));
      Serial.print(HTRPID.setPoint);
      Serial.println(PSIGMessage);
      Serial.print(F("Output range is "));
      Serial.print(HTRPID.outMin);
      Serial.print(F(" to "));
      Serial.print(HTRPID.outMax);
      Serial.println(PSIGMessage);
      Serial.print(F("Kp is "));
      Serial.println(HTRPID.Kp);
      Serial.print(F("Ki is "));
      Serial.println(HTRPID.Ki);
      Serial.print(F("Kd is "));
      Serial.println(HTRPID.Kd);
      Serial.print(F("ADC to pressure conversion equation is ADC * "));
      Serial.print(slopeADCtoPSIG);
      Serial.print(F(" + "));
      Serial.println(offsetADCtoPSIG);
      Serial.print(F("Control is in "));
      Serial.println(HTRPID.inAuto ? F("auto") : F("manual"));
      Serial.print(F("PID pTerm is "));
      Serial.println(HTRPID.pTerm);
      Serial.print(F("PID iTerm is "));
      Serial.println(HTRPID.iTerm);
      Serial.print(F("PID dTerm is "));
      Serial.println(HTRPID.dTerm);
      Serial.print(F("Heater cycle time is "));
      Serial.print(HTRPID.sampleTime);
      Serial.println(MilliSecondsMessage);
      Serial.print(F("Temperature is "));
      Serial.print(temperature);
      Serial.println(" K");
      Serial.print(F("Loop delay is "));
      Serial.print(loopDelay);
      Serial.println(MilliSecondsMessage);
      break;
  }

}

//Commands from ORCA (never sent unsolicited)
short kCmdVerison     = 1;  //1;
short kCmdReadAdcs      = 2;  //2;             --read all adcs
short kCmdReadInputs    = 3;  //3,mask;        --read input pins using mask
short kCmdWriteAnalog   = 4;  //4,pin,value;   --set pin pwm to value
short kCmdWriteOutput   = 5;  //5,pin,value;   --set output pin to value
short kCmdWriteOutputs    = 6;  //6,mask;        --set outputs based on mask
short kCmdSetControlValue = 7;  //7,chan,value;  --set control value. chan 0-9. value is unsigned short

//Messages which can be sent unsolicited to ORCA.
short kInputsChanged    = 20; //20,i0,i1,i2,...i13;
short kCustomValueChanged   = 21; //21,chan,value
short kAdcValueChanged    = 22; //22,chan,value
short kArduinoStarted   = 23;
short kUnKnownCmd     = 99;

float kSketchVersion = 1.1; //change whenever command formats change

// ------------------ C A L L B A C K  M E T H O D S -------------------------
void readAnalogValues()
{
  Serial << 2;
  for (char i = 0; i < 6; i++) Serial << "," << analogRead(i);
  Serial << eol;
}

void readInputPins()
{
  inputMask = cmdMessenger.readInt();
  Serial << kCmdReadInputs;
  for (char i = 0; i < 14; i++) {
    if (inputMask & (1 << i)) {
      if (i >= 2) {
        pinMode(i, INPUT_PULLUP);
        Serial << "," << digitalRead(i);
      }
      else Serial << ",0"; //return 0 for the serial lines
    }
    else   Serial << ",0";
  }
  Serial << eol;
}

void writeOutputPin()
{
  short pin   = cmdMessenger.readInt();
  short state = cmdMessenger.readInt();
  if (pin == HeaterPin) return; // do not allow changing state of heater pin
  if (pin >= 2 && (~inputMask & (1 << pin))) {
    pinMode(pin, OUTPUT);
    if ( state)  digitalWrite(pin, HIGH);
    else        digitalWrite(pin, LOW);
  }
  Serial << kCmdWriteOutput << "," << pin << "," << state << eol;
}


void writeOutputs()
{
  short pin;
  short outputTypeMask  = cmdMessenger.readInt() & ~inputMask; //don't write inputs
  short writeMask       = cmdMessenger.readInt() & ~inputMask;
  if (outputTypeMask) {
    for (pin = 2; pin < 14; pin++) {
      if (outputTypeMask & (1 << pin)) {
        pinMode(pin, OUTPUT);
        if ( writeMask & (1 << pin))  digitalWrite(pin, HIGH);
        else                       digitalWrite(pin, LOW);
      }
    }
  }
  else writeMask = 0;
  //echo the command back
  Serial << kCmdWriteOutputs << "," << outputTypeMask << "," << writeMask << eol;
}

void writeAnalog()
{
  short pin   = cmdMessenger.readInt();
  short state = cmdMessenger.readInt();
  if (pin >= 2 && (~inputMask & (1 << pin))) {
    pinMode(pin, OUTPUT);
    analogWrite(pin, state); //Sets the PWM value of the pin
  }
  //echo the command back
  Serial << kCmdWriteAnalog << "," << pin << "," << state << eol;
}

void setControlValue()
{
  //users can use this value in their custom code as needed.
  short chan  = cmdMessenger.readInt();
  unsigned short value = cmdMessenger.readInt();
  if (chan >= 0 && chan < 10) {
    controlValue[chan] = value;
  }
  //echo the command back
  Serial << kCmdSetControlValue << "," << chan << "," << value << eol;
}


void sketchVersion()  {
  Serial << kCmdVerison << "," << kSketchVersion << eol;
}
void unKnownCmd()     {
  Serial << kUnKnownCmd << eol;
}

// ------------------ E N D  C A L L B A C K  M E T H O D S ------------------

void setup()
{
  wdt_enable(WDTO_8S);
  delay(1000);
  analogReference(DEFAULT); // use the 5 V reference
  HTRPID.sampleTime = HTRCycleTime;
  pinMode(HeaterPin, OUTPUT);
  HTRPID.inAuto = true;
  pinMode(PowerLED, OUTPUT);//for LED
  pinMode(HeaterLED, OUTPUT);//for LED
  HTRPID.outMax = MaxDutyCycle;
  // load settings from EEPROM
  long iTest;
  EEPROM.get(0, iTest);
  if (iTest != -1) // if the EEPROM has never been written to then the stored values should be 255 in every byte, which should be -1 for a signed long int.
    LoadFromEEPROM();

  Serial.begin(115200); // Arduino Uno, Mega, with AT8u2 USB
  wdt_reset(); // reset watchdog timer before delay
  delay(1000);
  wdt_reset(); // reset watchdog timer before delay

  cmdMessenger.print_LF_CR();
  cmdMessenger.attach(kCmdVerison,     sketchVersion);
  cmdMessenger.attach(kCmdReadAdcs,    readAnalogValues);
  cmdMessenger.attach(kCmdReadInputs,  readInputPins);
  cmdMessenger.attach(kCmdWriteAnalog, writeAnalog);
  cmdMessenger.attach(kCmdWriteOutput, writeOutputPin);
  cmdMessenger.attach(kCmdWriteOutputs, writeOutputs);
  cmdMessenger.attach(kCmdSetControlValue, setControlValue);

  cmdMessenger.attach(unKnownCmd);

  //Tell the world we had a reset
  Serial << kArduinoStarted << ",Reset" << eol;
  wdt_reset(); // reset watchdog timer before delay

}

void scanInputsForChange()
{
  if (inputMask) {
    unsigned short inputs = 0;
    if (inputMask) {
      for (char i = 2; i < 14; i++) {
        if (inputMask & (1 << i)) {
          inputs |= ((unsigned short)debouncedDigitalRead(i) << i);
        }
      }
      if (inputs != oldInputs) {
        oldInputs = inputs;
        Serial << kInputsChanged << "," << inputs <<  eol;
      }
    }
  }
}

boolean         lastPinState[14];
boolean         pinState[14];
unsigned long   lastDebounceTime[14];
unsigned long   debounceDelay = 50;

boolean debouncedDigitalRead(int aPin)
{
  boolean currentValue = digitalRead(aPin);
  if (currentValue != lastPinState[aPin]) lastDebounceTime[aPin] = millis();
  if ((millis() - lastDebounceTime[aPin]) > debounceDelay) pinState[aPin] = currentValue;
  lastPinState[aPin] = currentValue;
  return pinState[aPin];
}

//--------------------------------------------------------------
void loop()
{
  static unsigned long tPowerLED = millis();
  static unsigned long tInterlockLED = millis();
  parseSerial(); // this will check for human readable command codes, if not found the next line uses the cmdMessenger the Orca sketch uses
  cmdMessenger.feedinSerialData(); //process incoming commands
  scanInputsForChange();
  wdt_reset(); // reset watchdog timer before measurement
  unsigned long now = millis();
  if (now-tPowerLED>LedPeriod) tPowerLED = now;
  if (now-tInterlockLED>InterlockBlinkPeriod) tInterlockLED = now;      
  analogWrite(PowerLED,(now-tPowerLED)/LedPeriod*(LedMax-LedMin)+LedMin);//Turns on the Green LED
  // read current pressure
  int iPressureADCValue = analogRead(SensorPin);
  int iTemperatureADCValue = analogRead(ThermocouplePin);
  pressure = double(iPressureADCValue) * slopeADCtoPSIG + offsetADCtoPSIG; // 10 bit ADC with 5V full scale, 0.5 V is 0 psig, 4.5 V is 30 psig
  temperature = double(iTemperatureADCValue) * slopeADCtoK + offsetADCtoK;// 10 bit ADC with 5 V full scale, 1.25 V is room temperature and slope is 5 mV per degree
  if (temperature > HighTemperatureTrip) highTemperatureTripped = true;
  if (temperature < HighTemperatureReset) highTemperatureTripped = false;

  // compute the PID setting with measured pressure
  // Heater power is controller with pulse width modulation.
  if (HTRPID.Compute()) // If the PID recalculated heater power, this is also the start of the heater power window
  {
    if (power > MaxDutyCycle) HTRPID.SetOutput(MaxDutyCycle);
    // turn the heater on at the beginning of the window and determine when to turn the heater off
    windowStopTime = (power > PowerPeriodInMs / HTRPID.sampleTime) ? power * HTRPID.sampleTime : 0; // if it won't be at least one cycle, don't turn on the heater
    if (windowStopTime > HTRPID.sampleTime) windowStopTime = HTRPID.sampleTime;
    if (Serial)
    {
      if (humanReadable)
      {
        Serial.print(PressureMessage);
        Serial.print(pressure);
        Serial.println(PSIGMessage);
        Serial.print(F("Temperature "));
        Serial.print(temperature);
        Serial.println('K');
        Serial.print(F("Turning heater on for "));
        Serial.print(windowStopTime);
        Serial.print(F(" out of "));
        Serial.print(HTRPID.sampleTime);
        Serial.println(MilliSecondsMessage);
        if (highTemperatureTripped || temperature < MinTemperature) {
          Serial.print("Not turning heater on as temperature is out of operation band at ");
          Serial.println(temperature);
        }
      }
      else
      {
        //customValues can be sent back to ORCA with the customValue commands
        //example:  kCustomValueChanged,channelNumber,value;
        //Serial << kCustomValueChanged << "," << 0 << "," << 123 << eol;
        Serial << kCustomValueChanged << "," << 0 << "," << iPressureADCValue << eol;
        Serial << kCustomValueChanged << "," << 1 << "," << windowStopTime << eol;
      }
    }
    heaterOn = ((windowStopTime > 0) && (!highTemperatureTripped && temperature > MinTemperature)); // heater on bool tracks if it should be on, but it doesn't actually turn on unless temperature is sufficiently low
    windowStopTime += HTRPID.lastTime; // The heater pulse uses the same timing variable as the PID calculation to ensure synchronization.
  }
  else // if (HTRPID.Compute())
  {
    // If the heater is on and it's time to turn it off, then do so
    if ((now > windowStopTime) && heaterOn)
    {
      if (Serial && humanReadable)
      {
        Serial.print(PressureMessage);
        Serial.print(pressure);
        Serial.println(F(" turning heater off."));
      }
      heaterOn = false;
    }
  }
  if (!highTemperatureTripped && temperature > MinTemperature) { // interlock met
    digitalWrite(HeaterPin, heaterOn); 
    digitalWrite(HeaterLED, heaterOn); 
  }
  else { // interlock NOT met
    digitalWrite(HeaterPin, LOW); // keep heater off
    digitalWrite(HeaterLED,((now-tInterlockLED)<(InterlockBlinkPeriod/2))); //blink the red LED     
  }
  if (loopDelay)
  {
    wdt_reset(); // reset watchdog timer before delay
    delay(loopDelay);
  }
}
