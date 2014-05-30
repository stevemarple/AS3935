#include <AsyncDelay.h>
#include <SoftWire.h>
#include <AS3935.h>

#include <Wire.h>
#include <RTCx.h>

#ifdef JTD
#include <DisableJTAG.h>
#endif

AS3935 as3935;
bool ledState = true;
AsyncDelay d;
unsigned long timestamp;

void int2Handler(void)
{
  as3935.interruptHandler();
}

void timestampCB(void)
{
  timestamp = millis();
}

void readRegs(uint8_t start, uint8_t end)
{
  for (uint8_t reg = start; reg < end; ++reg) {
    delay(50);
    uint8_t val;
    as3935.readRegister(reg, val);

    Serial.print("Reg: 0x");
    Serial.print(reg, HEX);
    Serial.print(": 0x");
    Serial.println(val, HEX);
    Serial.flush();
  }
  Serial.print("State: ");
  Serial.println(as3935.getState(), DEC);

  Serial.print("EIFR: ");
  Serial.println(EIFR, HEX);
  Serial.println("-------------");
}

void printTm(Stream &str, struct RTCx::tm *tm)
{
  Serial.print(tm->tm_year + 1900);
  Serial.print('-');
  Serial.print(tm->tm_mon + 1);
  Serial.print('-');
  Serial.print(tm->tm_mday);
  Serial.print('T');
  Serial.print(tm->tm_hour);
  Serial.print(':');
  Serial.print(tm->tm_min);
  Serial.print(':');
  Serial.print(tm->tm_sec);
  Serial.print(" yday=");
  Serial.print(tm->tm_yday);
  Serial.print(" wday=");
  Serial.println(tm->tm_wday);
}

void setup(void)
{
#ifdef JTD
  disableJTAG();
#endif

  Serial.begin(9600);
  Wire.begin();
  uint8_t addressList[] = {RTCx_MCP7941x_ADDRESS,
			   RTCx_DS1307_ADDRESS};

  // Autoprobe to find a real-time clock.
  if (rtc.autoprobe(addressList, sizeof(addressList))) {
    // Found something, hopefully a clock.
    Serial.print("Autoprobe found ");
    switch (rtc.getDevice()) {
      case RTCx::DS1307:
	Serial.print("DS1307");
	break;
    case RTCx::MCP7941x:
      Serial.print("MCP7941x");
      break;
    default:
      // Ooops. Must update this example!
      Serial.print("unknown device");
      break;
    }
    Serial.print(" at 0x");
    Serial.println(rtc.getAddress(), HEX);
  }
  else {
    // Nothing found at any of the addresses listed.
    Serial.println("No RTCx found");
    return;
  }

  // Enable the battery backup. This happens by default on the DS1307
  // but needs to be enabled on the MCP7941x.
  rtc.enableBatteryBackup();

  // rtc.clearVBAT();
  
  // Ensure the oscillator is running.
  rtc.startClock();

  if (rtc.getDevice() == RTCx::MCP7941x) {
    Serial.print("Calibration: ");
    Serial.println(rtc.getCalibration(), DEC);
    // rtc.setCalibration(-127);
  }

  struct RTCx::tm tm;
  rtc.readClock(tm);
  printTm(Serial, &tm);
  
  as3935.initialise(14, 17, 0x03, 3, true, timestampCB);
  readRegs(0, 0x09);
  as3935.start();
  d.start(1000, AsyncDelay::MILLIS);

  while (!d.isExpired())
    as3935.process();
  
  readRegs(0, 0x09);
  Serial.println("Setting registers");
  as3935.setNoiseFloor(0);
  as3935.setSpikeRejection(0);
  readRegs(0, 0x09);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);
  
  attachInterrupt(2, int2Handler, RISING);
  d.start(1000, AsyncDelay::MILLIS);

  //as3935.calibrate();
  readRegs(0, 0x09);
  Serial.println("setup() done");
}

const uint8_t bufLen = 30;
char buffer[bufLen + 1] = {'\0'};
uint8_t bufPos = 0;
unsigned long last = 0;
uint8_t count = 0;
void loop(void)
{
  struct RTCx::tm tm;

  while (Serial.available()) {
    char c = Serial.read();
    if ((c == '\r' || c == '\n' || c == '\0')) {
      if (bufPos <= bufLen && buffer[0] == 'C') {
	// Check time error
	buffer[bufPos] = '\0';
	RTCx::time_t pcTime = atol(&(buffer[1]));
	rtc.readClock(&tm);
	RTCx::time_t mcuTime = RTCx::mktime(&tm);
	Serial.print("MCU clock error: ");
	Serial.print(mcuTime - pcTime);
	Serial.println(" s");
	Serial.println("~~~~~");
      }
      if (bufPos <= bufLen && buffer[0] == 'T') {
	// Set time
	buffer[bufPos] = '\0';
	RTCx::time_t t = atol(&(buffer[1])); 
	RTCx::gmtime_r(&t, &tm);
	rtc.setClock(&tm);
	Serial.println("Clock set");
	Serial.println(&(buffer[0]));
	printTm(Serial, &tm);
	Serial.println("~~~~~");
      }
      if (bufPos <= bufLen && buffer[0] == 'X') {
	// Set calibration value
	buffer[bufPos] = '\0';
	if (rtc.getDevice() == RTCx::MCP7941x) {
	  int8_t oldCal = rtc.getCalibration();
	  char *endptr;
	  long cal = strtol(&(buffer[1]), &endptr, 0);
	  if (cal >= -127 && cal <= 127 && endptr == &buffer[bufPos]) {
	    Serial.print("Previous calibration: ");
	    Serial.println(oldCal, DEC);
	    Serial.print("Calibration: ");
	    Serial.println(cal, DEC);
	      rtc.setCalibration(cal);
	  }
	  else 
	    Serial.println("Bad value for calibration");
	}
	else {
	  Serial.println("Cannot set calibration: not a MCP7941x");
	}
      }
      if (bufPos <= bufLen && buffer[0] == 'I') {
	if (as3935.setIndoor(true))
	  Serial.println("Set to indoor mode");
	else
	  Serial.println("Could not set indoor mode");
      }
      if (bufPos <= bufLen && buffer[0] == 'O') {
	if (as3935.setIndoor(false))
	  Serial.println("Set to outdoor mode");
	else
	  Serial.println("Could not set outdoor mode");
      }
      if (bufPos <= bufLen && buffer[0] == 'D') {
	if (as3935.setMaskDisturber(false))
	  Serial.println("Allow disturbers");
	else
	  Serial.println("Could not set to allow disturbers");
      }
      if (bufPos <= bufLen && buffer[0] == 'Q') {
	if (as3935.setMaskDisturber(true))
	  Serial.println("Prevent disturbers");
	else
	  Serial.println("Could not set to prevent disturbers");
      }
      if (bufPos <= bufLen && buffer[0] == 'R') {
	Serial.println("Reading registers:");
	readRegs(0, 0x09);
      }
      if (bufPos <= bufLen && buffer[0] == 'S') {
	Serial.println("Fake strike!");
	as3935.interruptHandler();
      }
	  
      bufPos = 0;
    }
    else if (bufPos < bufLen)
      // Store character
      buffer[bufPos++] = c; 
  }
    
  if (as3935.process()) {
    uint8_t flags = as3935.getInterruptFlags();
    uint8_t dist = as3935.getDistance();

    Serial.println("-------------------");
    Serial.print("Timestamp: ");
    Serial.println(timestamp);
    Serial.print("Interrupt flags: ");
    Serial.println(flags, HEX);
    Serial.print("Distance: ");
    Serial.println(dist, DEC);
    
  }
  if (as3935.getBusError()) 
    Serial.println("Bus error!");

  if (as3935.getTriggered()) 
    Serial.println("Triggered!");


  if (d.isExpired()) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    
    if (++count > 5) {
      count = 0;
      rtc.readClock(tm);
      
      Serial.print("millis = ");
      Serial.print(millis());
      RTCx::time_t t = RTCx::mktime(&tm);
      Serial.print(", unixtime = ");
      Serial.print(t);
      Serial.print(", ");
      printTm(Serial, &tm);
      //readRegs(0, 0x09);
    }
    d.start(1000, AsyncDelay::MILLIS);
  }
  
}
