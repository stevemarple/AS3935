#include <AsyncDelay.h>
#include <SoftWire.h>
#include <AS3935.h>

#ifdef JTD
#include <DisableJTAG.h>
#endif

AS3935 as3935;
bool ledState = true;
AsyncDelay d;

void int2Handler(void)
{
	as3935.interruptHandler();
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

	Serial.println("-------------");
}


void printInterruptReason(Stream &s, uint8_t value, const char *prefix = nullptr)
{
	if (value & AS3935::intNoiseLevelTooHigh) {
		if (prefix)
			s.print(prefix);
		s.println(F("Noise level too high"));
	}
	if (value & AS3935::intDisturberDetected) {
		if (prefix)
			s.print(prefix);
		s.println(F("Disturber detected"));
	}
	if (value & AS3935::intLightningDetected) {
		if (prefix)
			s.print(prefix);
		s.println(F("Lightning detected"));
	}
}


void setup(void)
{
#ifdef JTD
	disableJTAG();
#endif

	Serial.begin(9600);
	as3935.initialise(14, 17, 0x03, 3, true, NULL);

	// as3935.calibrate();
	as3935.start();
	d.start(1000, AsyncDelay::MILLIS);

	while (!d.isExpired())
		as3935.process();

	readRegs(0, 0x09);
	as3935.setNoiseFloor(0);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, ledState);

	attachInterrupt(2, int2Handler, RISING);
	d.start(1000, AsyncDelay::MILLIS);
	Serial.println("setup() done");
}

uint8_t count = 0;
void loop(void)
{
	if (as3935.process()) {
		uint8_t flags = as3935.getInterruptFlags();
		uint8_t dist = as3935.getDistance();

		Serial.println("-------------------");
		Serial.println("Interrupt!");
		Serial.println("Reason(s):");
		printInterruptReason(Serial, flags, "    ");
		if (AS3935::intLightningDetected & flags) {
			Serial.print("Distance: ");
			Serial.println(dist, DEC);
		}
	}

	if (as3935.getBusError()) {
		Serial.println("Bus error!");
		as3935.clearBusError();
	}

	// Flash the LED to show activity
	if (d.isExpired()) {
		ledState = !ledState;
		digitalWrite(LED_BUILTIN, ledState);

		// // Periodically output the AS3935 registers
		// if (++count > 5) {
		// 	count = 0;
		// 	readRegs(0, 0x09);
		// }
		d.start(1000, AsyncDelay::MILLIS);
	}

}
