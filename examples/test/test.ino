#include <AsyncDelay.h>
#include <SoftWire.h>
#include <AS3935.h>

#ifdef JTD
#include <DisableJTAG.h>
#endif

SoftWire i2c(14, 17);
uint8_t addr = 0x03;

void setup(void)
{
#ifdef JTD
  disableJTAG();
#endif
  
  Serial.begin(9600);
  i2c.setTimeout_ms(40);
  i2c.setDelay_us(2);
  i2c.begin();
  i2c.stop();
  
  for (uint8_t reg = 0; reg < 0x10; ++reg) {
    delay(50);
    i2c.start(addr, SoftWire::writeMode);
    i2c.write(reg);
    i2c.repeatedStart(addr, SoftWire::readMode);
    //i2c.stop();
    //i2c.start(addr, SoftWire::readMode);
    uint8_t val;
    i2c.readThenNack(val);
    i2c.stop();
    
    Serial.print("Reg: 0x");
    Serial.print(reg, HEX);
    Serial.print(": 0x");
    Serial.println(val, HEX);
    Serial.flush();
  }
  
}

void loop(void)
{
  ;
}
