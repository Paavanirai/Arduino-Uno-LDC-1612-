#include "C:\\Users\\Paavani Rai\\Documents\\Arduino\\libraries\\itiadasm-ldc16xx_lib-arduino-f091e572f6c7\\LDC16xx_lib.h"
#include <Wire.h>



void setup(){
LDC_regNameAddr reg_array[] = {
  
{LDC16xx_DATA_MSB_CH0,       "DATA_MSB_CH0      " },
{LDC16xx_DATA_LSB_CH0,       "DATA_LSB_CH0      " },
{LDC16xx_DATA_MSB_CH1,       "DATA_MSB_CH1      " },   
{LDC16xx_DATA_LSB_CH1,       "DATA_LSB_CH1      " },   
{LDC16xx_DATA_MSB_CH2,       "DATA_MSB_CH2      " },   
{LDC16xx_DATA_LSB_CH2,       "DATA_LSB_CH2      " },   
{LDC16xx_DATA_MSB_CH3,       "DATA_MSB_CH3      " },   
{LDC16xx_DATA_LSB_CH3,       "DATA_LSB_CH3      " },   
{LDC16xx_RCOUNT_CH0,         "RCOUNT_CH0        " },     
{LDC16xx_RCOUNT_CH1,         "RCOUNT_CH1        " },     
{LDC16xx_RCOUNT_CH2,         "RCOUNT_CH2        " },     
{LDC16xx_RCOUNT_CH3,         "RCOUNT_CH3        " },     
{LDC16xx_OFFSET_CH0,         "OFFSET_CH0        " },     
{LDC16xx_OFFSET_CH1,         "OFFSET_CH1        " },     
{LDC16xx_OFFSET_CH2,         "OFFSET_CH2        " },     
{LDC16xx_OFFSET_CH3,         "OFFSET_CH3        " },     
{LDC16xx_SETTLECOUNT_CH0,    "SETTLECOUNT_CH0   " }, 
{LDC16xx_SETTLECOUNT_CH1,    "SETTLECOUNT_CH1   " },  
{LDC16xx_SETTLECOUNT_CH2,    "SETTLECOUNT_CH2   " }, 
{LDC16xx_SETTLECOUNT_CH3,    "SETTLECOUNT_CH3   " }, 
{LDC16xx_CLOCK_DIVIDERS_CH0, "CLOCK_DIVIDERS_CH0" }, 
{LDC16xx_CLOCK_DIVIDERS_CH1, "CLOCK_DIVIDERS_CH1" }, 
{LDC16xx_CLOCK_DIVIDERS_CH2, "CLOCK_DIVIDERS_CH2" }, 
{LDC16xx_CLOCK_DIVIDERS_CH3, "CLOCK_DIVIDERS_CH3" }, 
{LDC16xx_STATUS,             "STATUS            " }, 
{LDC16xx_ERROR_CONFIG,       "ERROR_CONFIG      " }, 
{LDC16xx_CONFIG,             "CONFIG            " }, 
{LDC16xx_MUX_CONFIG,         "MUX_CONFIG        " }, 
{LDC16xx_RESET_DEV,          "RESET_DEV         " },  
{LDC16xx_DRIVE_CURRENT_CH0,  "DRIVE_CURRENT_CH0 " }, 
{LDC16xx_DRIVE_CURRENT_CH1,  "DRIVE_CURRENT_CH1 " }, 
{LDC16xx_DRIVE_CURRENT_CH2,  "DRIVE_CURRENT_CH2 " }, 
{LDC16xx_DRIVE_CURRENT_CH3,  "DRIVE_CURRENT_CH3 " }, 
{LDC16xx_MANUFACTURER_ID,    "MANUFACTURER_ID   " },   
{LDC16xx_DEVICE_ID,          "DEVICE_ID         " }    
};

#define REG_ARR_SIZE sizeof(reg_array)/sizeof(LDC_regNameAddr)

// initialize I2C interface
LDC16xx::LDC16xx(TwoWire &iface, uint32_t clock){
i2c = &iface; 
  i2c->begin(); // start I2C peripheral as a masterr
  i2c->setClock(clock); // set clock to fast I2C 400kHz
}
}

void loop(){
// read data from given LDC register
int LDC16xx::readRegister(uint8_t reg, uint16_t * data){

beginTransmission(0x2A);
write(reg);
endTransmission();

  delayMicroseconds(2);

  int bytes = requestFrom(0x2A, 2);
  
  if(bytes != 2)
    return -1; // Error

  uint8_t MSB = read();
  uint8_t LSB = read();

  *data =  (MSB << 8) | LSB;

  return 0;
}

// write data to given LDC register
void LDC16xx::writeRegister(uint8_t reg, uint16_t data){

beginTransmission(0x2A);
write(reg);
write(data >> 8);
write(data);
endTransmission();
  
}

// read data for a defined channel (0-3)
int8_t LDC16xx::readChannel(uint8_t channel, uint32_t *data){

  uint8_t error = 0;

  if(channel > 3)
    return -1;  

  uint16_t MSB, LSB;
  readRegister(LDC16xx_DATA_MSB_CH0 + 2*channel, &MSB);
  readRegister(LDC16xx_DATA_LSB_CH0 + 2*channel, &LSB);

  error = MSB >> 12;
  *data = (((uint32_t)(MSB & 0x0FFF)) << 16) | LSB;
  
  return error;
  
}

// load full configuration array into LDC
void LDC16xx::loadConfig(LDC_configReg cfg[], uint8_t size){
  // upload default config to LDC
  for(uint8_t i=0; i<size; i++)
    writeRegister(cfg[i].reg, cfg[i].value);
}

// display LDC register values using user provided serial stream
void LDC16xx::registerDump(HardwareSerial &s){
  Serial.println("LDC16xx registers dump:");
  for(int i=0; i<REG_ARR_SIZE; i++){
    uint16_t data;
    readRegister(reg_array[i].addr, &data);
    Serial.println(reg_array[i].name);
    Serial.print(" (");
    Serial.print(i,2,s, HEX);
    Serial.println(") : ");
    Serial.println(data,4,s, HEX);
    Serial.println();
  }
}

// clear and set bits in LDC registers, provided user masks
void LDC16xx::clearAndSetRegisterBits(uint8_t reg, uint16_t clear_mask, uint16_t set_mask){
   // obtain current value of error config register
  uint16_t val;
  readRegister(reg, &val);
  // clear desired bits
  val &= ~clear_mask;
  // set desired bits
  val |= set_mask;

  // write new value to the device
  writeRegister(reg, val);
}

// reset the device using I2C command
void LDC16xx::resetDevice(){
  writeRegister(LDC16xx_RESET_DEV, LDC16xx_BITS_RESET_DEV);
  // 10ms should be enough to start up device
  delay(10);
}

// attach interrupt to a user provided pin and bind user callback function
void LDC16xx::enableDataReadyInterrupt(uint8_t int_pin, void (*isr)()){
  // clear any INTB source and set only DRDY_2INT
  clearAndSetRegisterBits(LDC16xx_ERROR_CONFIG, 0x00FF, 0x0001);
  clearAndSetRegisterBits(LDC16xx_CONFIG, 1 << 7, 0);
  // attach interrupt to the provided pin
  pinMode(int_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(int_pin), isr, FALLING);  
}

// print hex number to given stream with a defined precision (including leading zeros)
void LDC16xx::printHex(int num, int precision, HardwareSerial &s) {
     char tmp[16];
     char format[16];

     Serial.println(format, "0x%%.%dX", precision);

     Serial.println(tmp, format, num);
     Serial.println(tmp);
}
}
