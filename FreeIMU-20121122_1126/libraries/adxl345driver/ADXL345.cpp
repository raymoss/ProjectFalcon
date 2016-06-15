/**************************************************************************
 *                                                                         *
 * ADXL345 Driver for Arduino                                              *
 *                                                                         *
 ***************************************************************************
 *                                                                         * 
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/
#include <cstdint>
#include "ADXL345.h"
#include "my_i2c.h"
#include "stm32f4xx_i2c.h"
#include "usart_print.h"
#define TO_READ (6)      // num of bytes we are going to read each time (two bytes for each axis)

ADXL345::ADXL345() {
  status = ADXL345_OK;
  error_code = ADXL345_NO_ERROR;

  gains[0] = 0.00376390;
  gains[1] = 0.00376009;
  gains[2] = 0.00349265;
}

void ADXL345::init(uint8_t address) {
  _dev_address = address;
  int i=0;
  usart_printfm(USARTx,(const int *)"Device address :%2x\n\r",address);
  powerOn();
  //usart_printfm(USARTx,(const int *)"Device address 2:%2x\n\r",address);
  //powerOn();
  //getRegisterBit(ADXL345_POWER_CTL,1);
}

void ADXL345::powerOn() {
  //Turning on the ADXL345
  //writeTo(ADXL345_POWER_CTL, 0);      
  //writeTo(ADXL345_POWER_CTL, 16);
  writeTo(_dev_address,ADXL345_POWER_CTL, 8);
  writeTo(_dev_address,ADXL345_DATA_FORMAT,0xB);
}

// Reads the acceleration into an array of three places
int ADXL345::readAccel(int *xyz){
  int a=0;
  a=readAccel(xyz, xyz + 1, xyz + 2);
  if(a<0) return -1;
}

// Reads the acceleration into three variable x, y and z
int ADXL345::readAccel(int *x, int *y, int *z) {
 int a=0;
  a=readFrom(_dev_address,ADXL345_DATAX0, 2, &_buff[0]); //read the acceleration data from the ADXL345
  if(a<0) return -1;
  //  ms_delay(1000);
//  readFrom(_dev_address,ADXL345_DATAX1, 1, &_buff[1]);
//  ms_delay(1000);
  a=readFrom(_dev_address,ADXL345_DATAY0, 2, &_buff[2]);
  if(a<0) return -1;
//  usart_printfm(USARTx,(const int *)"Value y0= %d\n\r",_buff[2]);
//  ms_delay(1000);
  //readFrom(_dev_address,ADXL345_DATAY1, 1, &_buff[3]);
//  usart_printfm(USARTx,(const int *)"Value y1= %d\n\r",_buff[3]);
//  ms_delay(1000);
  a=readFrom(_dev_address,ADXL345_DATAZ0, 2, &_buff[4]);
  if(a<0) return -1;
  //  usart_printfm(USARTx,(const int *)"Value z0= %d\n\r",_buff[4]);
//  ms_delay(1000);
//  readFrom(_dev_address,ADXL345_DATAZ1, 1, &_buff[5]);
//  usart_printfm(USARTx,(const int *)"Value z1= %d\n\r",_buff[5]);
//  ms_delay(1000);
 /*TODO:Multibyte read is not working for now .Need to debug from logic analyser*/ 
// readFrom(_dev_address,ADXL345_DATAX0, 6, _buff);
  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  *x = (((int)_buff[1]) << 8) | _buff[0];  
  *y = (((int)_buff[3]) << 8) | _buff[2];
 // usart_printfm(USARTx,(const int *)"Value y0= %d\n\r",*y);
  *z = (((int)_buff[5]) << 8) | _buff[4];
return 0;
}

int ADXL345::get_Gxyz(int *xyz_int){
  int i,a;
  //int xyz_int[3];
  a=readAccel(xyz_int);
  if(a<0) return -1;
  for(i=0; i<3; i++){
    //xyz[i] = xyz_int[i] * gains[i];
  }
return 0;
}

// Writes val to address register on device
//void ADXL345::writeTo(byte address, byte val) {
//  int a=0;
//  a=i2c_beginTransmission(I2C2,_dev_address,i2c_write); // start transmission to device
//  //usart_printfm(USARTx,(const int *)"Writing to this device address: %2x\n\r",_dev_address);
//  //assert(a);
//  a=i2c_sendData(I2C2,address);             // send register address
//  //assert(a);   
//  a=i2c_sendData(I2C2,val);  // send value to write
//  //assert(a);
//  a=i2c_stopTransmission(I2C2);         // end transmission
//}

// Reads num bytes starting from address register on device in to _buff array
//void ADXL345::readFrom(_dev_address,byte address, int num, byte _buff[]) {
//  int a=0,i=0;
//  a=i2c_beginTransmission(I2C2,_dev_address,i2c_write); // start transmission to device
//  //assert(a);
// // usart_printfm(USARTx,(const int *)"Reading from this device address: %2x\n\r",_dev_address);
//  a=i2c_sendData(I2C2,address);             // sends address to read from
//  //usart_printfm(USARTx,(const int *)"sent this data which is the address to read from: %2x\n\r",address);
//  //  if(a<0)
////     return (a-1);
//  //i2c_stopTransmission(I2C2);         // end transmission
////  if(a<0)
////     return (a-2);
//  i2c_beginTransmission(I2C2,_dev_address,i2c_read); // start transmission to device
//  //usart_printfm(USARTx,(const int *)"Initiated the register read cycle: %2x\n\r",_dev_address);
//  //  if(a<0)
////     return a;
//  // request 6 bytes from device
//  if(num==1){
//    I2C_AcknowledgeConfig(I2C2, DISABLE);
//	int timeout;
//        uint8_t flag1=0,flag2=0;
//        /* Test on I2C1 EV8 and clear it */
//        timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
//         
//        
//	// wait until one byte has been received
//      
//	while( !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) ){
//          ms_delay(100);
//          if((timeout--)==0){
//            flag1 = I2C2->SR1;
//            flag2 = I2C2->SR2;
//            usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
//            usart_printf(USARTx,"Failing at read nnack stage\n\r");
//            return ;
//          }
//        }
//	// wait until one byte has been received
//	 I2C_GenerateSTOP(I2C2, ENABLE);
//	// read data from I2C data register and return data byte
//	_buff[i] = I2C_ReceiveData(I2C2);
//        return;
//  }
//  if(num==2)
//  {
//    while(I2C_GetFlagStatus(I2C2,I2C_FLAG_ADDR)!=SET);
//    I2C_AcknowledgeConfig(I2C2, DISABLE);
//    I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Next);
//    while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BTF)!=SET);
//    i2c_stopTransmission(I2C2);
//    _buff[i] = I2C_ReceiveData(I2C2);
//    i++;
//    _buff[i] = I2C_ReceiveData(I2C2);
//    num-=2;
//    // receive a byte
//    //usart_printfm(USARTx,(const int *)"this byte read from accel: %2x\n\r",i);
//   return ; 
//  }
//  while(num>0)         // device may send less than requested (abnormal)
//  if(num==3){
//  //_buff[i]=I2C_read_ack(I2C2);
//  while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BTF)!=SET);
//  I2C_AcknowledgeConfig(I2C2, DISABLE);
//  _buff[i]=I2C_ReceiveData(I2C2);
//  i++;
//  num--;
//  while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BTF)!=SET);
//  i2c_stopTransmission(I2C2);
//  _buff[i]=I2C_ReceiveData(I2C2);
//  i++;
//  _buff[i]=I2C_ReceiveData(I2C2);
//  i++;
//  num-=2;
//  usart_printfm(USARTx,(const int *)"This byte read from nack: %2x\n\r",i);
//  return; 
//  }
//  else{
//    _buff[i]=I2C_read_ack(I2C2);
//    i++;
//    num--;
//  }
////  if(i != num){
////    status = ADXL345_ERROR;
////    error_code = ADXL345_READ_ERROR;
////  }
//  //ms_delay(10);
//  //i2c_stopTransmission(I2C2);         // end transmission
//}

// Gets the range setting and return it into rangeSetting
// it can be 2, 4, 8 or 16
void ADXL345::getRangeSetting(byte* rangeSetting) {
  byte _b;
  readFrom(_dev_address,ADXL345_DATA_FORMAT, 1, &_b);
  *rangeSetting = _b & 0x3;
}

// Sets the range setting, possible values are: 2, 4, 8, 16
void ADXL345::setRangeSetting(int val) {
  byte _s;
  byte _b;

  switch (val) {
  case 2:  
    _s = 0x0;
    break;
  case 4:  
    _s = 0x1;
    break;
  case 8:  
    _s = 0x2;
    break;
  case 16:
    _s = 0x3;
    break;
  default:
    _s = 0x0;
  }
  readFrom(_dev_address,ADXL345_DATA_FORMAT, 1, &_b);
  _s |= (_b & 0xEC);
  writeTo(_dev_address,ADXL345_DATA_FORMAT, _s);
}
// gets the state of the SELF_TEST bit
bool ADXL345::getSelfTestBit() {
  return getRegisterBit(ADXL345_DATA_FORMAT, 7);
}

// Sets the SELF-TEST bit
// if set to 1 it applies a self-test force to the sensor causing a shift in the output data
// if set to 0 it disables the self-test force
void ADXL345::setSelfTestBit(bool selfTestBit) {
  setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
}

// Gets the state of the SPI bit
bool ADXL345::getSpiBit() {
  return getRegisterBit(ADXL345_DATA_FORMAT, 6);
}

// Sets the SPI bit
// if set to 1 it sets the device to 3-wire mode
// if set to 0 it sets the device to 4-wire SPI mode
void ADXL345::setSpiBit(bool spiBit) {
  setRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
}

// Gets the state of the INT_INVERT bit
bool ADXL345::getInterruptLevelBit() {
  return getRegisterBit(ADXL345_DATA_FORMAT, 5);
}

// Sets the INT_INVERT bit
// if set to 0 sets the interrupts to active high
// if set to 1 sets the interrupts to active low
void ADXL345::setInterruptLevelBit(bool interruptLevelBit) {
  setRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
}

// Gets the state of the FULL_RES bit
bool ADXL345::getFullResBit() {
  return getRegisterBit(ADXL345_DATA_FORMAT, 3);
}

// Sets the FULL_RES bit
// if set to 1, the device is in full resolution mode, where the output resolution increases with the
//   g range set by the range bits to maintain a 4mg/LSB scal factor
// if set to 0, the device is in 10-bit mode, and the range buts determine the maximum g range
//   and scale factor
void ADXL345::setFullResBit(bool fullResBit) {
  setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}

// Gets the state of the justify bit
bool ADXL345::getJustifyBit() {
  return getRegisterBit(ADXL345_DATA_FORMAT, 2);
}

// Sets the JUSTIFY bit
// if sets to 1 selects the left justified mode
// if sets to 0 selects right justified mode with sign extension
void ADXL345::setJustifyBit(bool justifyBit) {
  setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
}

// Sets the THRESH_TAP byte value
// it should be between 0 and 255
// the scale factor is 62.5 mg/LSB
// A value of 0 may result in undesirable behavior
void ADXL345::setTapThreshold(int tapThreshold) {
  tapThreshold = min(max(tapThreshold,0),255);
  byte _b = byte (tapThreshold);
  writeTo(_dev_address,ADXL345_THRESH_TAP, _b);  
}

// Gets the THRESH_TAP byte value
// return value is comprised between 0 and 255
// the scale factor is 62.5 mg/LSB
int ADXL345::getTapThreshold() {
  byte _b;
  readFrom(_dev_address,ADXL345_THRESH_TAP, 1, &_b);  
  return int (_b);
}

// set/get the gain for each axis in Gs / count
void ADXL345::setAxisGains(float *_gains){
  int i;
  for(i = 0; i < 3; i++){
    gains[i] = _gains[i];
  }
}
void ADXL345::getAxisGains(float *_gains){
  int i;
  for(i = 0; i < 3; i++){
    _gains[i] = gains[i];
  }
}
 

// Sets the OFSX, OFSY and OFSZ bytes
// OFSX, OFSY and OFSZ are user offset adjustments in twos complement format with
// a scale factor of 15,6mg/LSB
// OFSX, OFSY and OFSZ should be comprised between
void ADXL345::setAxisOffset(int x, int y, int z) {
  writeTo(_dev_address,ADXL345_OFSX, byte (x));  
  writeTo(_dev_address,ADXL345_OFSY, byte (y));  
  writeTo(_dev_address,ADXL345_OFSZ, byte (z));  
}

// Gets the OFSX, OFSY and OFSZ bytes
void ADXL345::getAxisOffset(int* x, int* y, int*z) {
  byte _b;
  readFrom(_dev_address,ADXL345_OFSX, 1, &_b);  
  *x = int (_b);
  readFrom(_dev_address,ADXL345_OFSY, 1, &_b);  
  *y = int (_b);
  readFrom(_dev_address,ADXL345_OFSZ, 1, &_b);  
  *z = int (_b);
}

// Sets the DUR byte
// The DUR byte contains an unsigned time value representing the maximum time
// that an event must be above THRESH_TAP threshold to qualify as a tap event
// The scale factor is 625µs/LSB
// A value of 0 disables the tap/float tap funcitons. Max value is 255.
void ADXL345::setTapDuration(int tapDuration) {
  tapDuration = min(max(tapDuration,0),255);
  byte _b = byte (tapDuration);
  writeTo(_dev_address,ADXL345_DUR, _b);  
}

// Gets the DUR byte
int ADXL345::getTapDuration() {
  byte _b;
  readFrom(_dev_address,ADXL345_DUR, 1, &_b);  
  return int (_b);
}

// Sets the latency (latent register) which contains an unsigned time value
// representing the wait time from the detection of a tap event to the start
// of the time window, during which a possible second tap can be detected.
// The scale factor is 1.25ms/LSB. A value of 0 disables the float tap function.
// It accepts a maximum value of 255.
void ADXL345::setDoubleTapLatency(int floatTapLatency) {
  byte _b = byte (floatTapLatency);
  writeTo(_dev_address,ADXL345_LATENT, _b);  
}

// Gets the Latent value
int ADXL345::getDoubleTapLatency() {
  byte _b;
  readFrom(_dev_address,ADXL345_LATENT, 1, &_b);  
  return int (_b);
}

// Sets the Window register, which contains an unsigned time value representing
// the amount of time after the expiration of the latency time (Latent register)
// during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
// value of 0 disables the float tap function. The maximum value is 255.
void ADXL345::setDoubleTapWindow(int floatTapWindow) {
  floatTapWindow = min(max(floatTapWindow,0),255);
  byte _b = byte (floatTapWindow);
  writeTo(_dev_address,ADXL345_WINDOW, _b);  
}

// Gets the Window register
int ADXL345::getDoubleTapWindow() {
  byte _b;
  readFrom(_dev_address,ADXL345_WINDOW, 1, &_b);  
  return int (_b);
}

// Sets the THRESH_ACT byte which holds the threshold value for detecting activity.
// The data format is unsigned, so the magnitude of the activity event is compared
// with the value is compared with the value in the THRESH_ACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// activity interrupt is enabled. The maximum value is 255.
void ADXL345::setActivityThreshold(int activityThreshold) {
  activityThreshold = min(max(activityThreshold,0),255);
  byte _b = byte (activityThreshold);
  writeTo(_dev_address,ADXL345_THRESH_ACT, _b);  
}

// Gets the THRESH_ACT byte
int ADXL345::getActivityThreshold() {
  byte _b;
  readFrom(_dev_address,ADXL345_THRESH_ACT, 1, &_b);  
  return int (_b);
}

// Sets the THRESH_INACT byte which holds the threshold value for detecting inactivity.
// The data format is unsigned, so the magnitude of the inactivity event is compared
// with the value is compared with the value in the THRESH_INACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// inactivity interrupt is enabled. The maximum value is 255.
void ADXL345::setInactivityThreshold(int inactivityThreshold) {
  inactivityThreshold = min(max(inactivityThreshold,0),255);
  byte _b = byte (inactivityThreshold);
  writeTo(_dev_address,ADXL345_THRESH_INACT, _b);  
}

// Gets the THRESH_INACT byte
int ADXL345::getInactivityThreshold() {
  byte _b;
  readFrom(_dev_address,ADXL345_THRESH_INACT, 1, &_b);  
  return int (_b);
}

// Sets the TIME_INACT register, which contains an unsigned time value representing the
// amount of time that acceleration must be less thant the value in the THRESH_INACT
// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
// be between 0 and 255.
void ADXL345::setTimeInactivity(int timeInactivity) {
  timeInactivity = min(max(timeInactivity,0),255);
  byte _b = byte (timeInactivity);
  writeTo(_dev_address,ADXL345_TIME_INACT, _b);  
}

// Gets the TIME_INACT register
int ADXL345::getTimeInactivity() {
  byte _b;
  readFrom(_dev_address,ADXL345_TIME_INACT, 1, &_b);  
  return int (_b);
}

// Sets the THRESH_FF register which holds the threshold value, in an unsigned format, for
// free-fall detection. The root-sum-square (RSS) value of all axes is calculated and
// compared whith the value in THRESH_FF to determine if a free-fall event occured. The
// scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the free-fall
// interrupt is enabled. The maximum value is 255.
void ADXL345::setFreeFallThreshold(int freeFallThreshold) {
  freeFallThreshold = min(max(freeFallThreshold,0),255);
  byte _b = byte (freeFallThreshold);
  writeTo(_dev_address,ADXL345_THRESH_FF, _b);  
}

// Gets the THRESH_FF register.
int ADXL345::getFreeFallThreshold() {
  byte _b;
  readFrom(_dev_address,ADXL345_THRESH_FF, 1, &_b);  
  return int (_b);
}

// Sets the TIME_FF register, which holds an unsigned time value representing the minimum
// time that the RSS value of all axes must be less than THRESH_FF to generate a free-fall
// interrupt. The scale factor is 5ms/LSB. A value of 0 may result in undesirable behavior if
// the free-fall interrupt is enabled. The maximum value is 255.
void ADXL345::setFreeFallDuration(int freeFallDuration) {
  freeFallDuration = min(max(freeFallDuration,0),255);  
  byte _b = byte (freeFallDuration);
  writeTo(_dev_address,ADXL345_TIME_FF, _b);  
}

// Gets the TIME_FF register.
int ADXL345::getFreeFallDuration() {
  byte _b;
  readFrom(_dev_address,ADXL345_TIME_FF, 1, &_b);  
  return int (_b);
}

bool ADXL345::isActivityXEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 6);
}
bool ADXL345::isActivityYEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 5);
}
bool ADXL345::isActivityZEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 4);
}
bool ADXL345::isInactivityXEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 2);
}
bool ADXL345::isInactivityYEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 1);
}
bool ADXL345::isInactivityZEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 0);
}

void ADXL345::setActivityX(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state);
}
void ADXL345::setActivityY(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state);
}
void ADXL345::setActivityZ(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state);
}
void ADXL345::setInactivityX(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 2, state);
}
void ADXL345::setInactivityY(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 1, state);
}
void ADXL345::setInactivityZ(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 0, state);
}

bool ADXL345::isActivityAc() {
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 7);
}
bool ADXL345::isInactivityAc(){
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 3);
}

void ADXL345::setActivityAc(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 7, state);
}
void ADXL345::setInactivityAc(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 3, state);
}

bool ADXL345::getSuppressBit(){
  return getRegisterBit(ADXL345_TAP_AXES, 3);
}
void ADXL345::setSuppressBit(bool state) {  
  setRegisterBit(ADXL345_TAP_AXES, 3, state);
}

bool ADXL345::isTapDetectionOnX(){
  return getRegisterBit(ADXL345_TAP_AXES, 2);
}
void ADXL345::setTapDetectionOnX(bool state) {  
  setRegisterBit(ADXL345_TAP_AXES, 2, state);
}
bool ADXL345::isTapDetectionOnY(){
  return getRegisterBit(ADXL345_TAP_AXES, 1);
}
void ADXL345::setTapDetectionOnY(bool state) {  
  setRegisterBit(ADXL345_TAP_AXES, 1, state);
}
bool ADXL345::isTapDetectionOnZ(){
  return getRegisterBit(ADXL345_TAP_AXES, 0);
}
void ADXL345::setTapDetectionOnZ(bool state) {  
  setRegisterBit(ADXL345_TAP_AXES, 0, state);
}

bool ADXL345::isActivitySourceOnX(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 6);
}
bool ADXL345::isActivitySourceOnY(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 5);
}
bool ADXL345::isActivitySourceOnZ(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 4);
}

bool ADXL345::isTapSourceOnX(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 2);
}
bool ADXL345::isTapSourceOnY(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 1);
}
bool ADXL345::isTapSourceOnZ(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 0);
}

bool ADXL345::isAsleep(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 3);
}

bool ADXL345::isLowPower(){
  return getRegisterBit(ADXL345_BW_RATE, 4);
}
void ADXL345::setLowPower(bool state) {  
  setRegisterBit(ADXL345_BW_RATE, 4, state);
}

float ADXL345::getRate(){
  byte _b;
  readFrom(_dev_address,ADXL345_BW_RATE, 1, &_b);
  _b &= 0x0F;
  return (pow(2,((int) _b)-6)) * 6.25;
}

void ADXL345::setRate(float rate){
  byte _b,_s;
  int v = (int) (rate / 6.25);
  int r = 0;
  while (v >>= 1)
  {
    r++;
  }
  if (r <= 9) {
    readFrom(_dev_address,ADXL345_BW_RATE, 1, &_b);
    _s = (byte) (r + 6) | (_b & 0xF0);
    writeTo(_dev_address,ADXL345_BW_RATE, _s);
  }
}

void ADXL345::set_bw(byte bw_code){
  if((bw_code < ADXL345_BW_3) || (bw_code > ADXL345_BW_1600)){
    status = false;
    error_code = ADXL345_BAD_ARG;
  }
  else{
    writeTo(_dev_address,ADXL345_BW_RATE, bw_code);
  }
}

byte ADXL345::get_bw_code(){
  byte bw_code;
  readFrom(_dev_address,ADXL345_BW_RATE, 1, &bw_code);
  return bw_code;
}

byte ADXL345::getInterruptSource() {
  byte _b;
  readFrom(_dev_address,ADXL345_INT_SOURCE, 1, &_b);
  return _b;
}

bool ADXL345::getInterruptSource(byte interruptBit) {
  return getRegisterBit(ADXL345_INT_SOURCE,interruptBit);
}

bool ADXL345::getInterruptMapping(byte interruptBit) {
  return getRegisterBit(ADXL345_INT_MAP,interruptBit);
}

// Set the mapping of an interrupt to pin1 or pin2
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
void ADXL345::setInterruptMapping(byte interruptBit, bool interruptPin) {
  setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
}

bool ADXL345::isInterruptEnabled(byte interruptBit) {
  return getRegisterBit(ADXL345_INT_ENABLE,interruptBit);
}

void ADXL345::setInterrupt(byte interruptBit, bool state) {
  setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}

void ADXL345::setRegisterBit(byte regAdress, int bitPos, bool state) {
  byte _b;
  readFrom(_dev_address,regAdress, 1, &_b);
  if (state) {
    _b |= (1 << bitPos);  // forces nth bit of _b to be 1.  all other bits left alone.
  }
  else {
    _b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
  }
  writeTo(_dev_address,regAdress, _b);  
}

bool ADXL345::getRegisterBit(byte regAdress, int position) {
  byte _b;
  readFrom(_dev_address,regAdress, 1, &_b);
   usart_printfm(USARTx,(const int*)"Device id : %2x",_b);
  return ((_b)&(1<<position));
}

// print all register value to the serial ouptut, which requires it to be setup
// this can be used to manually to check the current configuration of the device
void ADXL345::printAllRegister() {
  byte _b;
  //usart_printf(USARTx,"0x00: ");
  readFrom(_dev_address,0x00, 1, &_b);
  print_byte(_b);
  //usart_printf(USARTx,"\n");
  int i;
  for (i=29;i<=57;i++){
    //usart_printf(USARTx,"0x:%d",i);
    //usart_printf(USARTx,": ");
    readFrom(_dev_address,i, 1, &_b);
    print_byte(_b);
    //usart_printf(USARTx,"\n");    
  }
}

void print_byte(byte val){
  int i;
  //usart_printf(USARTx,"B");
  for(i=7; i>=0; i--){
 
    //usart_printf(USARTx,"%d",val >> i &1);
  }
  //readFrom(_dev_address,0x00,1,&i);
 
}

void* accelerometer_initialization(byte device_address){
  ADXL345* accel = new ADXL345();
  accel->init(ADXL345_ADDR_ALT_LOW);
  accel->set_bw(ADXL345_BW_12);
  return (void*)(accel);
}

int accel_xyz(void *accel,int accel_data[]){
  int a=0;
  a=((ADXL345*)accel)->get_Gxyz(accel_data);
  if(a<0) return -1;
  return 0;
}

int pow(int x, int y){
    int result=1;
    for(;y>0;y--){
    result*=x;
    }
    return x;
  }