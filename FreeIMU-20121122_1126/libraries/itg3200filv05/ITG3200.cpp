/****************************************************************************
* ITG3200.h - ITG-3200/I2C library v0.5 for Arduino                         *
* Copyright 2010-2011 Filipe Vieira & various contributors                  *
* http://code.google.com/p/itg-3200driver                                   *
* This file is part of ITG-3200 Arduino library.                            *
*                                                                           *
* This library is free software: you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation, either version 3 of the License, or      *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
****************************************************************************/
/****************************************************************************
* Tested on Arduino Mega with ITG-3200 Breakout                             *
* SCL     -> pin 21     (no pull up resistors)                              *
* SDA     -> pin 20     (no pull up resistors)                              *
* CLK & GND -> pin GND                                                      *
* INT       -> not connected  (but can be used)                             *
* VIO & VDD -> pin 3.3V                                                     *
*****************************************************************************/
#include <cstdint>
#include "ITG3200.h"
#include "my_i2c.h"
#include "usart_print.h"

ITG3200::ITG3200() {
  setGains(1.0,1.0,1.0);
  setOffsets(0,0,0);
  setRevPolarity(0,0,0);
  //Wire.begin();       //Normally this code is called from setup() at user code
                        //but some people reported that joining I2C bus earlier
                        //apparently solved problems with master/slave conditions.
                        //Uncomment if needed.
}

int ITG3200::init(byte  address) {
  // Uncomment or change your default ITG3200 initialization
  int a=0;
  // fast sample rate - divisor = 0 filter = 0 clocksrc = 0, 1, 2, or 3  (raw values)
  _dev_address = address;
  a=init(address, NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_XGYRO_REF, true, true);
  MYASSERT(a,"Filaed to init")
  return 0;
  // slow sample rate - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 0, 1, 2, or 3  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW010_SR1, INTERNALOSC, true, true);
  
  // fast sample rate 32Khz external clock - divisor = 0  filter = 0  clocksrc = 4  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_EXTERNAL32, true, true);
  
  // slow sample rate 32Khz external clock - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 4  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW010_SR1, PLL_EXTERNAL32, true, true);
}

int ITG3200::init(byte address, byte _SRateDiv, byte _Range, byte _filterBW, byte _ClockSrc, bool _ITGReady, bool _INTRawDataReady) {
   int a=0;
  // _dev_address = address;
  // //this needs to be debugged
  // a=readFrom(_dev_address,WHO_AM_I,1,_buff);
  // MYASSERT(a,"Failed to write to register\n\r")
  // usart_printfm(USARTx,(const int *)"Succesfully read from register\n\r");
  // usart_printfm(USARTx,(const int *)"Value of the who am i register %d \n\r",_buff[0]);
  
// //  a=writeTo(_dev_address,WHO_AM_I,66);
// //  MYASSERT(a,"Failed to write 2 register2\n\r")
// //  usart_printfm(USARTx,(const int *)"Succesfully written to register2\n\r");
  
  // a=readFrom(_dev_address,WHO_AM_I,1,_buff);
  // usart_printfm(USARTx,(const int *)"Value of the who am i register %d \n\r",_buff[0]);
  // MYASSERT(a,"Failed to write to register\n\r")
  // //this is needed to be debugged
  
  // usart_printfm(USARTx,(const int *)"Value of address:%2x\n\r",_dev_address);
  a=setSampleRateDiv(_SRateDiv);
  MYASSERT(a,"Failed to set sample rate div\n\r")
  a=setFSRange(_Range);
  MYASSERT(a,"Failed to set range\n\r")
  a=setFilterBW(_filterBW);
  MYASSERT(a,"Failed to set filter BW\n\r")
  a=setClockSource(_ClockSrc);
  MYASSERT(a,"Failed to set clocksrc\n\r")
  a=setITGReady(_ITGReady);
  MYASSERT(a,"Failed to set ITG ready\n\r")
  a=setRawDataReady(_INTRawDataReady);  
  MYASSERT(a,"Failed to set data ready mode\n\r")
  ms_delay(GYROSTART_UP_DELAY);  // startup 
  return 0;
}

byte ITG3200::getDevAddr() {
  /*a=readFrom( _dev_address,WHO_AM_I, 1, &_buff[0]); 
  return _buff[0];  */
  return _dev_address;
}

void ITG3200::setDevAddr(unsigned int  _addr) {
  int a=0;
  a=writeTo( _dev_address,WHO_AM_I, _addr); 
  //MYASSERT(a,"Failed to set device address\n\r")
  _dev_address = _addr;
}

byte ITG3200::getSampleRateDiv() {
  int a=0;
  a=readFrom( _dev_address,SMPLRT_DIV, 1, &_buff[0]);
  MYASSERT(a,"Failed to get sample rate division\n\r")
  return _buff[0];
}

int ITG3200::setSampleRateDiv(byte _SampleRate) {
  int a=0;
  a=writeTo( _dev_address,SMPLRT_DIV, _SampleRate);
  MYASSERT(a,"Failed to set sample rate div\n\r")
  return 0;
  }

byte ITG3200::getFSRange() {
  int a=0;
  a=readFrom( _dev_address,DLPF_FS, 1, &_buff[0]);
  return ((_buff[0] & DLPFFS_FS_SEL) >> 3);
}

int ITG3200::setFSRange(byte _Range) {
  int a=0;
  a=readFrom( _dev_address,DLPF_FS, 1, &_buff[0]);
  MYASSERT(a,"Failed to get DLPF_FS\n\r")  
  a=writeTo( _dev_address,DLPF_FS, ((_buff[0] & ~DLPFFS_FS_SEL) | (_Range << 3)) ); 
  MYASSERT(a,"Failed to set FSrange\n\r")
    return 0;
  }

byte ITG3200::getFilterBW() {  
  int a=0;
  a=readFrom( _dev_address,DLPF_FS, 1, &_buff[0]);
  return (_buff[0] & DLPFFS_DLPF_CFG); 
}

int ITG3200::setFilterBW(byte _BW) {   
  int a=0;
  a=readFrom( _dev_address,DLPF_FS, 1, &_buff[0]);
  MYASSERT(a,"Failed to get BW\n\r")
  a=writeTo( _dev_address,DLPF_FS, ((_buff[0] & ~DLPFFS_DLPF_CFG) | _BW));
  MYASSERT(a,"Failed to set BW\n\r")
    return 0;
}

bool ITG3200::isINTActiveOnLow() {  
int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_ACTL) >> 7);
}

void ITG3200::setINTLogiclvl(bool _State) {
 int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  a=writeTo( _dev_address,INT_CFG, ((_buff[0] & ~INTCFG_ACTL) | (_State << 7))); 
}

bool ITG3200::isINTOpenDrain() {  
 int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_OPEN) >> 6);
}

void ITG3200::setINTDriveType(bool _State) {
 int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  a=writeTo( _dev_address,INT_CFG, ((_buff[0] & ~INTCFG_OPEN) | _State << 6)); 
}

bool ITG3200::isLatchUntilCleared() {    
 int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_LATCH_INT_EN) >> 5);
}

void ITG3200::setLatchMode(bool _State) {
 int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  a=writeTo( _dev_address,INT_CFG, ((_buff[0] & ~INTCFG_LATCH_INT_EN) | _State << 5)); 
}

bool ITG3200::isAnyRegClrMode() {    
 int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_INT_ANYRD_2CLEAR) >> 4);
}

void ITG3200::setLatchClearMode(bool _State) {
 int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  a=writeTo( _dev_address,INT_CFG, ((_buff[0] & ~INTCFG_INT_ANYRD_2CLEAR) | _State << 4)); 
}

bool ITG3200::isITGReadyOn() {   
  int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_ITG_RDY_EN) >> 2);
}

int ITG3200::setITGReady(bool _State) {
   int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  MYASSERT(a,"Failed to set interrupt mode\n\r")
  a=writeTo( _dev_address,INT_CFG, ((_buff[0] & ~INTCFG_ITG_RDY_EN) | _State << 2)); 
  MYASSERT(a,"Failed to set sample rate div\n\r")
  return 0;
  }

bool ITG3200::isRawDataReadyOn() {
 int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  return (_buff[0] & INTCFG_RAW_RDY_EN);
}

int ITG3200::setRawDataReady(bool _State) {
  int a=0;
  a=readFrom( _dev_address,INT_CFG, 1, &_buff[0]);
  MYASSERT(a,"Fialed to read raw data ready\n\r")
  a=writeTo( _dev_address,INT_CFG, ((_buff[0] & ~INTCFG_RAW_RDY_EN) | _State)); 
    MYASSERT(a,"Failed to set raw data ready\n\r")
  return 0;
  }

bool ITG3200::isITGReady() {
  int a=0;
  a=readFrom( _dev_address,INT_STATUS, 1, &_buff[0]);
  return ((_buff[0] & INTSTATUS_ITG_RDY) >> 2);
}

bool ITG3200::isRawDataReady() {
 int a=0;
  a=readFrom( _dev_address,INT_STATUS, 1, &_buff[0]);
  return (_buff[0] & INTSTATUS_RAW_DATA_RDY);
}

void ITG3200::readTemp(float *_Temp) {
  int a=0;
  a=readFrom( _dev_address,TEMP_OUT,2,_buff);
  *_Temp = 35 + (((_buff[0] << 8) | _buff[1]) + 13200) / 280.0;    // F=C*9/5+32  
}

int ITG3200::readGyroRaw(int *_GyroX, int *_GyroY, int *_GyroZ){
  int a=0;
  a=readFrom( _dev_address,GYRO_XOUT, 2, &_buff[0]);
  MYASSERT(a,"Failed to read gyro data from registers\n\r")
    a=readFrom( _dev_address,GYRO_YOUT, 2, &_buff[2]);
  MYASSERT(a,"Failed to read gyro data from registers\n\r")
    a=readFrom( _dev_address,GYRO_ZOUT, 2, &_buff[4]);
  MYASSERT(a,"Failed to read gyro data from registers\n\r")
  *_GyroX = (((int)(signed char)_buff[0] << 8) | _buff[1]);
  *_GyroY = (((int)(signed char)_buff[2] << 8) | _buff[3]); 
  *_GyroZ = (((int)(signed char)_buff[4] << 8) | _buff[5]);
   
  return 0;
  }

int ITG3200::readGyroRaw(int *_GyroXYZ){
  int a=0;
  a=readGyroRaw(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
  MYASSERT(a,"Failed to get from readGyroRaw3\n\r")
  return 0;
}

void ITG3200::setRevPolarity(bool _Xpol, bool _Ypol, bool _Zpol) {
  polarities[0] = _Xpol ? -1 : 1;
  polarities[1] = _Ypol ? -1 : 1;
  polarities[2] = _Zpol ? -1 : 1;
}

void ITG3200::setGains(float _Xgain, float _Ygain, float _Zgain) {
  gains[0] = _Xgain;
  gains[1] = _Ygain;
  gains[2] = _Zgain;
}

void ITG3200::setOffsets(int _Xoffset, int _Yoffset, int _Zoffset) {
  offsets[0] = _Xoffset;
  offsets[1] = _Yoffset;
  offsets[2] = _Zoffset;
}

int ITG3200::zeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS) {
  int a=0;
  int xyz[3]; 
  float tmpOffsets[] = {0,0,0};

  for (int i = 0;i < totSamples;i++){
    ms_delay(sampleDelayMS);
    readGyroRaw(xyz);
    //MYASSERT(a,"Failed to read sensor data for offset\n\r")
    tmpOffsets[0] += xyz[0];
    tmpOffsets[1] += xyz[1];
    tmpOffsets[2] += xyz[2];  
  }
  setOffsets(-tmpOffsets[0] / totSamples+0.5, -tmpOffsets[1] / totSamples+0.5, -tmpOffsets[2] / totSamples+0.5);
return 0;
  }

int ITG3200::readGyroRawCal(int *_GyroX, int *_GyroY, int *_GyroZ) {
  int a=0;
  readGyroRaw(_GyroX, _GyroY, _GyroZ);
  MYASSERT(a,"Failed to get gyro data readgyrorawcal\n\r")
  *_GyroX += offsets[0];
  *_GyroY += offsets[1];
  *_GyroZ += offsets[2];
    return 0;
  }

int ITG3200::readGyroRawCal(int *_GyroXYZ) {
  int a=0;
  a=readGyroRawCal(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
    MYASSERT(a,"Failed to read from readgyrorawcal\n\r")
    return 0;
  }

void ITG3200::readGyro(float *_GyroX, float *_GyroY, float *_GyroZ){
  int x, y, z;
  
  readGyroRawCal(&x, &y, &z); // x,y,z will contain calibrated integer values from the sensor
  *_GyroX =  x / 14.375 * polarities[0] * gains[0];
  *_GyroY =  y / 14.375 * polarities[1] * gains[1];
  *_GyroZ =  z / 14.375 * polarities[2] * gains[2];
}

void ITG3200::readGyro(float *_GyroXYZ){
  readGyro(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void ITG3200::reset() {     
 int a=0;
  a=writeTo( _dev_address,PWR_MGM, PWRMGM_HRESET); 
  ms_delay(GYROSTART_UP_DELAY); //gyro startup 
}

bool ITG3200::isLowPower() {   
  int a=0;
  a=readFrom( _dev_address,PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_SLEEP) >> 6;
}
  
void ITG3200::setPowerMode(bool _State) {
  int a=0;
  a=readFrom( _dev_address,PWR_MGM, 1, &_buff[0]);
  a=writeTo( _dev_address,PWR_MGM, ((_buff[0] & ~PWRMGM_SLEEP) | _State << 6));  
}

bool ITG3200::isXgyroStandby() {
  int a=0;
  a=readFrom( _dev_address,PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_XG) >> 5;
}

bool ITG3200::isYgyroStandby() {
  int a=0;
  a=readFrom( _dev_address,PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_YG) >> 4;
}

bool ITG3200::isZgyroStandby() {
  int a=0;
  a=readFrom( _dev_address,PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_ZG) >> 3;
}

void ITG3200::setXgyroStandby(bool _Status) {
  int a=0;
  a=readFrom( _dev_address,PWR_MGM, 1, &_buff[0]);
  a=writeTo( _dev_address,PWR_MGM, ((_buff[0] & PWRMGM_STBY_XG) | _Status << 5));
}

void ITG3200::setYgyroStandby(bool _Status) {
  int a=0;
  a=readFrom( _dev_address,PWR_MGM, 1, &_buff[0]);
  a=writeTo( _dev_address,PWR_MGM, ((_buff[0] & PWRMGM_STBY_YG) | _Status << 4));
}

void ITG3200::setZgyroStandby(bool _Status) {
  int a=0;
  a=readFrom( _dev_address,PWR_MGM, 1, &_buff[0]);
  a=writeTo( _dev_address,PWR_MGM, ((_buff[0] & PWRMGM_STBY_ZG) | _Status << 3));
}

byte ITG3200::getClockSource() {  
  int a=0;
  a=readFrom( _dev_address,PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_CLK_SEL);
}

int ITG3200::setClockSource(byte _CLKsource) {   
  int a=0;
  a=readFrom( _dev_address,PWR_MGM, 1, &_buff[0]);
  MYASSERT(a,"Failed to read clock source\n\r")
  a=writeTo( _dev_address,PWR_MGM, ((_buff[0] & ~PWRMGM_CLK_SEL) | _CLKsource)); 
    MYASSERT(a,"Failed to write clock source\n\r")
    return 0;
  }

/*
void ITG3200::a=writeTo( _dev_address,uint8_t _addr, uint8_t _val) {
  Wire.beginTransmission(_dev_address);   // start transmission to device 
  Wire.write(_addr); // send register address
  Wire.write(_val); // send value to write
  Wire.endTransmission(); // end transmission
  */

//void ITG3200::a=readFrom( _dev_address,uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
//  Wire.beginTransmission(_dev_address); // start transmission to device 
//  Wire.write(_addr); // sends register address to read from
//  Wire.endTransmission(); // end transmission
//  
//  Wire.beginTransmission(_dev_address); // start transmission to device 
//  Wire.requestFrom(_dev_address, _nbytes);// send data n-bytes read
//  uint8_t i = 0; 
//  while (Wire.available()) {
//    __buff[i] = Wire.read(); // receive DATA
//    i++;
//  }
//  Wire.endTransmission(); // end transmission
//}

//void ITG3200::zeroGyro() {
//  const int totSamples = 3;
//  float raw[3];
//  float tmpOffsets[] = {0,0,0};
//  
//  for (int i = 0; i < totSamples; i++){
//    readGyro(raw);
//    tmpOffsets[0] += raw[3];
//    tmpOffsets[1] += raw[4];
//    tmpOffsets[2] += raw[5];
//  }
//  
//  gyro_off_x = tmpOffsets[0] / totSamples;
//  gyro_off_y = tmpOffsets[1] / totSamples;
//  gyro_off_z = tmpOffsets[2] / totSamples;
//}
void *gyro_initialisation(byte _dev_address){
  int a=0;
  ITG3200* gyro=new ITG3200();
  a=gyro->init(_dev_address);
  if(a<0){
    usart_printfm(USARTx,(const int *)"failed to initialze gyro\n\r");
    return null;
  }
  a=gyro->zeroCalibrate(2500,2);
  
  if(a<0){
    usart_printfm(USARTx,(const int *)"failed to zero calibrate gyro\n\r");
    return null;
  }
  //gyro->zeroGyro();
 
  return (void *)gyro;
  
}

int gyro_xyz(void *gyro,float *xyz){
  int a=0;
  ((ITG3200*)gyro)->readGyro(xyz);
  //usart_printfm(USARTx,(const int *)"Xoffset=%d\n\r Yoffset=%d\n\r,Zoffset=%d\n\r",((ITG3200*)gyro)->offsets[0],((ITG3200*)gyro)->offsets[1],((ITG3200*)gyro)->offsets[2]);
  //MYASSERT(a,"Failed to read from gyro\n\r")
  return 0;
}

int gyro_xyz_raw(void *gyro,int *xyz){
  int a=0;
  ((ITG3200*)gyro)->readGyroRaw(xyz);
   MYASSERT(a,"Failed to read from gyro\n\r")
     return 0;
}
