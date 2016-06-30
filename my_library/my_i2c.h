#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_conf.h"
#include "usart_print.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#define I2C_TIMEOUT_MAX 1000
#define USARTx USART1     //Checker:Change this value ifu want to initialize different usart
#ifndef my_i2c
#define my_i2c
//#define assert(a)       {if(a<0) return ;}
#define I2Ci I2C2       //Checker: Change this value if u want to initialize different I2Cbus 
#define byte uint8_t
//#define bool uint8_t
#define i2c_write 0
#define i2c_read  1
#ifdef __cplusplus
extern "C"{
#endif
int i2c_beginTransmission(I2C_TypeDef* I2Cx,uint8_t device_address,uint8_t operation);
int i2c_sendData(I2C_TypeDef* I2Cx,int data);
int i2c_stopTransmission(I2C_TypeDef* I2Cx);
//uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
//uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
int readFrom(byte _dev_address,byte address, int num, byte _buff[]);
int writeTo(byte _dev_address,byte address, byte val);
int i2cInitialize(I2C_TypeDef* I2C);

#ifdef __cplusplus
}
#endif


int accelerometer_initialize(I2C_TypeDef* I2C);
int gyroscope_initialize(I2C_TypeDef* I2C);
int magnetometer_initialize(I2C_TypeDef* I2C);
int accelerometer_read(I2C_TypeDef* I2C);
int gyroscope_read(I2C_TypeDef* I2C);
int magnetometer_read(I2C_TypeDef* I2C);

#endif

