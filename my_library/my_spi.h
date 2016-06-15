//Functions for spi library
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_conf.h"
#include "usart_print.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#define SPI_TIMEOUT_MAX 10
#define USARTx USART1     //Checker:Change this value if u want to initialize different usart
#ifndef my_spi
#define my_spi
#define READWRITE_CMD              ((uint8_t)0x80)
#define DUMMY_BYTE                 ((uint8_t)0xFF)
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
#define SPIx SPI2       //Checker: Change this value if u want to initialize different SPIbus 
#define byte uint8_t
#define MYASSERT(a,message) if(a<0){ usart_printf(USARTx,const char *message); return -1;}
#ifdef __cplusplus
extern "C"{
#endif
void SPI_LowLevel_Init(void);
uint8_t SPI_SendByte(uint8_t byte);
uint8_t SPI_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
uint8_t SPI_Write(const uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
#ifdef __cplusplus
}
#endif
 #endif
// int transfer(byte _data);
// int begin(); 
// int beginTransaction();
// int endTransaction();
// int end();
// int setBitOrder(uint8_t);
// int setDataMode(uint8_t);
// int setClockDivider(uint8_t);
// int chipSelect();
// int transfernb(char*,char*);
// int transfern(char*,uint8_t);