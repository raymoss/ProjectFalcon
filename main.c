#include "stm32f4xx.h"
#include "usart_print.h"
//#include "stm32f4xx_conf.h"
#include "ADXL345.h"
#include "HMC58X3.h"
#include "ITG3200.h"
#include "my_i2c.h"
#define null (void*)0
//extern void *accelerometer_initialization(byte device_address);
//extern int accel_xyz(void *accel,int accel_data[]);
//extern void *magnetometer_initialisation(byte _dev_address);
//extern int magnet_xyz(void *magnet,int *xyz);
void main(void){
SystemInit();
  int a=0;
void *magnet_object;
void *accel_object;
int accel_data[3];
int magnet_data[3];
int gyro_data[3];
void *gyro_object;
//basic led blinking to verify if the code is working or not
RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable the clock to GPIOD
GPIOD->MODER = (1 << 26);             // set pin 13 to be general purpose output
GPIOD->ODR ^= (1 << 13);           // Toggle the pin 
usart_initialization(USART1);
i2c_initialize(I2C2);
usart_printf(USART1,"Hello\n\r");
uint8_t accel_address=0x53;
uint8_t magnet_address=0x1E;
uint8_t gyro_address=ITG3200_ADDR_AD0_LOW;
accel_object=accelerometer_initialization(accel_address);
magnet_object=magnetometer_initialisation(magnet_address);
gyro_object=gyro_initialisation(gyro_address);
if(gyro_object==null){
  usart_printf(USART1,"Failed to initialize gyro1\n\r");
  while(1);
}
usart_printf(USART1,"Initialized gyro\n\r");
if(magnet_object==null){
  usart_printf(USART1,"Failed to initialize magnet\n\r");
  while(1);
}
usart_printf(USART1,"Initialized magnet\n\r");
if(accel_object==null){
  usart_printf(USART1,"Failed to initialize accel\n\r");
  while(1);
}
usart_printf(USART1,"Initialized accel\n\r");
int i=0;
ms_delay(1000);

for (;;) {
  
  GPIOD->ODR ^= (1 << 13);           // Toggle the pin 
  a=magnet_xyz(magnet_object,magnet_data);
  if(a<0){
    usart_printf(USART1,"Something is fishy!!Reinitializing\n\r");
    //magnet_object=magnetometer_initialisation(magnet_address);
    magnet_object=magnetometer_initialisation(magnet_address);
    while(magnet_object==null){
      usart_printf(USART1,"Something is fishy!!Reinitializing\n\r");
      magnet_object=magnetometer_initialisation(magnet_address);
      //magnet_object=magnetometer_initialisation(magnet_address); 
    ms_delay(1000);
    }}
  else{
    for(i=0;i<3;i++)
      usart_printf(USART1,"magnet_Data a[%d]: %d\n\r",i,magnet_data[i]);
  }
  a=gyro_xyz(gyro_object,gyro_data);
  if(a<0){
    usart_printf(USART1,"Something is fishy!!Reinitializing\n\r");
    gyro_object=gyro_initialisation(gyro_address);
    while(gyro_object==null){
      usart_printf(USART1,"Something is fishy!!Reinitializing\n\r");
      gyro_object=gyro_initialisation(gyro_address); 
    ms_delay(1000);
    }}
  else{
    for(i=0;i<3;i++)
      usart_printf(USART1,"gyro_Data a[%d]: %d\n\r",i,gyro_data[i]);
  }
  a=accel_xyz(accel_object,accel_data);  
  if(a<0){
    usart_printf(USART1,"Something is fishy!!Reinitializing\n\r");
    accel_object=accelerometer_initialization(accel_address);
    while(accel_object==null){
      usart_printf(USART1,"Something is fishy!!Reinitializing\n\r");
      accel_object=accelerometer_initialization(accel_address); 
    ms_delay(1000);
    }}
  else{
    for(i=0;i<3;i++)
      usart_printf(USART1,"accel_Data a[%d]: %d\n\r",i,accel_data[i]);
  }
}
}
