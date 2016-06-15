#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "usart_print.h"
//#include "stm32f4xx_conf.h"
#include "RF24.h"
#define null (void*)0

void main(void){
int a=0;
void *radio;
char c;
SystemInit();
//basic led blinking to verify if the code is working or not
RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable the clock to GPIOD
GPIOD->MODER = (1 << 26);             // set pin 13 to be general purpose output
GPIOD->ODR ^= (1 << 13);           // Toggle the pin 
usart_initialization(USART1);
usart_printf(USART1,"Hello\n\r");
//ms_delay(5000);
radio=comms_initialize(radio);
//usart_printf(USART1,"Holy cow!!!Initialise was a success!!\n\r");
while(radio==NULL){
  usart_printf(USART1,"Failed to initialize the comms sensor.Reinitialising...\n\r");
  ms_delay(3000);
  radio=comms_initialize();
}
usart_printf(USART1,"Holy cow!!!Initialise was a success!!\n\r");
for(;;){
  a=ping_pong(radio);
  //usart_printf(USART1,"Last transaction is completed with error code %d\n\r",a);
  }
}


//sudo airmon-ng start wlan0
//sudo iw dev wlan0 set channel 36 HT40+











//while(1){
//  if(USART_available(USART1)>0){
//    USART_getc(USART1,&c);
//    usart_printf(USART1,"Got this value %c\n\r",c);
//  }
//}
//}









  