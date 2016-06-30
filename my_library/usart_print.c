#include "usart_print.h"

void ms_delay(int ms);
void usart_printf(USART_TypeDef* USARTx,const char *format, ...);
void USART_puts(USART_TypeDef* USARTx,const char *s);
void USART_putc(USART_TypeDef* USARTx, char c);

void ms_delay(int ms){
	while (ms-- > 0) {
      volatile int x=1990;
      while (x-- > 0)
         __asm("nop");
   }
}
      void micro_delay(int ms){
        while (ms-- > 0) {
      volatile int x=2;
      while (x-- > 0)
         __asm("nop");
   }
        }
void usart_printf2(char *buffer, int char_no){
while (char_no != 0){
while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1,*buffer);
buffer++;
char_no--;
}
}

void usart_printf(USART_TypeDef* USARTx,const char *format, ...) {
 va_list list;
 va_start(list, format);
 int len = vsnprintf(0, 0, format, list);
 char *s; 
s = (char *)malloc(len + 1);
 vsprintf(s, format, list);
USART_puts(USARTx,s);
 free(s);
 va_end(list);
 return;
}

void USART_putc(USART_TypeDef* USARTx,char c){
while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
USART_SendData(USARTx,c);
}

void USART_puts(USART_TypeDef* USARTx,const char *s){
 int i;
 for(i=0;s[i]!='\0';i++) USART_putc(USARTx,s[i]);
}
void USART_getc(USART_TypeDef* USARTx, char *c){
 uint16_t data;
 while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE)==RESET);
 data=USART_ReceiveData(USARTx);
 *c=(data & 0xFF);
}
void usart_scanf(USART_TypeDef* USARTx, char *c){
USART_getc(USARTx,c);
}

int USART_available(USART_TypeDef* USARTx){
  if(USART_GetFlagStatus(USARTx,USART_FLAG_RXNE)==SET)
    return 1;
  else 
    return -1;
}

void usart_printfm(USART_TypeDef* USARTx,const int *format,...){
usart_printf(USARTx,(const char*)format);
}
//public funtions list
/*
void *_sbrk(ptrdiff_t increment)
{
extern char _end;
static char *heap_end = 0;
char *prev_heap_end;
char register *stack_ptr asm("sp");
if (heap_end == 0) {
heap_end = &_end;
}
prev_heap_end = heap_end;
if ((heap_end + increment) > stack_ptr) {
USART_puts("sp=");
//PUTP(stack_ptr);
USART_puts("\n");
USART_puts("heap=");
//PUTP(heap_end);
USART_puts("\n");
USART_puts("sbrk: no more heap\n");
//errno = ENOMEM;

return (void*)-1;
}
heap_end += increment;
return prev_heap_end;
}
*/

void usart_initialization(USART_TypeDef* USARTx){
	USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  
  if(USARTx == USART1){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
  
  /* Configure USART1 pins:  Rx and Tx ----------------------------*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx  ;
  USART_Init(USART1, &USART_InitStructure);
  
  USART_Cmd(USART1,ENABLE);
  }
  if(USARTx == USART3) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
  
  /* Configure USART1 pins:  Rx and Tx ----------------------------*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 NVIC_InitTypeDef NVIC_InitStructure;
NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3,USART_FLAG_TXE | USART_FLAG_RXNE,ENABLE);
  USART_Cmd(USART3,ENABLE);
  
  
  }
 }

