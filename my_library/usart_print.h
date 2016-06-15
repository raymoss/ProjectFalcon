#include "stm32f4xx.h"
#include "misc.h"
#include "stm32f4xx_conf.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/* public functions */
//void *_sbrk(ptrdiff_t increment);

#ifdef __cplusplus
extern "C"{
#endif
void usart_printfm(USART_TypeDef* USART,const int* foramt,...);
void usart_initialization(USART_TypeDef* USART);
void usart_printf(USART_TypeDef* USART,const char *format, ...);
void usart_scanf(USART_TypeDef* USART,char *);
void ms_delay(int ms);
void micro_delay(int ms);
int USART_available(USART_TypeDef* USART);
#ifdef __cplusplus
}
#endif
