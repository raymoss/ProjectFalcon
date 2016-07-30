#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_conf.h"
#include "usart_print.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#define USARTx USART1     //Checker:Change this value if u want to initialize different usart
#ifndef motor_control
#define motor_control
#define byte uint8_t
#define TIMi TIM1
#define MYASSERT(a,message) if(a<0){ usart_printf(USARTx,const char *message); return -1;}
extern int kp,ki,kd,error_pitch,error_roll,error_pitch_d,error_roll_d,error_pitch_i,error_roll_i,throttle;
extern int motor1_error,motor2_error,motor3_error,motor4_error;
extern int motor_speed1,motor_speed2,motor_speed3,motor_speed4;
#ifdef __cplusplus
extern "C"{
#endif
void esc_initialize(TIM_TypeDef* TIMx);
void esc_write(uint16_t time,uint8_t channel_no);
#ifdef __cplusplus
}
#endif
 #endif