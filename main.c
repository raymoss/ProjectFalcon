#include "motor_control.h"
//timming is screwed . NEed to mend the timmings again . 
void main(){
    SystemInit();
    usart_initialization(USART1);
    usart_printf(USART1,"Starting with PWM\n\r");
    esc_initialize(TIM1);
    char c;
    while(1){
      if(USART_available(USART1)>0){
      usart_scanf(USART1,&c);
      usart_printf(USART1,"Got this value %c\n\r",c);
      switch(c){
      case '1':
        esc_write(800,1);
        esc_write(800,2);
        esc_write(800,3);
        esc_write(800,4);
        break ;
      case '2':
        esc_write(900,1);
        esc_write(900,2);
        esc_write(900,3);
        esc_write(900,4);
        break ;
      case '3':
        esc_write(2400,1);
        esc_write(2400,2);
        esc_write(2400,3);
        esc_write(2400,4);
        break;
      }
      
      }
    }
  }
