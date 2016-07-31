#include "main.h"
float final_roll=0.0 , final_pitch=0.0,final_yaw=0.0;
static uint32_t duration=0;
uint8_t tag=0,fail_event=0;
uint8_t run_motor=2;
uint8_t Systick_Init(){
  return SysTick_Config(SystemCoreClock/1000);
}

void SysTick_Handler(void){
  duration++;
}
void delay(uint32_t value){
  uint32_t current=duration;
  while(((duration-current))<value);
  return ;
}
//TODO: Understand kalman filter and add the bias to the matrix. Tune Kp,Ki,Kd parameters.
void main(void){
int a=0;
char c;
uint8_t a1=1;
 SystemInit();
 a=Systick_Init();
//basic led blinking to verify if the code is working or not
RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable the clock to GPIOD
GPIOD->MODER = (1 << 26);             // set pin 13 to be general purpose output
//GPIOD->ODR ^= (1 << 13);           // Toggle the pin 
float i[3]={0.0,0.0,0.0};
float pi=3.14;
usart_initialization(USART1);
usart_printf(USART1,"Hello\n\r");
esc_initialize(TIM1);
IMU_Initialisation();
kalman_initialization();
comms_initialisation();
GPIOD->ODR = (1 << 13);
if(a==1)
  usart_printf(USART1,"Failed to initialize systick\n\r");
for (;;){
  if(fail_event>=5 || duration>=30000 || run_motor!=1){
  run_motor=2;
  esc_stop();
  usart_printf(USART1,"Duration expired!!\n\r");
  }
  if(commsAvailable(radio)>0){
    duration=0;
    comms_data c;
    a1=readComms(radio,&c);
    if(a1==1){
      usart_printf(USART1,"Failed to read from comms\n\r");
      continue;
    }
    //usart_printf(USART1,"Got something\n\r");
    //usart_printf(USART1,"c.code=%d,c.value=%d\n\r",c.code,c.value);
    if(c.code==PING){
      a1=pingReport();
      if(a1==0){
      tag=0;
      fail_event=0;
      }
      if(a1==1){
      if(tag==0){
        tag=1;
        fail_event=1;
      } else 
          fail_event++;
    usart_printf(USART1,"Failed to write from comms\n\r");
      }
      
      continue;
    }
    if(c.code>=RUN && c.code<=THROTTLE)
      a1=motorControl(c);
    if(KP_VALUE<=c.code<=KD_VALUE)
      a1=motorPID(c);
    if(c.code==ROLL_ANGLE){
      a1=sendRollAngle();
    if(a1==1){
      usart_printf(USART1,"Failed to write from comms\n\r");
      continue;
    }}
    if(c.code==PITCH_ANGLE){
      a1=sendPitchAngle();
    if(a1==1){
      usart_printf(USART1,"Failed to write from comms\n\r");
      continue;
    }
    }
  }
  
  if(run_motor==1){
    //usart_printf(USART1,"Starting motor\n\r");
    a=kalman_roll_pitch(&final_roll,&final_pitch);
    if(a<0){
      usart_printf(USARTx,"Something fishy with kalman_rollpitch value\n\r");
      continue;
    }
    pid_control(&final_roll,&final_pitch);
    //usart_printf(USARTx,"%f,%f\n\r",final_roll,final_pitch);
    //delay(1000);
    }
  }

}


///plotting code test
//for(;;){
//  if( i[0]>2*pi)
//    i[0]-=2*pi;
//  if( i[1] > 2*pi)
//    i[1]-=2*pi;
//  if( i[2] > 2*pi)
//    i[2]-=2*pi;
//  
//  //serialPrintFloatArr(i,3);
//  usart_printf(USART1,"%02f,%02f,%02f\r\n",i[0],i[1],i[2]);
//  i[0]+=0.1;
//  //delay(10);
//
//}



//for(;;){
//  if(usart_available(USART1)>0){
//  usart_scanf(USART1,&c);
//  usart_printf(USART1,"Got this value %c\n\r",c);
//  
//  switch(c){
//    case '1':
//        esc_write(800,1);
//        esc_write(800,2);
//        esc_write(800,3);
//        esc_write(800,4);
//        break ;
//      case '2':
//        esc_write(900,1);
//        esc_write(900,2);
//        esc_write(900,3);
//        esc_write(900,4);
//        break ;
//      case '3':
//        esc_write(2400,1);
//        esc_write(2400,2);
//        esc_write(2400,3);
//        esc_write(2400,4);
//        break;
//      case '4':
//        esc_write(2400,1);
//        esc_write(2400,2);
//        esc_write(2400,3);
//        esc_write(2400,4);
//        break;
//      case '5':
//        esc_write(700,1);
//        esc_write(700,2);
//        esc_write(700,3);
//        esc_write(700,4);
//        break;
//      }
//  }
//  }