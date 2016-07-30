#include "motor_control.h"
float time=1;
int kp=0,ki=0,kd=0,error_pitch,error_roll,error_pitch_d=0,error_roll_d=0,error_pitch_i=0,error_roll_i=0,throttle=750;
int motor1_error=0,motor2_error=0,motor3_error=0,motor4_error=0;
int motor_speed1,motor_speed2,motor_speed3,motor_speed4;
void NVCC_Config(){
NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM4 clock enable */
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* Enable the TIM4 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void TIM_Config_PWMOutput(){
    TIM_OCInitTypeDef outputChannelInit ;
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = 750;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    /*
    To get proper duty cycle, you have simple equation
    
    pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
    
    where DutyCycle is in percent, between 0 and 100%
    
    25% duty cycle:     pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
    50% duty cycle:     pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
    75% duty cycle:     pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
    100% duty cycle:    pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399
    
    Remember: if pulse_length is larger than TIM_Period, you will have output HIGH all the time
*/
    TIM_OC1Init(TIMi, &outputChannelInit);
    TIM_OC1PreloadConfig(TIMi, TIM_OCPreload_Enable);
    TIM_OC2Init(TIMi, &outputChannelInit);
    TIM_OC2PreloadConfig(TIMi, TIM_OCPreload_Enable);
    TIM_OC3Init(TIMi, &outputChannelInit);
    TIM_OC3PreloadConfig(TIMi, TIM_OCPreload_Enable);
    TIM_OC4Init(TIMi, &outputChannelInit);
    TIM_OC4PreloadConfig(TIMi, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIMi, ENABLE);
    
}
void TIM_Config_timebase(){
  
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);                          //Change it for appropiate timer
  uint16_t Prescalar= (uint16_t) 84;    // 255;
  /*    
    TIM4 is connected to APB1 bus, which has on F407 device 42MHz clock                 
    But, timer has internal PLL, which double this frequency for timer, up to 84MHz     
    Remember: Not each timer is connected to APB1, there are also timers connected     
    on APB2, which works at 84MHz by default, and internal PLL increase                 
    this to up to 168MHz                                                             
    
    Set timer prescaller 
    Timer count frequency is set with 
    
    timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)        
    
    In our case, we want a max frequency for timer, so we set prescaller to 0         
    And our timer will have tick frequency        
    
    timer_tick_frequency = 84000000 / (255 + 1) =  328125
*/    
  TIM_TimeBaseStructure.TIM_Prescaler = Prescalar ;
  /*
    Set timer period when it have reset
    First you have to know max value for timer
    In our case it is 16bit = 65535
    To get your frequency for PWM, equation is simple
    
    PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
    
    If you know your PWM frequency you want to have timer period set correct
    
    TIM_Period = timer_tick_frequency / PWM_frequency - 1
    
    In our case, for 10Khz PWM_frequency, set Period to
    
    TIM_Period = (328125 / 50) - 1 = 6561
    
    If you get TIM_Period larger than max timer value (in our case 65535),
    you have to choose larger prescaler and slow down timer tick frequency
*/
  TIM_TimeBaseStructure.TIM_Period = 19999; //6561;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit( TIMi, &TIM_TimeBaseStructure );
  // TIM_ITConfig( TIM4, TIM_IT_Update, ENABLE );
}
void GPIO_Config(){
GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
  
  /* GPIOC Configuration: TIM1 CH1 CH2 CH3 CH4 : PE9 PE11  PE13  PE14 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
}
void TIM_IRQHandler(){
  TIM_ClearFlag(TIM1, TIM_IT_Update);
  GPIOC->ODR ^= GPIO_Pin_6;
}
uint16_t timetotick(uint16_t time){
  return time;
}
void TIM_Config_PWM(){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
/* Setup TIM / PWM values
     Servo Requirements:  (May be different for your servo)
        - 50Hz (== 20ms) PWM signal
        - 0.6 - 2.1 ms Duty Cycle
     
     1. Determine Required Timer_Freq.
            TIM_Period = (Timer_Freq. / PWM_Freq) - 1
     
            - We need a period of 20ms (or 20000탎) and our PWM_Freq = 50Hz (i.e. 1/20ms)
            - See NOTES, for why we use 탎
            TIM_Period = 20000 - 1 = 19999  (since its 0 offset)
     
            Timer_Freq = (TIM_Period + 1) * PWM_Freq.
            Timer_Freq = (19999 + 1) * 50
            Timer_Freq = 1000000 = 1MHz
     
     2. Determine Pre-Scaler
        APB1 clock frequency:
            - SYS_CLK/4 when prescaler == 1 (i.e. 168MHz / 4 = 42MHz)
            - SYS_CLK/2 when prescaler != 1 (i.e. 168MHz / 2 = 84MHz)
     
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     Prescaler = APB1_Freq / Timer_Freq
        Prescaler = 84 MHz / 1 MHz
        Prescaler = 84
     
        For our example, we can prescale the TIM clock by 84, which gives us a Timer_Freq of 1MHz
            Timer_Freq = 84 MHz / 84 = 1 MHz
        So the TIMx_CNT register will increase by 1 000 000 ticks every second. When TIMx_CNT is increased by 1 that is 1 탎. So if we want a duty cycle of 1.5ms (1500 탎) then we can set our CCRx register to 1500.
     
     NOTES:
        - TIMx_CNT Register is 16 bits, i.e. we can count from 0 to (2^16)-1 = 65535
        - If the period, TIMx_ARR, is greater than the max TIMx_CNT value (65535), then we need to choose a larger prescaler value in order to slow down the count.
        - We use the 탎 for a more precise adjustment of the duty cycle
     
     */
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    TIM_OCInitTypeDef           TIM_OCInitStructure;
    uint16_t PrescalerValue = (uint16_t) 9;

    // Time Base Configuration
    TIM_TimeBaseStructure.TIM_Period        = 32280;
    TIM_TimeBaseStructure.TIM_Prescaler     = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIMi, &TIM_TimeBaseStructure);
    
    // Common TIM Settings
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;                        // Initial duty cycle
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIMi, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIMi, TIM_OCPreload_Enable);
    TIM_OC2Init(TIMi, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIMi, TIM_OCPreload_Enable);
    TIM_OC3Init(TIMi, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIMi, TIM_OCPreload_Enable);
    TIM_OC4Init(TIMi, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIMi, TIM_OCPreload_Enable);
   
    
    TIM_ARRPreloadConfig(TIMi, ENABLE);
    TIM_CtrlPWMOutputs(TIMi,ENABLE);
}
void esc_write(uint16_t time,uint8_t channel_no){
  
  if(time<=700){
  usart_printf(USARTx,"%d channel Reached lower bound speed\n\r",channel_no);
  time=700;
  }
  if(time>=2400){
  usart_printf(USARTx,"%d channel reached upper bound speed\n\r",channel_no);
  time=2400;
  }
  //tick=timetotick(time);
  if(channel_no==1){
    TIMi->CCR1=time*1.619+0.642;
  }else if(channel_no==2){
    TIMi->CCR2=time*1.619+0.642;
  }else if(channel_no==3){
    TIMi->CCR3=time*1.619+0.642;
  }else if(channel_no==4){
    TIMi->CCR4=time*1.619+0.642;
  }
  //usart_printf(USARTx,"Current speeed %d\n\r",time);
}
void esc_stop(){
  esc_write(700,1);
  esc_write(700,2);
  esc_write(700,3);
  esc_write(700,4);
}
void esc_initialize(TIM_TypeDef* TIMx){
 // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); //change this if different timer then TIM4
  GPIO_Config();
  TIM_Config_PWM();
  
  //TIM4_Config_PWMOutput();    //default configuration . Speed at 750 ms
  TIM_Cmd(TIMx, ENABLE);
  esc_write(700,1);
  esc_write(700,2);
  esc_write(700,3);
  esc_write(700,4);
}

void pid_control(float *final_roll,float *final_pitch){
  //motor_speed=kp*error+ki*error*dt+kd*error/dt
  
  
  error_roll=(int)(*final_roll)/2;            //need to modify this statement after taking user input
  error_pitch=(int)(*final_pitch)/2;
  error_roll_i+=error_roll;
  error_pitch_i+=error_pitch;
  motor_speed2=throttle - (kp*error_roll+ki*(error_roll_i)*time+kd*(error_roll-error_roll_d)*time) + motor2_error;
  motor_speed4=throttle + (kp*error_roll+ki*(error_roll_i)*time+kd*(error_roll-error_roll_d)*time) + motor4_error;
  motor_speed1=throttle - (kp*error_pitch+ki*(error_pitch_i)*time+kd*(error_pitch-error_pitch_d)*time) + motor1_error;
  motor_speed3=throttle + (kp*error_pitch+ki*(error_pitch_i)*time+kd*(error_pitch-error_pitch_d)*time) + motor3_error;
  if( (700<motor_speed1<2400) && (700<motor_speed2<2400) && (700<motor_speed3<2400) && (700<motor_speed4<2400)){
  esc_write(motor_speed1,1);
  esc_write(motor_speed2,2);
  esc_write(motor_speed3,3);
  esc_write(motor_speed4,4);
  }
  error_roll_d=error_roll;
  error_pitch_d=error_pitch;
}