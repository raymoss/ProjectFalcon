#include"comms_misc.h"
void *radio;

uint8_t comms_initialisation(){
  uint8_t counter=3;
  while(counter!=0){
  radio=comms_initialize();
  if(radio!=NULL)
    break;
  else 
    counter--;
  ms_delay(3000);
  }
  if(counter==0){
    usart_printf(USARTx,"Failed to initialise the comms module\n\r");
    return 1;
  }
  return 0;  
}

uint8_t sendYawAngle(){
comms_data c;
c.code=YAW_ANGLE;
c.value=final_yaw;
return writeComms(radio,c);
}

uint8_t sendRollAngle(){
comms_data c;
c.code=ROLL_ANGLE;
c.value=(int)final_roll;
//usart_printf(USARTx,"Final_roll=%d\n\r",c.value);
return writeComms(radio,c);
}
uint8_t pingReport(){
comms_data c;
c.code=PING;
c.value=0;
return writeComms(radio,c);
}
uint8_t sendPitchAngle(){
comms_data c;
c.code=PITCH_ANGLE;
c.value=(int)final_pitch;
return writeComms(radio,c);
}

uint8_t sendInitialisationError(uint8_t value){
comms_data c;
c.code=INITIALISE;
c.value=value;
return writeComms(radio,c);
}

uint8_t sendRuntimeError(uint8_t value){
comms_data c;
c.code=RUNTIME;
c.value=value;
return writeComms(radio,c);
}

uint8_t motorControl(comms_data c){
uint8_t code=c.code;
int value=c.value;
if(code==RUN){
  if(value==1)
    run_motor=1;
  if(value==2)
    run_motor=2;
  }
else if(code==MOTOR1_ERROR_VALUE){
  motor1_error=value;
  usart_printf(USARTx,"Set the value of motor1error=%u",c.value);
}
else if(code==MOTOR2_ERROR_VALUE)
  motor2_error=value;
else if(code==MOTOR3_ERROR_VALUE)
  motor3_error=value;
else if(code==MOTOR4_ERROR_VALUE)
  motor4_error=value;
else if(code==MOTOR1_SPEED){
  c.code=MOTOR1_SPEED;
  c.value=motor_speed1;
  usart_printf(USARTx,"Set the value of motor1speed=%u",c.value);
  return writeComms(radio,c);
}
else if(code==MOTOR2_SPEED){
  c.code=MOTOR2_SPEED;
  c.value=motor_speed2;
  return writeComms(radio,c);
}
else if(code==MOTOR3_SPEED){
  c.code=MOTOR3_SPEED;
  c.value=motor_speed3;
  return writeComms(radio,c);
}
else if(code==MOTOR4_SPEED){
  c.code=MOTOR4_SPEED;
  c.value=motor_speed4;
  //usart_printf(USARTx,"Set the value of motor4speed=%u",c.value);
  return writeComms(radio,c);
}
else if(code==THROTTLE){
throttle=c.value;
//usart_printf(USARTx,"Set the value of motor1speed=%u",c.value);
}
return 0; 
  }

uint8_t motorPID(comms_data c){
  uint8_t code=c.code;
  int value=c.value;
  if(code==KP_VALUE){
    kp=value;
    usart_printf(USARTx,"Set the value of kp=%u",c.value);
  }
  if(code==KI_VALUE)
    ki=value;
  if(code==KD_VALUE)
    kd==value;
  return 0;
}

