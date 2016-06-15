#include "my_i2c.h"
/*Checker:Initializing the i2c2 peripheral*/ 


 void i2c_initialize(I2C_TypeDef* I2Cx){
  GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as I2CxSDA and I2CxSCL
  I2C_InitTypeDef I2C_InitStruct; // this is for the I2Cx initilization
 
	/* enable APB1 peripheral clock for I2Cx*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
 
	/* enable the peripheral clock for the pins used by
	 PB10 for I2Cx SCL and PB11 for I2Cx_SDL*/
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        
       
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11; 
        /* Pins 10(I2Cx_SCL) and 11(I2Cx_SDA) */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 
        /* the pins are configured as alternate function so the USART peripheral has access to them*/
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
        /* this defines the IO speed and has nothing to do with the baudrate!*/
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
        /* this defines the output type as open drain*/
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        /* this activates the pullup resistors on the IO pins*/
	GPIO_Init(GPIOB,&GPIO_InitStruct);
        
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_I2C2);
         GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_I2C2);   
   /* This sequence sets up the I2CxSDA and I2CxSCL pins
	 * so they work correctly with the I2Cx peripheral
	 */
    I2C_DeInit(I2Cx);
    
    I2C_InitStruct.I2C_Mode= I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle=I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1= 0xEE;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_ClockSpeed=100000;
    
    I2C_Init(I2Cx,&I2C_InitStruct);
    I2C_Cmd(I2Cx,ENABLE);
}
int i2c_beginTransmission(I2C_TypeDef* I2Cx,uint8_t device_address,uint8_t operation){
  int timeout;
  uint32_t flag1=0,flag2=0;
  device_address= device_address << 1;
  I2C_GenerateSTART(I2Cx, ENABLE);
  //flag1 = I2Cx->SR1;
  //flag2 = I2Cx->SR2;
  //usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
  //usart_printf(USARTx,"Sending start bit to address:%02x\n\r",device_address);
       /* Test on I2Cx EV5, Start trnsmitted successfully and clear it */
       timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
       
       while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
       {
            /* If the timeout delay is exeeded, exit with error code */
         ms_delay(100);
         if ((timeout--) == 0){ 
           flag1 = I2Cx->SR1;
           flag2 = I2Cx->SR2;
           usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
           usart_printf(USARTx,"Timeout reached while checking the EV5 event\n\r");
           return -1;
           
         }}
       timeout = I2C_TIMEOUT_MAX;
       if (!operation){
          I2C_Send7bitAddress(I2Cx, device_address, I2C_Direction_Transmitter);
          //timeout = I2C_TIMEOUT_MAX;
          while (!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        {
            /* If the timeout delay is exeeded, exit with error code */
         ms_delay(100);
         if ((timeout--) == 0){ 
           flag1 = I2Cx->SR1;
           flag2 = I2Cx->SR2;
           usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
           usart_printf(USARTx,"Timeout reached while verifying transmitter selection\n\r");
           return -2;
           
         }}
      //usart_printf(USARTx,"Transmitter mode selected. Send the address\n\r");
       }
       else{
          I2C_Send7bitAddress(I2Cx, device_address, I2C_Direction_Receiver);
          while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
      {
            /* If the timeout delay is exeeded, exit with error code*/ 
        ms_delay(100);
        if ((timeout--) == 0){
          flag1 = I2Cx->SR1;
           flag2 = I2Cx->SR2;
           usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
          usart_printf(USARTx,"Timeout reached while recieving acknowledge on revice mode \n\r");
          return -2;
      }
      }}
//usart_printf(USARTx,"Success with stage one. Going to senddata\n\r");
return 0;
}

int i2c_sendData(I2C_TypeDef* I2Cx,int data){
  I2C_SendData(I2Cx, data);
  //usart_printf(USARTx,"Sending data:%d\n\r",data);      
  int timeout;
        uint8_t flag1=0,flag2=0;
        /* Test on I2C1 EV8 and clear it */
        timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
         
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
        {
             
          /* If the timeout delay is exeeded, exit with error code */
          if ((timeout--) == 0) {
            flag1 = I2Cx->SR1;
           flag2 = I2Cx->SR2;
           usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
           usart_printf(USARTx,"Failing at sending data");
            return -3;
            
          }
        }  
return 0;
}
int i2c_stopTransmission(I2C_TypeDef* I2Cx){
I2C_GenerateSTOP(I2Cx, ENABLE);
return 0;
}
int I2C_read(I2C_TypeDef* I2Cx,byte buff ){
  
  // enable acknowledge of recieved data
        int timeout;
        uint8_t flag1=0,flag2=0;
        /* Test on I2C1 EV8 and clear it */
        timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
         I2C_AcknowledgeConfig(I2Cx, ENABLE);
        //
	// wait until one byte has been received
       
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ){
          ms_delay(100);
          if((timeout--)==0){
            flag1 = I2Cx->SR1;
            flag2 = I2Cx->SR2;
            usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
            usart_printf(USARTx,"Failing at read ack stage\n\r");
            return -1;
          }
        }
        // read data from I2C data register and return data byte
	byte data = I2C_ReceiveData(I2Cx);
	return 0;
  }
int readFrom(byte _dev_address,byte address, int num, byte _buff[]) {
  int a=0,i=0;
  int timeout;
  uint8_t flag1=0,flag2=0;
  a=i2c_beginTransmission(I2Ci,_dev_address,i2c_write); // start transmission to device
  if(a<0) return -1;
 // usart_printfm(USART1,(const int *)"Reading from this device address: %2x\n\r",_dev_address);
  a=i2c_sendData(I2Ci,address);             // sends address to read from
  if(a<0) return -1;
  //usart_printfm(USART1,(const int *)"sent this data which is the address to read from: %2x\n\r",address);
  //  if(a<0)
//     return (a-1);
  //i2c_stopTransmission(I2Cx);         // end transmission
//  if(a<0)
//     return (a-2);
  a=i2c_beginTransmission(I2Ci,_dev_address,i2c_read); // start transmission to device
  if(a<0) return -1;
  //usart_printfm(USART1,(const int *)"Initiated the register read cycle: %2x\n\r",_dev_address);
  //  if(a<0)
//     return a;
  // request 6 bytes from device
  if(num==1){
    I2C_AcknowledgeConfig(I2Ci, DISABLE);
	
        
        /* Test on I2C1 EV8 and clear it */
        timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
         
        
	// wait until one byte has been received
      
	while( !I2C_CheckEvent(I2Ci, I2C_EVENT_MASTER_BYTE_RECEIVED) ){
          ms_delay(100);
          if((timeout--)==0){
            flag1 = I2Ci->SR1;
            flag2 = I2Ci->SR2;
            usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
            usart_printf(USARTx,"Failing at read nnack stage\n\r");
            return -1;
          }
        }
	// wait until one byte has been received
	 I2C_GenerateSTOP(I2Ci, ENABLE);
	// read data from I2C data register and return data byte
	_buff[i] = I2C_ReceiveData(I2Ci);
        return 0;
  }
  if(num==2)
  {
    timeout = I2C_TIMEOUT_MAX; 
//    while(I2C_GetFlagStatus(I2Ci,I2C_FLAG_ADDR)!=SET){
//     ms_delay(100);
//          if((timeout--)==0){
//            flag1 = I2Ci->SR1;
//            flag2 = I2Ci->SR2;
//            usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
//            usart_printf(USARTx,"Failing at reading 2 bytes stage\n\r");
//            return ;
//          }}
    I2C_AcknowledgeConfig(I2Ci, DISABLE);
    I2C_NACKPositionConfig(I2Ci, I2C_NACKPosition_Next);
    timeout = I2C_TIMEOUT_MAX; 
    while(I2C_GetFlagStatus(I2Ci,I2C_FLAG_BTF)!=SET){
      ms_delay(100);
          if((timeout--)==0){
            flag1 = I2Ci->SR1;
            flag2 = I2Ci->SR2;
            usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
            usart_printf(USARTx,"Failing at reading 2 bytes stage2\n\r");
            return -1;
          }}
    
    i2c_stopTransmission(I2Ci);
    _buff[i] = I2C_ReceiveData(I2Ci);
    i++;
    _buff[i] = I2C_ReceiveData(I2Ci);
    num-=2;
    // receive a byte
    //usart_printfm(USART1,(const int *)"this byte read from accel: %2x\n\r",i);
   return 0; 
  }
  while(num>0)         // device may send less than requested (abnormal)
  if(num==3){
  //_buff[i]=I2C_read_ack(I2Cx);
    timeout = I2C_TIMEOUT_MAX; 
    while(I2C_GetFlagStatus(I2Ci,I2C_FLAG_BTF)!=SET){
     ms_delay(100);
          if((timeout--)==0){
            flag1 = I2Ci->SR1;
            flag2 = I2Ci->SR2;
            usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
            usart_printf(USARTx,"Failing at reading 2 bytes stage2\n\r");
            return -1;
          }}
  I2C_AcknowledgeConfig(I2Ci, DISABLE);
  _buff[i]=I2C_ReceiveData(I2Ci);
  
  i++;
  num--;
  while(I2C_GetFlagStatus(I2Ci,I2C_FLAG_BTF)!=SET)
    {
     ms_delay(100);
          if((timeout--)==0){
            flag1 = I2Ci->SR1;
            flag2 = I2Ci->SR2;
            usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
            usart_printf(USARTx,"Failing at reading 2 bytes stage2\n\r");
            return -1;
          }}
  i2c_stopTransmission(I2Ci);
  _buff[i]=I2C_ReceiveData(I2Ci);
  i++;
  _buff[i]=I2C_ReceiveData(I2Ci);
  i++;
  num-=2;
  //usart_print(USART1,"This byte read from nack: %2x\n\r",i);
  return 0; 
  }
  else{
    a=I2C_read(I2Ci,_buff[i]);
     if(a<0) return -1;
    i++;
    num--;
  }
//  if(i != num){
//    status = ADXL345_ERROR;
//    error_code = ADXL345_READ_ERROR;
//  }
  //ms_delay(10);
  //i2c_stopTransmission(I2Cx);         // end transmission
//end :
//  usart_printf(USARTx,"Error occured !! Restarting the interface!!");
//  ms_delay(5000);
//  return -1;
return 0;
}

int writeTo(byte _dev_address,byte address, byte val) {
  int a=0;
  a=i2c_beginTransmission(I2Ci,_dev_address,i2c_write); // start transmission to device
  //usart_printfm(USART1,(const int *)"Writing to this device address: %2x\n\r",_dev_address);
   if(a<0) return -1;
  a=i2c_sendData(I2Ci,address);             // send register address
   if(a<0) return -1; 
  a=i2c_sendData(I2Ci,val);  // send value to write
  if(a<0) return -1;
  while(I2C_GetFlagStatus(I2Ci,I2C_FLAG_BUSY)!=SET);
  a=i2c_stopTransmission(I2Ci);         // end transmission
return 0;
}


/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
//uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
//	// disabe acknowledge of received data
//	// nack also generates stop condition after last byte received
//	// see reference manual for more info
//	I2C_AcknowledgeConfig(I2Cx, DISABLE);
//	int timeout;
//        uint8_t flag1=0,flag2=0;
//        /* Test on I2C1 EV8 and clear it */
//        timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
//         
//        
//	// wait until one byte has been received
//      
//	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ){
//          ms_delay(100);
//          if((timeout--)==0){
//            flag1 = I2Cx->SR1;
//            flag2 = I2Cx->SR2;
//            usart_printf(USARTx,"Flag1:%04x \n\r Flag2:%04x\n\r",flag1,flag2);
//            usart_printf(USARTx,"Failing at read nnack stage\n\r");
//            return -1;
//          }
//        }
//	// wait until one byte has been received
//	 I2C_GenerateSTOP(I2Cx, ENABLE);
//	// read data from I2C data register and return data byte
//	uint8_t data = I2C_ReceiveData(I2Cx);
//         //I2C_AcknowledgeConfig(I2Cx, ENABLE);
//	//I2C_GenerateSTOP(I2Cx, ENABLE);
//        return data;
//}
