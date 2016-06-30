#include "filter.h"
volatile unsigned int *DWT_CYCCNT  = (volatile unsigned int *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL = (volatile unsigned int *)0xE0001000; //address of the register
volatile unsigned int *SCB_DEMCR   = (volatile unsigned int *)0xE000EDFC; //address of the register
unsigned long int start=0;
void ResetTiming(void)
{
      *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
        *DWT_CYCCNT = 0; // reset the counter
        *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
 }
 

int accel_rollpitch(void *accel_object,float *accel_roll,float *accel_pitch){
  int a; float accel_data[3];
  float value_x,value_y,value_z;
  a=accel_xyz(accel_object,accel_data);
  if(a<0){
    usart_printf(USARTx,"Issue with reading the accel sensor value\n\r");
    return -1;
  }
  value_x=(accel_data[0]-8191)*4.0/16384;
  value_y=(accel_data[1]-8191)*4.0/16384;
  value_z=(accel_data[2]-8191)*4.0/16384;
  //usart_printf(USARTx,"Values of accel :\n\rRawAccel_x=%f\n\rRawAccel_y=%f\n\rRawAccel_z=%f\n\r",accel_data[0],accel_data[1],accel_data[2]);
  //usart_printf(USARTx,"Values of accel :\n\rAccel_x=%f\n\rAccel_y=%f\n\rAccel_z=%f\n\r",value_x,value_y,value_z);
  *accel_pitch=atan2f(value_y,value_z)*180/PI;
  *accel_roll=atan2f(value_x,value_z)*180/PI;
  return 0;
}

int gyro_rollpitch(void *gyro_object,float *gyro_roll_rate,float *gyro_pitch_rate){
  int a; float gyro_data[3];
  int gyro_data_raw[3];
  //unsigned int time=((*DWT_CYCCNT)-start)/10000000;
  //usart_printf(USARTx,"Time =%lu\n\r",time);
  //GPIOD->ODR ^= (1 << 13);
  //usart_printf(USARTx,"DWT_CYCCNT=%lu\n\r",*DWT_CYCCNT);
  float value_x,value_y,value_z;
  a=gyro_xyz(gyro_object,gyro_data);
  if(a<0){
    usart_printf(USARTx,"Issue with reading the gyro sensor value\n\r");
    return -1;
  }
  *gyro_roll_rate=gyro_data[0]*180/PI;
  *gyro_pitch_rate=gyro_data[1]*180/PI;
  //  *gyro_pitch+=gyro_data[1]*time;
  //  *gyro_roll+=gyro_data[0]*time;
  //  ResetTiming();
  //  start=*DWT_CYCCNT;
  //  usart_printf(USARTx,"Values of gyro :\n\rRawGyro_x=%f\n\rRawgyro_y=%f\n\rRawgyro_z=%f\n\r",gyro_data[0],gyro_data[1],gyro_data[2]);
  //  usart_printf(USARTx,"RawGyro_x=%d Rawgyro_y=%d Rawgyro_z=%d\n\r",gyro_data_raw[0],gyro_data_raw[1],gyro_data_raw[2]);
  return 0;
}

int magnet_yaw(void *magnet_object,float *mag_yaw){
  int a;
  float magnet_data[3];
  a=magnet_xyz(magnet_object,magnet_data);
  if(a<0){
    usart_printf(USARTx,"Issue with reading the magnet sensor value\n\r");
    return -1;
  }
  float heading = atan2f(magnet_data[1], magnet_data[0]);
  
  float declinationAngle = -0.4605;
  heading += declinationAngle;
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  
  float headingDegrees = heading * 180/PI;
  
  usart_printf(USARTx,"magnet_data_x=%f magnet_data_y=%f data_z=%f degress=%f\n\r",magnet_data[0],magnet_data[1],magnet_data[2],headingDegrees);
  return 0;
}

int kalman_roll_pitch(void* accel_object,void* gyro_object,float *final_roll,float *final_pitch){
  int a=0;
  float accel_roll,accel_pitch,gyro_roll_rate,gyro_pitch_rate;
  void *kalman;
  a=accel_rollpitch(accel_object,&accel_roll,&accel_pitch);
  if(a<0){
    usart_printf(USARTx,"Issue with reading the accel sensor value in kalman\n\r");
    return -1;
   }
   unsigned int time=((*DWT_CYCCNT)-start)/10000000;
   //usart_printf(USARTx,"DWT_CYCCNT=%lu\n\r",*DWT_CYCCNT);
   ResetTiming();
   start=*DWT_CYCCNT;
   a=gyro_rollpitch(gyro_object,&gyro_roll_rate,&gyro_pitch_rate);
   if(a<0){
    usart_printf(USARTx,"Issue with reading the gyro sensor value in kalman\n\r");
    return -1;
  }
   time=1;
   kalman=kalman_initialize();
  *final_roll=kalman_getAngle(kalman,accel_roll, gyro_roll_rate,time);
  *final_pitch=kalman_getAngle(kalman,accel_pitch,gyro_pitch_rate,time);
  return 0;
}