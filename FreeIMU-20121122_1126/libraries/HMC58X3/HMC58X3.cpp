/*
HMC58X3.cpp - Interface a Honeywell HMC58X3 or HMC5883L magnetometer to an Arduino via i2c
Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>

Based on:
http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1274748346
 Modification/extension of the following by E.J.Muller
http://eclecti.cc/hardware/hmc5843-magnetometer-library-for-arduino
 Copyright (c) 2009 Nirav Patel, 

The above were based on:
http://www.sparkfun.com/datasheets/Sensors/Magneto/HMC58X3-v11.c
http://www.atmel.com/dyn/resources/prod_documents/doc2545.pdf


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

//#define DEBUG (1)

#include <cstdint>
#include "HMC58X3.h"
#include "my_i2c.h"
#include "stm32f4xx_i2c.h"
#include "usart_print.h"

/*!
    Counts/milli-gauss per gain for the self test bias current.
*/
#if defined(ISHMC5843)
  const int counts_per_milligauss[8]={  
    1620,
    1300,
     970,
     780,
     530,
     460,
     390,
     280
  };
#else // HMC5883L
  const int counts_per_milligauss[8]={  
    1370,
    1090,
    820,
    660,
    440,
    390,
    330,
    230
  };
#endif

/* PUBLIC METHODS */

HMC58X3::HMC58X3() { 
  
  x_scale=1.0F;
  y_scale=1.0F;
  z_scale=1.0F;
}


int HMC58X3::init(bool setmode,byte address) {
  // note that we don't initialize Wire here. 
  // You'll have to do that in setup() in your Arduino program
  ms_delay(5); // you need to wait at least 5ms after power on to initialize
  _dev_address=address;
  int a=0;
  if (setmode) {
    a=setMode(0);
    if(a<0){
            usart_printfm(USARTx,(const int *)"Failed to set mode");
            return -1;
        }
  }
  
  a=writeTo( _dev_address,HMC58X3_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
  if(a<0){
            usart_printfm(USARTx,(const int *)"Failed to conf confa");
            return -1;
        }
  a=writeTo( _dev_address,HMC58X3_R_CONFB, 0xA0);
  if(a<0){
            usart_printfm(USARTx,(const int *)"Failed to conf confb");
            return -1;
        }
  a=writeTo( _dev_address,HMC58X3_R_MODE, 0x00);
  if(a<0){
            usart_printfm(USARTx,(const int *)"Failed to conf rmode ");
            return -1;
        }
  return 0;
}


int HMC58X3::setMode(unsigned char mode) {
  int a=0;
  if (mode > 2) {
    return 0;
  }
  
  a=writeTo( _dev_address,HMC58X3_R_MODE, mode);
  if(a<0){
            usart_printfm(USARTx,(const int *)"Failed to set mode");
            return -1;
        }
  ms_delay(100);
  return 0;
}

/*
    Calibrate which has a few weaknesses.
    1. Uses wrong gain for first reading.
    2. Uses max instead of max of average when normalizing the axis to one another.
    3. Doesn't use neg bias. (possible improvement in measurement).
*/
void HMC58X3::calibrate(unsigned char gain) {
  int a=0;
  x_scale=1; // get actual values
  y_scale=1;
  z_scale=1;
  a=writeTo( _dev_address,HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
  setGain(gain);
  float x, y, z, mx=0, my=0, mz=0, t=10;
  
  for (int i=0; i<(int)t; i++) { 
    setMode(1);
    getValues(&x,&y,&z);
    if (x>mx) mx=x;
    if (y>my) my=y;
    if (z>mz) mz=z;
  }
  
  float max=0;
  if (mx>max) max=mx;
  if (my>max) max=my;
  if (mz>max) max=mz;
  x_max=mx;
  y_max=my;
  z_max=mz;
  x_scale=max/mx; // calc scales
  y_scale=max/my;
  z_scale=max/mz;

  a=writeTo( _dev_address,HMC58X3_R_CONFA, 0x010); // set RegA/DOR back to default
}   // calibrate().

/*!
    \brief Calibrate using the self test operation.
  
    Average the values using bias mode to obtain the scale factors.

    \param gain [in] Gain setting for the sensor. See data sheet.
    \param n_samples [in] Number of samples to average together while applying the positive and negative bias.
    \return Returns false if any of the following occurs:
        # Invalid input parameters. (gain>7 or n_samples=0).
        # Id registers are wrong for the compiled device. Unfortunately, we can't distinguish between HMC5843 and HMC5883L.
        # Calibration saturates during the positive or negative bias on any of the readings.
        # Readings are outside of the expected range for bias current. 
*/
int HMC58X3::calibrate(unsigned char gain,unsigned int n_samples) 
{
    int a=0;
    int xyz[3];                     // 16 bit integer values for each axis.
    long int xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
    bool bret=true;                 // Function return value.  Will return false if the wrong identifier is returned, saturation is detected or response is out of range to self test bias.
    byte id[3];                     // Three identification registers should return 'H43'.
    long int low_limit, high_limit;                                    
    /*
        Make sure we are talking to the correct device.
        Hard to believe Honeywell didn't change the identifier.
    */
    if ((8>gain) && (0<n_samples)) // Notice this allows gain setting of 7 which the data sheet warns against.
    {
        a=getID(id);
        if(a<0){
            usart_printfm(USARTx,(const int *)"Failed to get id");
            return -1;
        }
        if (('H' == id[0]) && ('4' == id[1]) && ('3' == id[2]))
        {   /*
                Use the positive bias current to impose a known field on each axis.
                This field depends on the device and the axis.
            */
            a=writeTo( _dev_address,HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
              if(a<0){
                    usart_printfm(USARTx,(const int *)"Failed to pos bias");
                    return -1;
                }
            /*
                Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
                The new gain setting is effective from the second measurement and on.
            */
            a=setGain(gain);
              if(a<0){
                    usart_printfm(USARTx,(const int *)"Failed to set gain");
                    return -1;
                }
            a=setMode(1);                               // Change to single measurement mode.
              if(a<0){
                    usart_printfm(USARTx,(const int *)"Failed to set mode");
                    return -1;
                }                         
            a=getRaw(&xyz[0],&xyz[1],&xyz[2]);    // Get the raw values and ignore since this reading may use previous gain.
                if(a<0){
                    usart_printfm(USARTx,(const int *)"Failed to get values");
                    return -1;
                }  
            for (unsigned int i=0; i<n_samples; i++) 
            { 
                setMode(1);
                getRaw(&xyz[0],&xyz[1],&xyz[2]);   // Get the raw values in case the scales have already been changed.
                /*
                    Since the measurements are noisy, they should be averaged rather than taking the max.
                */
                xyz_total[0]+=xyz[0];
                xyz_total[1]+=xyz[1];
                xyz_total[2]+=xyz[2];
                /*
                    Detect saturation.
                */
                if (-(1<<12) >= min(xyz[0],min(xyz[1],xyz[2])))
                {
                    usart_printfm(USARTx,(const int *)"HMC58x3 Self test saturated. Increase range.");
                    bret=false;
                    break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
                }
            }
            /*
                Apply the negative bias. (Same gain)
            */
            a=writeTo( _dev_address,HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
            for (unsigned int i=0; i<n_samples; i++) 
            { 
                setMode(1);
                getRaw(&xyz[0],&xyz[1],&xyz[2]);   // Get the raw values in case the scales have already been changed.
                /*
                    Since the measurements are noisy, they should be averaged.
                */
                xyz_total[0]-=xyz[0];
                xyz_total[1]-=xyz[1];
                xyz_total[2]-=xyz[2];
                /*
                    Detect saturation.
                */
                if (-(1<<12) >= min(xyz[0],min(xyz[1],xyz[2])))
                {
                    usart_printfm(USARTx,(const int *)"HMC58x3 Self test saturated. Increase range.");
                    bret=false;
                    break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
                }
            }
            /*
                Compare the values against the expected self test bias gauss.
                Notice, the same limits are applied to all axis.
            */
            low_limit =SELF_TEST_LOW_LIMIT *counts_per_milligauss[gain]*2*n_samples;
            high_limit=SELF_TEST_HIGH_LIMIT*counts_per_milligauss[gain]*2*n_samples;

            if ((true==bret) && 
                (low_limit <= xyz_total[0]) && (high_limit >= xyz_total[0]) &&
                (low_limit <= xyz_total[1]) && (high_limit >= xyz_total[1]) &&
                (low_limit <= xyz_total[2]) && (high_limit >= xyz_total[2]) )
            {   /*
                    Successful calibration.
                    Normalize the scale factors so all axis return the same range of values for the bias field.
                    Factor of 2 is from summation of total of n_samples from both positive and negative bias.
                */
                x_scale=(counts_per_milligauss[gain]*(HMC58X3_X_SELF_TEST_GAUSS*2))/(xyz_total[0]/n_samples);
                y_scale=(counts_per_milligauss[gain]*(HMC58X3_Y_SELF_TEST_GAUSS*2))/(xyz_total[1]/n_samples);
                z_scale=(counts_per_milligauss[gain]*(HMC58X3_Z_SELF_TEST_GAUSS*2))/(xyz_total[2]/n_samples);
            }else
            {
               usart_printfm(USARTx,(const int *)"HMC58x3 Self test out of range.");
                bret=false;
            }
            a=writeTo( _dev_address,HMC58X3_R_CONFA, 0x010); // set RegA/DOR back to default.
        }else
        {
            #if defined(ISHMC5843)
                usart_printfm(USARTx,(const int *)"HMC5843 failed id check.");
            #else
                usart_printfm(USARTx,(const int *)"HMC5883L failed id check.");
            #endif
            bret=false;
        }
    }else
    {   /*
            Bad input parameters.
        */
        usart_printfm(USARTx,(const int *)"HMC58x3 Bad parameters.");
        bret=false;
    }
    return(bret);
}   //  calibrate().

// set data output rate
// 0-6, 4 default, normal operation assumed
int HMC58X3::setDOR(unsigned char DOR) {
  int a=0;
  if (DOR>6) return 0;
  a=writeTo( _dev_address,HMC58X3_R_CONFA,DOR<<2);
  if(a<0){
      usart_printfm(USARTx,(const int *)"Failed to set data rate");
      return -1;
  }
  return 0;
}


int HMC58X3::setGain(unsigned char gain) { 
  // 0-7, 1 default
  int a=0;
  if (gain > 7) return 0;
  a=writeTo( _dev_address,HMC58X3_R_CONFB, gain << 5);
  if(a<0){
      usart_printfm(USARTx,(const int *)"Failed to setgain");
      return -1;
  }
  return 0;
}


//int HMC58X3::a=writeTo( _dev_address,byte _dev_address,unsigned char reg, unsigned char val) {
//  a=writeTo( _dev_address,byte address, byte val);
//  Wire.beginTransmission(_dev_address);
//  Wire.write(reg);        // send register address
//  Wire.write(val);        // send value to write
//  Wire.endTransmission(); //end transmission
//}


void HMC58X3::getValues(int *x,int *y,int *z) {
  float fx,fy,fz;
  getValues(&fx,&fy,&fz);
  *x= (int) (fx + 0.5);
  *y= (int) (fy + 0.5);
  *z= (int) (fz + 0.5);
}


void HMC58X3::getValues(float *x,float *y,float *z) {
  int xr,yr,zr;
  
  getRaw(&xr, &yr, &zr);
  *x= ((float) xr) / x_scale;
  *y = ((float) yr) / y_scale;
  *z = ((float) zr) / z_scale;
}


int HMC58X3::getRaw(int *x,int *y,int *z) {
  int a=0;
  uint8_t _buff[6];
  a=readFrom(HMC58X3_ADDR,HMC58X3_R_XM, 2, &_buff[0]);
  //a=readFrom(HMC58X3_ADDR,HMC58X3_R_XM, 1, &_buff[1]);
  if(a<0){
      usart_printfm(USARTx,(const int *)"Failed to read from x");
      return -1;
  }
  a=readFrom(HMC58X3_ADDR,HMC58X3_R_YM, 2, &_buff[4]);   // the Z registers comes before the Y registers in the HMC5883L
  if(a<0){
      usart_printfm(USARTx,(const int *)"Failed to read from z");
      return -1;
  }
  a=readFrom(HMC58X3_ADDR,HMC58X3_R_ZM, 2, &_buff[2]);
  if(a<0){
      usart_printfm(USARTx,(const int *)"Failed to read from y");
      return -1;
  }
  *x=(_buff[0]<<8)| _buff[1];
  *y=(_buff[2]<<8)| _buff[3];
  *z=(_buff[4]<<8)| _buff[5];
  return 0;
//  Wire.beginTransmission(HMC58X3_ADDR);
//  Wire.requestFrom(HMC58X3_ADDR, 6);
//  if(6 == Wire.available()) {
//    // read out the 3 values, 2 bytes each.
//    *x = (Wire.read() << 8) | Wire.read();
//    #ifdef ISHMC5843
//      *y = (Wire.read() << 8) | Wire.read();
//      *z = (Wire.read() << 8) | Wire.read();
//    #else // the Z registers comes before the Y registers in the HMC5883L
//      *z = (Wire.read() << 8) | Wire.read();
//      *y = (Wire.read() << 8) | Wire.read();
//    #endif
//    // the HMC58X3 will automatically wrap around on the next request
//  }
//  Wire.endTransmission();
}

int HMC58X3::getValues(int *xyz){
  int a=0;
  a=getRaw(&xyz[0],&xyz[1],&xyz[2]);
  if(a<0){
      usart_printfm(USARTx,(const int *)"Failed to getValues");
      return -1;
  }
  return 0;
}
void HMC58X3::getValues(float *xyz) {
  getValues(&xyz[0], &xyz[1], &xyz[2]);
}

/*! 
    \brief Retrieve the value of the three ID registers.    

    Note:  Both the HMC5843 and HMC5883L have the same 'H43' identification register values. (Looks like someone at Honeywell screwed up.)

    \param id [out] Returns the three id register values.
*/
int HMC58X3::getID(byte id[3]) 
{
  int a=0;
  a=readFrom(HMC58X3_ADDR,HMC58X3_R_IDA, 2, &id[0]);
  if(a<0){
      usart_printfm(USARTx,(const int *)"Failed to read id1");
      return -1;
  }
  a=readFrom(HMC58X3_ADDR,HMC58X3_R_IDC, 1, &id[2]);
  if(a<0){
      usart_printfm(USARTx,(const int *)"Failed to read id2");
      return -1;
  }
  return 0;
//  Wire.beginTransmission(HMC58X3_ADDR);
//  Wire.write(HMC58X3_R_IDA);             // Will start reading registers starting from Identification Register A.
//  Wire.endTransmission();
//  
//  Wire.beginTransmission(HMC58X3_ADDR);
//  Wire.requestFrom(HMC58X3_ADDR, 3);
//  if(3 == Wire.available()) 
//  {
//    id[0] = Wire.read();
//    id[1] = Wire.read();
//    id[2] = Wire.read();
//  }else
//  {
//      id[0]=0;  
//      id[1]=0;
//      id[2]=0;
//  }
//  Wire.endTransmission();
}   // getID().

void *magnetometer_initialisation(byte _dev_address){
  int a=0;
  HMC58X3 *magnet=new HMC58X3();
  a=((HMC58X3*)magnet)->init(True,_dev_address);
  if(a<0){
    usart_printfm(USARTx,(const int *)"Failed at magnetometer initialization\n\r");
    return null;
  }else
    return (void*)(magnet);
  }

int magnet_xyz(void *magnet,int *xyz){
int a=0;
a=((HMC58X3*)magnet)->getValues(xyz);
if(a<0){
    usart_printfm(USARTx,(const int *)"Failed at getting values from sensor\n\r");
    return -1;
  }
return 0;
}