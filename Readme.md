#PROJECT FALCON

**Quadcopter based on STM32F4discovery board**

##INTRODUCTION:
This projects objective is to build a quadcopter using STM32F4discovery board , nrf24L01 module( as a communication channel ), GY-85 9 axis IMU module (consists of ADXL345 accelerometer , ITG3205 gyrometer and HMC 5883 magnetometer) , ESC's and Motors from scratch( Yeah! ). For the components such as frame,ESC's and motors , we are using [ARF 525](http://www.quadkopters.com/product/arf-kit-rtf-kit-and-frames/arf-525-kit-with-kk-2-1-controller/). In place of KK2.1 board, we are using custom controller based on STM32F4Discovery. For sensor drivers , we are using the code of arduino as reference and then modifying them for arm based stm32 controller. We used this porting of drivers for IMU sensors and nrf24l01 module. We are combining the sensor value using kalman filter so that there inherent shortcommings (gyro's drift and accelerometer variation due to motion) can be overcomed . Fortunately we have an excellent library ([link](https://github.com/TKJElectronics/KalmanFilter)) for implementing the filter. The motor control is performed using PID algorithm using the roll and pitch angle provided by the filter. On hardware side, we are using timers to generate PWM signals for the ESC. Upper and lower limit is set at 700 and 2400 ms with 50Hz signal. We are using arduino board with another nrf24l01 module to control the quadcopter. We can change the PID parameters, motor drift value , control motor speed and observe roll and pitch angle using this communication channel.  

##Current status of the project:
1. We have a working IMU sensor which gives output in the form of integer.
2. PWM waves for motor control is also working fine. Verified with the logic analyser.
3. Input from the usart is working . Using this we can send run time commands to the controller.
4. Basic functionality of communication channel using nrf24l01 is working . We are using the stm32 as the server and arduino board as the client.
5. Get the angle of pitch and roll from the accelerometer and gyro.
6. Passing the angles from a filter (kalman or complementary ?) .
7. Design a PID controller for the motor control.
8. Prepare the basic protocol to debug the readings and control from communication channel. 
9. Optimize PID parameters for smooth movement of the motors. PENDING
10. Lot of variation in roll angle is observed when motor is started. PENDING 

##BUGS(Resolved):

1. There is an issue with systemInit values which causes the spi clocks to behave abnormally and hence generating abnormal SPI clocks.
Resolved it by correcting APB and AHB bus initialization code.
2. Sensor read from gyro was not working as master generates an acknowledge at the end of the I2C cycle. There was issue with sequence of operation in the I2C driver code. Modified it according to datasheet and voila!!
3. Intermittently, sensor read from gyroscope fails due to status register value 0x01. As repro rate is low , resolved it by providing some time delay on consecutive read.
4. #define I2Cx I2C2 
Resolved it by explicitely specifying the name. Need to find a workaround for that so that we can supply the parameter at run time and we get flexible code.
5. There is a issue when calling a c++ class from c. When the object is initialized , we copied the pointer value to void pointer and then use the void pointer to call other functions.
With the above mentioned method, we are able to call the functions but class data members are returning weird results. As a WAR, we are directly returning the results to main and then 
calling the functions using that pointer.

##BUGS(Pending):
1. Unable to read 6 bytes of i2c data from the sensors. Currently reading 2 bytes at a time.
2. After completion of write cycle, we are not sending the stop cycle. Currently it is not causing any issues but in future it can haunt us.
3. No variation in roll angle should be observed when motors are rotating in stationary mode.
4. ESC are getting exploded , need to check if it can be prevented.
