Current status of the project:
16/6/2106 : 2.34PM
1.We have a working IMU sensor which gives output in the form of integer.
2.PWM waves for motor control is also working fine. Verified with the logic analyser.
3.Input from the usart is working . Using this we can send run time commands to the controller.
4.Basic functionality of communication channel using nrf24l01 is working . We are using the stm32 as the server and arduino board as the client.
5. Get the angle of pitch and roll from the accelerometer and gyro.
6. Passing the angles from a filter (kalman or complementary ?) .
7. Design a PID controller for the motor control.
8. Prepare the basic protocol to debug the readings and control from communication channel. 
9. Optimize PID parameters for smooth movement of the motors. PENDING
10. Lot of variation in roll angle is observed when motor is started. PENDING 

BUGS(Resolved):
17/4/2016 : 2.55AM
1. #define I2Cx I2C2 
Resolved it by explicitely specifying the name. Need to find a workaround for that so that we can supply the parameter at run time and we get flexible code.
Successfully removed over 100 errors from the code. It is showing error in specifying i2c address. Need to correct that in the morning . Tommorrow we target to have a 
fully funtional c code for reading the accelerometer data and displaying it on the usart.
2. There is a issue when calling a c++ class from c. When the object is initialized , we copied the pointer value to void pointer and then use the void pointer to call other functions.
With the above mentioned method, we are able to call the functions but class data members are returning weird results. As a WAR, we are directly returning the results to main and then 
calling the functions using that pointer.
3. There is an issue with systemInit values which causes the spi clocks to behave abnormally. Resolved it by copying it from other project.
Lets hope for the best.


BUGS:
1. Unable to read 6 bytes of i2c data from the sensors. Currently reading 2 bytes at a time.
2. After completion of write cycle, we are not sending the stop cycle. Currently it is not causing any issues but in future it can haunt us.
3. No variation in roll angle should be observed when motors are rotating in stationary mode.
