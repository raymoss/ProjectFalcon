Current status of the project:
17/4/2016 : 2.55AM

Today we came across a stupid bug which caused a delay of almost 4 hrs. It was that we were just replacing the I2C object with a name I2C2 .
#define I2Cx I2C2 
Resolved it by explicitely specifying the name. Need to find a workaround for that so that we can supply the parameter at run time and we get flexible code.
Successfully removed over 100 errors from the code. It is showing error in specifying i2c address. Need to correct that in the morning . Tommorrow we target to have a 
fully funtional c code for reading the accelerometer data and displaying it on the usart.

Lets hope for the best.

