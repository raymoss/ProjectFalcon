/*
* Getting Started example sketch for nRF24L01+ radios
* This is a very basic example of how to send data from one node to another
* Updated: Dec 2014 by TMRh20
*/
#include <SPI.h>
#include "RF24.h"
#define RUN                             0
#define INITIALISE                      31
#define RUNTIME                         41
#define YAW_ANGLE                       21
#define ROLL_ANGLE                      22
#define PITCH_ANGLE                     23
#define KP_VALUE                        11
#define KI_VALUE                        12
#define KD_VALUE                        13
#define MOTOR1_ERROR_VALUE              1
#define MOTOR2_ERROR_VALUE              2
#define MOTOR3_ERROR_VALUE              3
#define MOTOR4_ERROR_VALUE              4
#define MOTOR_SPEED1                    5
#define MOTOR_SPEED2                    6
#define MOTOR_SPEED3                    7
#define MOTOR_SPEED4                    8
#define THROTTLE                        9
#define PING                             10
String inString = "";
typedef struct comms_data{
long code;
long value;
}comms_data;
int tag=0;
/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 1;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(9,10);
int get=9;
int give=9;
int timeout=500;
long int last;

/**********************************************************/
uint8_t addresses[][6] = {"1Node","2Node"};
// Used to control whether this node is sending or receiving
bool role = 1;
void setup() {
  Serial.begin(9600);
  Serial.println(F("RF24/examples/GettingStarted"));
  //Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
  
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
//   Serial.println(radio.getPayloadSize());
//   Serial.println(sizeof(comms_data));
//    Serial.println(sizeof(int));
  // Start the radio listening for data
  radio.startListening();
}
int stringToInt(){
inString = "";
int inChar='0';
  //Serial.println("I ma here");
  while(true){
  while (Serial.available() > 0 ) {
    inChar = Serial.read();
    Serial.println(inChar);
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
     Serial.println(inChar);
      inString += (char)inChar;
    }
    // if you get a newline, print the string,
    // then the string's value:
    if (inChar == '!') {
      Serial.print("Value:");
     // Serial.println(inString.toInt());
     // Serial.print("String: ");
     // Serial.println(inString);
      // clear the string for new input:
      return inString.toInt();
      
    }
  }
}}
void motorControl(){
  radio.stopListening();
  comms_data c;
  //bool timeout_flag=false;
  timeout=500;
  Serial.println("Enter the code:");
  while(!Serial.available());
  c.code=stringToInt();
  Serial.println("Enter the value");
  while(!Serial.available());
  c.value=stringToInt();
  Serial.println(c.code);
  Serial.println(c.value);
 
  if (!radio.write( &c, sizeof(comms_data) )){
       Serial.println(F("write failed"));
         }else{
   Serial.println("Write was successfull");    
         }
  
  }
void recieveMotorSpeed(){
  radio.stopListening();
  int timeout=500;
  bool timeout_flag=false;
  comms_data c;
  Serial.println("Enter the code:");
  while(!Serial.available());
  c.code=stringToInt();
  
  c.value=0;
 // Serial.println(c.code);
 // Serial.println(c.value);
  if (!radio.write( &c, sizeof(comms_data) )){
       Serial.println(F("write failed"));
         }else{
  // Serial.println("Write was successfull");    
         }
   radio.startListening();
   while ( ! radio.available() ){                             // While nothing is received
      if ((timeout--)==0 ){            // If waited longer than 200ms, indicate timeout and exit while loop
          timeout_flag = true;
          break;
      }      
    delay(10);  
  }
    if ( timeout_flag ){                                             // Describe the results
        Serial.println(F("Failed, response timed out."));
    }else{
     // Serial.println("Got something...");  
      radio.read( &c, sizeof(comms_data));
        if(MOTOR_SPEED1<=c.code<=MOTOR_SPEED4){
          Serial.print("MotorSpeed:");
          Serial.println(c.value);
        }
        else
        Serial.println("Failed to get the value");
        
    }
}

void motorPID(){
  radio.stopListening();
  comms_data c;
  Serial.println("Enter the error code:");
  while(!Serial.available());
  c.code=int(Serial.read());
  Serial.println("Enter the value");
  while(!Serial.available());
  c.value=int(Serial.read());
  Serial.println(c.code);
  Serial.println(c.value);
  if (!radio.write( &c, sizeof(comms_data) )){
       Serial.println(F("write failed"));
         }else{
   Serial.println("Write was successfull");    
         }
}
void recieveRollAngle(){
  radio.stopListening();
  int timeout=500;
  bool timeout_flag=false;
  comms_data c;
  c.code=ROLL_ANGLE;
  c.value=0;
 // Serial.println(c.code);
 // Serial.println(c.value);
  if (!radio.write( &c, sizeof(comms_data) )){
       Serial.println(F("write failed"));
         }else{
  // Serial.println("Write was successfull");    
         }
   radio.startListening();
   while ( ! radio.available() ){                             // While nothing is received
      if ((timeout--)==0 ){            // If waited longer than 200ms, indicate timeout and exit while loop
          timeout_flag = true;
          break;
      }      
    delay(10);  
  }
    if ( timeout_flag ){                                             // Describe the results
        Serial.println(F("Failed, response timed out."));
    }else{
     // Serial.println("Got something...");  
      radio.read( &c, sizeof(comms_data));
        if(c.code==ROLL_ANGLE){
          Serial.print("Roll_angle:");
          Serial.println(c.value);
        }
        else
        Serial.println("Failed to get the value");
        
    }
}
void recievePitchAngle(){
  radio.stopListening();
  int timeout=500;
  comms_data c;
  bool timeout_flag=false;
  c.code=PITCH_ANGLE;
  c.value=0;
  if (!radio.write( &c, sizeof(comms_data) )){
       Serial.println(F("write failed"));
         }else{
  // Serial.println("Write was successfull");    
         }
   radio.startListening();
   while ( ! radio.available() ){                             // While nothing is received
      if ((timeout--)==0 ){            // If waited longer than 200ms, indicate timeout and exit while loop
          timeout_flag = true;
          break;
      }      
    delay(10);  
  }
    if ( timeout_flag ){                                             // Describe the results
        Serial.println(F("Failed, response timed out."));
        
    }else{
    //  Serial.println("Got something...");  
      radio.read( &c, sizeof(comms_data));
        if(c.code==PITCH_ANGLE){
          Serial.print("Pitch_angle:");
          Serial.println(c.value);
        }
        else   
        Serial.println("Failed to get the value");
        
    }
}
void pingReport(){
  radio.stopListening();
  int timeout=500;
  bool timeout_flag=false;
  comms_data c;
  c.code=PING;
  c.value=0;
 // Serial.println(c.code);
 // Serial.println(c.value);
  if (!radio.write( &c, sizeof(comms_data) )){
       Serial.println(F("write failed"));
         }else{
  // Serial.println("Write was successfull");    
         }
   radio.startListening();
   while ( ! radio.available() ){                             // While nothing is received
      if ((timeout--)==0 ){            // If waited longer than 200ms, indicate timeout and exit while loop
          timeout_flag = true;
          break;
      }      
    delay(10);  
  }
    if ( timeout_flag ){                                             // Describe the results
        Serial.println(F("Failed, response timed out."));
    }else{
     // Serial.println("Got something...");  
      radio.read( &c, sizeof(comms_data));
        if(c.code==PING){
          Serial.println("Connected");
          }
        else
        Serial.println("Failed to get the value");
        
    }
}
void loop() {
  Serial.println("Enter yr choice:");
  Serial.println("1.motorControl");
  Serial.println("2.motorPID");
  Serial.println("3.recieveRoll");
  Serial.println("4.recievePitch");
  //while(!Serial.available());
  char c=Serial.read();
  switch(c){
  case '1':
  motorControl();
  tag=1;
  break;
  case '2':
  motorPID();
  break;
  case '3':
  recieveRollAngle();
  break;
  case '4':
  recievePitchAngle();
  break;
  case '5':
  recieveMotorSpeed();
  break;
  default:
  recieveRollAngle();
  delay(150);
  recievePitchAngle();
  delay(150);
  //pingReport();
  }                                                                                                                                                                                                                                                                                                
  delay(150);
  
} // Loop

//if(tag==1){
//  while(true){
//    last=millis();
//  recieveMotorSpeed(MOTOR_SPEED1);
//  //recieveRollAngle();
//  delay(150);
//  //recievePitchAngle();
//delay(150);  
//Serial.println(millis()-last);
//}
//  }
