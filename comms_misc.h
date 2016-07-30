#include "RF24.h"
#include "usart_print.h"
#include "motor_control.h"
#include "filter.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

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
#define MOTOR1_SPEED                    5
#define MOTOR2_SPEED                    6
#define MOTOR3_SPEED                    7
#define MOTOR4_SPEED                    8
#define THROTTLE                        9
#define PING                          10
extern void* radio;
extern uint8_t run_motor;
extern float final_roll, final_pitch,final_yaw;
#ifdef __cplusplus
extern "C"{
#endif
  uint8_t pingReport();
  uint8_t comms_initialisation();
  uint8_t motorControl(comms_data c);
  uint8_t motorPID(comms_data c);
  uint8_t sendYawAngle();
  uint8_t sendRollAngle();
  uint8_t sendPitchAngle();
#ifdef __cplusplus
}
#endif
