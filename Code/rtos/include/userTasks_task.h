/**
  * ============================================================================
  * File Name          : userTasks_task.h
  * Description        : Common task source header file
  * Author             : Sean Wood
  * ============================================================================
  */

#ifndef USERTASKS_TASK_H
#define USERTASKS_TASK_H

// == Includes ==
#include "cmsis_os.h"
#include "task.h"
#include "hal_lib.h"
#include "genericMessaging_lib.h"
#include "string.h"
#include "strings_res.h"
#include "stdio.h"

// == Definitions ==
#define AP_SSID                     "coreNet"
#define AP_KEY                      "electronics9663"
#define LED_BLINK_SLOW_PERIOD       1000 // Value to load the one-shot timer for SLOW blink [ms]
#define LED_BLINK_FAST_PERIOD       500 // Value to load the one-shot timer for FAST blink [ms]
#define LED_BLINK_SUPERFAST_PERIOD  100 // Value to load the one-shot timer for SUPERFAST blink [ms]

// == Type Declarations - Procedures ==
// Procedure statuses
typedef enum {
  PROC_STATUS_OK,
  PROC_STATUS_BUSY,
  PROC_STATUS_COMPLETED,
  PROC_STATUS_ERROR
} procStatus_t;

// Wifi communication procedures (flags)
typedef enum {
  WIFI_PROC_NONE,
  WIFI_PROC_AT_TEST,
  WIFI_PROC_INIT,
  WIFI_PROC_CONNECT_AP,
  WIFI_PROC_START_SERVER
} wifiProcedures_t;

// == Type Declarations - States ==

// Peripheral states - to be used for locking out peripherals during procedures etc.
typedef enum {
  GEN_STATE_READY,
  GEN_STATE_BUSY,
  GEN_STATE_ERROR
} genericStates_t;

// Method of communication with the wifi module. AUTO allows tasks to handle
// responses from the module. MANUAL routes all communication via USB<-->WIFI
typedef enum {
  COMM_STATE_AUTO,
  COMM_STATE_MANUAL
} commState_t;

// State of the indicator LED
typedef enum {
  LED_STATE_OFF,
  LED_STATE_ON,
  LED_STATE_BLINK_SLOW,
  LED_STATE_BLINK_FAST,
  LED_STATE_BLINK_SUPERFAST
} ledState_t;

// States of the motor controllers (PWMs)
typedef enum {
  MTR_STATE_OFF,
  MTR_STATE_STANDBY,
  MTR_STATE_RUNNING
} motorState_t;

// H-Bridge states
typedef enum {
  HB_STATE_DISABLED,
  HB_STATE_ENABLED
} hBridgeState_t;

// State of the line sensor algorithm
typedef enum {
  LNS_STATE_OFF,
  LNS_STATE_ON
} lineSensorState_t;

// State of the color light sensor
typedef enum {
  LIGHT_STATE_OFF,
  LIGHT_STATE_ON
} lightSensorState_t;

// == Type Declarations - Other Motor ==

// Directions of the motors
typedef enum {
  MTR_DIR_DISABLED,
  MTR_DIR_FWD,
  MTR_DIR_REV
} motorDir_t;

// Signals that can be sent to the motor task
typedef enum {
  MTR_SIG_START_TRACKING,
  MTR_SIG_STOP_TRACKING
} motorSignals_t;

// Global motor control data
// NOTE: Only the main function and the motorTask should set these values.
//       No other task is to write to these values!
typedef struct {
  motorDir_t leftMotorDir;
  motorDir_t rightMotorDir;
  int8_t leftMotorSpeed;
  int8_t rightMotorSpeed;
  hBridgeState_t hBridgeState;
} motorData_struct;

// == Type Declarations - Other Line Sensor ==

// Positions of the line being sensed
typedef enum {
  LINE_POS_LEFTLEFT,
  LINE_POS_LEFT,
  LINE_POS_CENTER,
  LINE_POS_RIGHT,
  LINE_POS_RIGHTRIGHT
} linePos_t;

// Signals to send to the Line Sensor Task (used for line and light sensors)
typedef enum {
  LINE_SIG_STOP,
  LINE_SIG_START,
  LIGHT_SIG_START,
  LIGHT_SIG_STOP
} lineSensorSignals_t;

// Global line sensor data
typedef struct {
  linePos_t linePos;
} lineSensorData_struct;

// == Type Declarations - Global Structs ==

typedef struct {
  size_t bossTaskStackHWM;
  size_t USARTInTaskStackHWM;
  size_t USARTOutTaskStackHWM;
  size_t motorTaskStackHWM;
  size_t lineSensorTaskStackHWM;
  size_t userIOTaskStackHWM;
} generalData_t;

// Proceedure flags
typedef struct {
  wifiProcedures_t wifiProcedures;
} globalProcedures_t;

// Custom peripheral states
typedef struct {
  commState_t commState;
  genericStates_t wifiState; // Task level peripheral state flag (for task level locking)
  motorState_t motorState;
  lineSensorState_t lineSensorState;
  lightSensorState_t lightSensorState;
  ledState_t ledState;
} globalStates_t;

// Global program flags
typedef struct {
  globalStates_t states;
  globalProcedures_t procedures;
  motorData_struct motorData;
  lineSensorData_struct lineSensorData;
  generalData_t generalData;
} globalFlags_t;

// == Exported Variables ==
extern osThreadId bossTaskHandle;
extern osThreadId USARTInTaskHandle;
extern osThreadId USARTOutTaskHandle;
extern osThreadId motorTaskHandle;
extern osThreadId lineSensorTaskHandle;
extern osThreadId userIOTaskHandle;

extern osTimerId ledTimerHandle;
extern osTimerId buzzerTimerHandle;

extern globalFlags_t globalFlags;

// USART In Task String Queue
extern osMessageQId msgQUSARTIn;

// USART Out Task String Queue
extern osMessageQId msgQUSARTOut;

// Boss Task Command Queue
extern osMessageQId msgQBoss;

// UserIO Task Command Queue
extern osMessageQId msgQUserIO;


// == Function Prototypes ==
void StartBossTask(void const * argument);
void StartUSARTInTask(void const * argument);
void StartUSARTInBufferTask(void const * argument);
void StartUSARTOutTask(void const * argument);
void StartMotorTask(void const * argument);
void StartLineSensorTask(void const * argument);
void StartUserIOTask(void const * argument);

void ledTimerCallback(void const * argument);
void buzzerTimerCallback(void const * argument);

#endif /*USERTASKS_TASK_H*/
