/**
 * ============================================================================
 * File Name          : motorTask_task.c
 * Description        : motorTask Body
 * Author             : Sean Wood
 * ============================================================================
 */

// == Includes ==
#include "userTasks_task.h"

// == Definitions ==
#define FALSE 0
#define TRUE 1

#define MTR_SPEED_REACTION_PERIOD     1 // Period of changing PWM motor speed [+-ms]
#define MTR_UPDATE_PERIOD             20 // Interval between updating the motor control [+-ms]
#define LEFT_TRIM                     ((float)0) // Percentage trim of the left motor
#define RIGHT_TRIM                    ((float)0) // Percentage trim of the right motor

// For BANG BANG SMOOTH control
#define REDUCTION_STEP                10 // Percentage reduction to take off the speed of the motors for LEFT and RIGHT positions

// For SMOOTH control TODO Actually need to set SMOOTH control parameters
#define TRACKING_GAIN                 ((float)2) // Amount to decrease the motor speed by to achieve tracking
#define AUTO_SPEED                    ((float)100) // Percentage PWM to apply to motors when in auto drive mode
#define MIN_TRACKING_SPEED            ((float)0) // Minimum speed a motor can reach during tracking
#define SETTLE_SPEED                  ((float)0.25) // Amount to decrease the cumulative error when line is in center
#define MAX_ERROR_COUNT               100000 // Maximum accumulation

// == Private Function Declarations ==
static void interpretSignal(osEvent *signalEvent);
static void controlMotors(void);
static void stopMotors(void);
static void disableMotors(void);
static void enableMotors(void);
static void setMotors(float leftSpeed, float rightSpeed);

static void disableLauncher(void);
static void enableLauncher(void);
static void launch(void);
static void terminateLaunch(void);

static void setLF(float speed);
static void setLR(float speed);
static void setRF(float speed);
static void setRR(float speed);
static void setLaunchF(float speed);
static void setLaunchR(float speed);

// == Function Definitions ==

/**
 * @brief motorTask
 * @param argument
 */
void StartMotorTask(void const * argument) {
  globalFlags.motorData.leftMotorDir = MTR_DIR_DISABLED;
  globalFlags.motorData.leftMotorDir = MTR_DIR_DISABLED;
  globalFlags.motorData.leftMotorSpeed = 0;
  globalFlags.motorData.rightMotorSpeed = 0;

  static uint8_t step = 0;

  disableMotors();

  /* Infinite loop */
  for (;;) {
    globalFlags.generalData.motorTaskStackHWM = uxTaskGetStackHighWaterMark(motorTaskHandle);

    // Wait for the signal forever, unless the motors are running
    osEvent signalEvent = osSignalWait(0,
        ((globalFlags.states.motorState != MTR_STATE_RUNNING) && (globalFlags.states.launcherState == LNCH_STATE_OFF)) ? (1000) : 0);

    if (signalEvent.status == osEventSignal) {
      interpretSignal(&signalEvent);
    }

    // If we are tracking the line
    if ((globalFlags.states.lineSensorState == LNS_STATE_ON)
        && globalFlags.states.motorState == MTR_STATE_RUNNING) {
      controlMotors();
    }

    // Check to see if the box line has been passed
    if (globalFlags.lineSensorData.boxPos || step != 0) {

      // Update data
      globalFlags.states.lineSensorState = LNS_STATE_OFF;
      globalFlags.states.motorState = MTR_STATE_OFF;

      osThreadSetPriority(motorTaskHandle, osPriorityAboveNormal);

      switch (step) {
      case 0:
        // First re-orient the robot
        setMotors(100, 0);
        osDelay(500);
        setMotors(0, 0);
        step = 1;
        break;
      case 1:
        // Make sure the robot is a bit more in the box
        setMotors(100, 100);
        osDelay(1250);
        step = 2;
        break;
      case 2:
        // Then continue to drive forward until the edge of the box is reached (until the sensors both trigger)
        setMotors(0, 0);
        enableLauncher();
        launch();
        step = 3;
        osThreadSetPriority(motorTaskHandle, osPriorityNormal);

//        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)) {
//          // Stop and launch after 1s
//          setMotors(0, 0);
//          osDelay(1000);
//          enableLauncher();
//          launch();
//          step = 3;
//          osThreadSetPriority(motorTaskHandle, osPriorityNormal);
//        } else {
//          setMotors(100, 100);
//        }
        break;
      default:
        break;
      }
    }

    osDelay(MTR_UPDATE_PERIOD);
  }
}

static void controlMotors(void) {
  // Grab the current line position as picked up by the sensors
  linePos_t linePositionCurrent = globalFlags.lineSensorData.linePos;
  static float errorCount = 0; // How many times have we updated the speed but have not achieved line tracking


  // Limit the accumulation
  if (errorCount < 80) {
    errorCount += 0.1;
  }

  // CONTROL SEQUENCE
  switch (linePositionCurrent) {
  case LINE_POS_LEFT: {
    // If control is BANG BANG, LEFT position should not do anything
    if (globalFlags.motorData.controlState == MTR_CTRL_BANG_BANG) {
      setMotors(AUTO_SPEED + LEFT_TRIM, AUTO_SPEED + RIGHT_TRIM);
    } else if (globalFlags.motorData.controlState == MTR_CTRL_BANG_BANG_SMOOTH) {
      // Else, if in BANG BANG SMOOTH mode, reduce the speed by a set step
      setMotors(AUTO_SPEED + LEFT_TRIM - REDUCTION_STEP,
      AUTO_SPEED + RIGHT_TRIM);
    } else if (globalFlags.motorData.controlState == MTR_CTRL_SMOOTH) {
      // Else, if in SMOOTH control, do the calcs

      // Adjust the error counter
      // If it is not zero, decrease until zero is reached
      if (errorCount > 0) {
        errorCount -= SETTLE_SPEED;
      }

      // Calculate the adjustment
      float newSpeed = globalFlags.motorData.leftMotorSpeed - (errorCount * TRACKING_GAIN);

      // Set the new speed of the motors, taking into account the minimum allowable speed
      setMotors((newSpeed < MIN_TRACKING_SPEED) ? (MIN_TRACKING_SPEED) : (newSpeed), globalFlags.motorData.rightMotorSpeed);
    }
    break;
  }
  case LINE_POS_RIGHT: {
    // If control is BANG BANG, LEFT position should not do anything
    if (globalFlags.motorData.controlState == MTR_CTRL_BANG_BANG) {
      setMotors(AUTO_SPEED + LEFT_TRIM, AUTO_SPEED + RIGHT_TRIM);
    } else if (globalFlags.motorData.controlState == MTR_CTRL_BANG_BANG_SMOOTH) {
      // Else, if in BANG BANG SMOOTH mode, reduce the speed by a set step
      setMotors(AUTO_SPEED + LEFT_TRIM,
      AUTO_SPEED + RIGHT_TRIM - REDUCTION_STEP);
    } else if (globalFlags.motorData.controlState == MTR_CTRL_SMOOTH) {
      // Else, if in SMOOTH control, do the calcs

      // Adjust the error counter
      // If it is not zero, decrease until zero is reached
      if (errorCount > 0) {
        errorCount -= SETTLE_SPEED;
      }

      // Calculate the adjustment
      float newSpeed = globalFlags.motorData.rightMotorSpeed - (errorCount * TRACKING_GAIN);

      // Set the new speed of the motors, taking into account the minimum allowable speed
      setMotors(globalFlags.motorData.leftMotorSpeed,
          (newSpeed < MIN_TRACKING_SPEED) ? (MIN_TRACKING_SPEED) : (newSpeed));
    }
    break;
  }
  case LINE_POS_CENTER:
    // If the line is tracked, set the speed of the motors to the set speed
    setMotors(AUTO_SPEED + LEFT_TRIM, AUTO_SPEED + RIGHT_TRIM);

    // Check for the SMOOTH control state
    if (globalFlags.motorData.controlState == MTR_CTRL_SMOOTH) {
      // Adjust the error counter
      // If it is not zero, decrease until zero is reached
      if (errorCount > 0) {
        errorCount -= SETTLE_SPEED;
      }
    }
    break;
  case LINE_POS_LEFTLEFT: {
    // If control is BANG BANG or BANG BANG SMOOTH, kill the left motor
    if (globalFlags.motorData.controlState == MTR_CTRL_BANG_BANG
        || globalFlags.motorData.controlState == MTR_CTRL_BANG_BANG_SMOOTH) {
      setMotors(0, AUTO_SPEED + RIGHT_TRIM);
    } else if (globalFlags.motorData.controlState == MTR_CTRL_SMOOTH) {
      // Else, if in SMOOTH control, do the calcs

      // Calculate the adjustment
      float newSpeed = globalFlags.motorData.leftMotorSpeed - (errorCount * TRACKING_GAIN * 2);

      // Set the new speed of the motors, taking into account the minimum allowable speed
      setMotors((newSpeed < MIN_TRACKING_SPEED) ? (MIN_TRACKING_SPEED) : (newSpeed), globalFlags.motorData.rightMotorSpeed);
    }
    break;
  }
  case LINE_POS_RIGHTRIGHT: {
    // If control is BANG BANG or BANG BANG SMOOTH, kill the right motor
    if (globalFlags.motorData.controlState == MTR_CTRL_BANG_BANG
        || globalFlags.motorData.controlState == MTR_CTRL_BANG_BANG_SMOOTH) {
      setMotors(AUTO_SPEED + LEFT_TRIM, 0);
    } else if (globalFlags.motorData.controlState == MTR_CTRL_SMOOTH) {
      // Else, if in SMOOTH control, do the calcs

      // Calculate the adjustment
      float newSpeed = globalFlags.motorData.rightMotorSpeed - (errorCount * TRACKING_GAIN * 2);

      // Set the new speed of the motors, taking into account the minimum allowable speed
      setMotors(globalFlags.motorData.leftMotorSpeed,
          (newSpeed < MIN_TRACKING_SPEED) ? (MIN_TRACKING_SPEED) : (newSpeed));
    }
    break;
  }
  default:
    break;
  }

}

/**
 * @brief Interpret the command received, and act on it
 * @param rxCommand: Command received in the message
 */
static void interpretSignal(osEvent *signalEvent) {
  int32_t signalEventValue = signalEvent->value.signals;
  switch (signalEventValue) {
  case MTR_SIG_START_TRACKING:
    // This must only be fired if the motors aren't actually running
    if (globalFlags.states.motorState != MTR_STATE_RUNNING) {
      // Enable the motors
      enableMotors();

      // Update the flags
      globalFlags.states.motorState = MTR_STATE_RUNNING;

      // Get the LED going!
      sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_LED_BLINK_FAST, 0);
    }

    break;
  case MTR_SIG_STANDBY:
    if (globalFlags.motorData.hBridgeState == HB_STATE_ENABLED) {
      disableMotors();
    }

    // This will reset the motors and set all PWM to zero
    enableMotors();

    // Send a signal to the line sensor task to enable sensing
    if (globalFlags.states.lineSensorState != LNS_STATE_ON) {
      osSignalSet(sensorTaskHandle, LINE_SIG_START);
      osDelay(100);
      globalFlags.states.lightSensorState = LIGHT_STATE_ON;
    }

    // Wait for the line sensor to start
    while (globalFlags.states.lineSensorState != LNS_STATE_ON) {
      osDelay(1);
    }

    // Update the flags
    globalFlags.states.motorState = MTR_STATE_STANDBY;

    // Do the LED and buzzer
    sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_LED_ON, 0);
//    sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_BUZZER_BURST_TWICE, 0);

    break;
  case MTR_SIG_STOP_TRACKING:
    // This must only be fired if the motors are actually running
    if (globalFlags.states.motorState == MTR_STATE_RUNNING) {
      // Disable the motors
      disableMotors();

      // Send a signal to the line sensor task to disable sensing
      osSignalSet(sensorTaskHandle, LINE_SIG_STOP);

      // Update states
      globalFlags.states.motorState = MTR_STATE_OFF;

      // Do the LED and buzzer
      sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_LED_ON, 0);
//      sendCommand(msgQUserIO, MSG_SRC_USER_IO_TASK, MSG_CMD_BUZZER_BURST_ONCE, 0);
    }
    break;
  case MTR_SIG_START_LAUNCHER:
    // Prep the launcher for launch
    enableLauncher();

    // Check to see if the ball is in position and launch if it is
    if (1) { // TODO Add ball sensor support here
      launch();
    } else {
      // Else signal that we are waiting for the ball
      sendCommand(msgQUserIO, MSG_SRC_MOTOR_TASK, MSG_CMD_LED_BLINK_SLOW, 0);

      // Horrid, just plain horrid
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
      osDelay(100);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
      osDelay(100);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
      osDelay(100);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
      osDelay(100);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
      osDelay(100);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

      globalFlags.states.launcherState = LNCH_STATE_WAIT_FOR_BALL;
    }

    break;
  case MTR_SIG_STOP_LAUNCHER:
    // Stop the launcher motor
    terminateLaunch();
    break;
  default:
    break;
  }
}

/**
 * @brief Stop the motors (set PWMs to zero and disable H-bridge)
 */
static void stopMotors(void) {
  // Disable the h-bridge enable pin
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  // Set all PWMs to zero
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

  // Update data
  globalFlags.motorData.leftMotorSpeed = 0;
  globalFlags.motorData.rightMotorSpeed = 0;
  globalFlags.motorData.hBridgeState = HB_STATE_DISABLED;
}

/**
 * @brief Disable the motors completely
 */
static void disableMotors(void) {
  // Disable the h-bridge enable pin
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  // Set all PWMs to zero
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

  // Stop all PWM outputs
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

  // Update data
  globalFlags.motorData.leftMotorSpeed = 0;
  globalFlags.motorData.rightMotorSpeed = 0;
  globalFlags.motorData.leftMotorDir = MTR_DIR_DISABLED;
  globalFlags.motorData.rightMotorDir = MTR_DIR_DISABLED;
  globalFlags.motorData.hBridgeState = HB_STATE_DISABLED;
}

/**
 * @brief Enable the motors and set PWMs to zero (assume motor direction to be FWD)
 */
static void enableMotors(void) {
  // Start the PWM channels
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // Set all PWMs to zero
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

  // Enable the h-bridge enable pin
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

  // Update data
  globalFlags.motorData.leftMotorSpeed = 0;
  globalFlags.motorData.rightMotorSpeed = 0;
  globalFlags.motorData.leftMotorDir = MTR_DIR_FWD;
  globalFlags.motorData.rightMotorDir = MTR_DIR_FWD;
  globalFlags.motorData.hBridgeState = HB_STATE_ENABLED;
}

/**
 * @brief Enable the motors and set PWMs to zero (assume motor direction to be FWD)
 * @param leftSpeed: Speed to set the left motor to
 * @param rightSpeed: Speed to set the right motor to
 * @note Speeds can be any value between -100 and 100 (reverse to forward)
 * @note This function does not enable motors if they are disabled (PWM off)
 */
static void setMotors(float leftSpeed, float rightSpeed) {
  // Load in data
  float leftSpeedCurrent = globalFlags.motorData.leftMotorSpeed;
  float rightSpeedCurrent = globalFlags.motorData.rightMotorSpeed;
  motorDir_t leftDirCurrent = globalFlags.motorData.leftMotorDir;
  motorDir_t rightDirCurrent = globalFlags.motorData.rightMotorDir;

  uint8_t dirChange = FALSE;
  uint8_t leftSpeedChangeCmplt = FALSE;
  uint8_t rightSpeedChangeCmplt = FALSE;

  // Check to see if the direction needs to change
  // If so, just to be safe, disable the H Bridge and make sure all PWM concerned is zero
  // Control direction change of left motor
  if ((leftSpeed >= 0) && (leftDirCurrent == MTR_DIR_REV)) { // If REV -> FWD
    // Disable the H Bridge
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    // Set speeds
    dirChange = TRUE;
    setLR(0);
    leftSpeedCurrent = 0;

    // Set directions
    leftDirCurrent = MTR_DIR_FWD;
    globalFlags.motorData.leftMotorDir = leftDirCurrent;

  } else if ((leftSpeed < 0) && (leftDirCurrent == MTR_DIR_FWD)) { // If FWD-> REV
    // Disable the H Bridge
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    // Set speeds
    dirChange = TRUE;
    setLF(0);
    leftSpeedCurrent = 0;

    // Set directions
    leftDirCurrent = MTR_DIR_REV;
    globalFlags.motorData.leftMotorDir = leftDirCurrent;

  }

  // Control direction change of right motor
  if ((rightSpeed >= 0) && (rightDirCurrent == MTR_DIR_REV)) { // If REV -> FWD
    // Disable the H Bridge
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    // Set speeds
    dirChange = TRUE;
    setRR(0);
    rightSpeedCurrent = 0;

    // Set directions
    rightDirCurrent = MTR_DIR_FWD;
    globalFlags.motorData.rightMotorDir = rightDirCurrent;

  } else if ((rightSpeed < 0) && (rightDirCurrent == MTR_DIR_FWD)) { // If FWD-> REV
    // Disable the H Bridge
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    // Set speeds
    dirChange = TRUE;
    setRF(0);
    rightSpeedCurrent = 0;

    // Set directions
    rightDirCurrent = MTR_DIR_REV;
    globalFlags.motorData.rightMotorDir = rightDirCurrent;

  }

  // If we have made a direction change (by stopping motors etc..)
  if (dirChange) {
    // Enable the Bridge again
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
  }

  // Check which PWM needs to be changed based on direction
  if (leftDirCurrent == MTR_DIR_FWD) {
    setLF(leftSpeed);
  } else {
    setLR(-leftSpeed);
  }

  // Check which PWM needs to be changed based on direction
  if (rightDirCurrent == MTR_DIR_FWD) {
    setRF(rightSpeed);
  } else {
    setRR(-rightSpeed);
  }

  // Update data
  globalFlags.motorData.leftMotorSpeed = leftSpeed;
  globalFlags.motorData.rightMotorSpeed = rightSpeed;
}

/**
 * @brief Disable the launcher completely
 */
static void disableLauncher(void) {
  // Disable the h-bridge enable pin
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  // Set all PWMs to zero
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);

  // Stop all PWM outputs
  HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);

  // Update data
  globalFlags.motorData.launcherHBridgeState = HB_STATE_DISABLED;
  globalFlags.states.launcherState = LNCH_STATE_OFF;
}


/**
 * @brief Enable the launcher and set PWMs to zero
 */
static void enableLauncher(void) {
  // Start the PWM channels
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

  // Set all PWMs to zero
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);

  // Enable the h-bridge enable pin
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

  // Update data
  globalFlags.motorData.launcherHBridgeState = HB_STATE_ENABLED;
  globalFlags.states.launcherState = LNCH_STATE_OFF; // Technically, the launcher is still off
}

/**
 * @brief Launch the ball by starting the launcher motor
 */
static void launch(void) {
  // Start the motor
  setLaunchF(100);

  // Update the flags
  globalFlags.states.launcherState = LNCH_STATE_RUNNING;
}

/**
 * @brief Stop the launcher by stopping the launcher motor
 */
static void terminateLaunch(void) {
  // Turn off the motor
  setLaunchF(0);

  // Update the flags
  globalFlags.states.launcherState = LNCH_STATE_OFF;
}

/**
 * @brief Set the PWM compare on the LF channel (TIM1 CH4)
 * @param speed: PWM value of the channel
 */
static void setLF(float speed) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t )(speed * 10));
}

/**
 * @brief Set the PWM compare on the LF channel (TIM1 CH2)
 * @param speed: PWM value of the channel
 */
static void setLR(float speed) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t )(speed * 10));
}

/**
 * @brief Set the PWM compare on the LF channel (TIM1 CH3)
 * @param speed: PWM value of the channel
 */
static void setRF(float speed) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t )(speed * 10));
}

/**
 * @brief Set the PWM compare on the LF channel (TIM1 CH1)
 * @param speed: PWM value of the channel
 */
static void setRR(float speed) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t )(speed * 10));
}

/**
 * @brief Set the PWM compare on the Launcher Forward channel (TIM15 CH1)
 * @param speed: PWM value of the channel
 */
static void setLaunchF(float speed) {
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (uint16_t )(speed * 10));
}

/**
 * @brief Set the PWM compare on the Launcher Revers channel (TIM15 CH2)
 * @param speed: PWM value of the channel
 */
static void setLaunchR(float speed) {
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, (uint16_t )(speed * 10));
}

