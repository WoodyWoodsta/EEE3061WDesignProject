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

#define MTR_SPEED_REACTION_PERIOD     1 // Period of changing motor speed by 1% [+-ms]
#define MTR_UPDATE_PERIOD             10 // Interval between updating the motor speeds [+-ms]
#define LEFT_TRIM                     (int8_t)(0) // Percentage trim of the left motor
#define RIGHT_TRIM                    (int8_t)(0) // Percentage trim of the right motor
#define TRACKING_COMMIT               1 // Amount to decrease the motor speed by to achieve tracking
#define AUTO_SPEED                    100 // Percentage PWM to apply to motors when in auto drive mode
#define MIN_TRACKING_SPEED            (int8_t)(-20) // Minimum speed a motor can reach during tracking
#define SETTLE_SPEED                  2 // Amount to decrease the cumulative error when line is in center

// == Private Function Declerations ==
static void interpretSignal(osEvent *signalEvent);
static void stopMotors(void);
static void disableMotors(void);
static void enableMotors(void);
static void setMotors(int8_t leftSpeed, int8_t rightSpeed);

static void setLF(uint8_t speed);
static void setLR(uint8_t speed);
static void setRF(uint8_t speed);
static void setRR(uint8_t speed);

// == Function Definitions ==

/**
 * @brief motorTask
 * @param argument
 */
void StartMotorTask(void const * argument) {
  msg_genericMessage_t rxMessage;
  uint32_t errorCount = 0; // How many times have we updated the speed but have not achieved line tracking
  disableMotors();
  enableMotors();

  /* Infinite loop */
  for (;;) {
    globalFlags.generalData.motorTaskStackHWM = uxTaskGetStackHighWaterMark(motorTaskHandle);

    // If we are tracking the line
    if ((globalFlags.states.lineSensorState == LNS_STATE_ON)
        && globalFlags.states.motorState == MTR_STATE_RUNNING) {
      // Grab the current line position as picked up by the sensors
      linePos_t linePositionCurrent = globalFlags.lineSensorData.linePos;

      switch(linePositionCurrent) {
      case LINE_POS_LEFT: {
        // Calculate the adjustment
        int8_t newSpeed = globalFlags.motorData.leftMotorSpeed - (++errorCount*TRACKING_COMMIT);

        // Set the new speed of the motors, taking into account the minimum allowable speed
        setMotors((newSpeed < MIN_TRACKING_SPEED) ? (MIN_TRACKING_SPEED) : (newSpeed),
            globalFlags.motorData.rightMotorSpeed);
        break;
      }
      case LINE_POS_RIGHT: {
        // Calculate the adjustment
        int8_t newSpeed = globalFlags.motorData.rightMotorSpeed - (++errorCount*TRACKING_COMMIT);

        // Set the new speed of the motors, taking into account the minimum allowable speed
        setMotors(globalFlags.motorData.leftMotorSpeed,
            (newSpeed < MIN_TRACKING_SPEED) ? (MIN_TRACKING_SPEED) : (newSpeed));
        break;
      }
      case LINE_POS_CENTER:
        // If the line is tracked, set the speed of the motors to the set speed
        setMotors(AUTO_SPEED + LEFT_TRIM, AUTO_SPEED + RIGHT_TRIM);

        // Adjust the error counter
        // If it is not zero, decrease until zero is reached
        if (errorCount != 0) {
          errorCount -= SETTLE_SPEED;
          if (errorCount < 0) {
            errorCount = 0;
          }
        }
      }
    }

    osEvent signalEvent = osSignalWait(0, 0);

    if (signalEvent.status == osEventSignal) {
      interpretSignal(&signalEvent);
    }

    osDelay(MTR_UPDATE_PERIOD);
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

      // Send a signal to the line sensor task to enable sensing
      osSignalSet(lineSensorTaskHandle, LINE_SIG_START);

      // Update states
      globalFlags.states.motorState = MTR_STATE_RUNNING;
    }

    break;
  case MTR_SIG_STOP_TRACKING:
    // This muct only be fired if the motors are actually running
    if (globalFlags.states.motorState == MTR_STATE_RUNNING) {
      // Disable the motors
      disableMotors();

      // Send a signal to the line sensor task to disable sensing
      osSignalSet(lineSensorTaskHandle, LINE_SIG_STOP);

      // Update states
      globalFlags.states.motorState = MTR_STATE_OFF;
    }
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
static void setMotors(int8_t leftSpeed, int8_t rightSpeed) {
  // Load in data
  int8_t leftSpeedCurrent = globalFlags.motorData.leftMotorSpeed;
  int8_t rightSpeedCurrent = globalFlags.motorData.rightMotorSpeed;
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

  if (dirChange) {
    // Enable the Bridge again
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
  }

  // While the speeds still need to be changed
  while ((!leftSpeedChangeCmplt) || (!rightSpeedChangeCmplt)) {
    // Check if the speed needs to be changed
    if (leftSpeedCurrent != leftSpeed) {
      // Check which PWM needs to be changed based on direction
      if (leftDirCurrent == MTR_DIR_FWD) {
        setLF(++leftSpeedCurrent);
      } else {
        setLR(-(--leftSpeedCurrent));
      }
    } else {
      // Update when finished
      leftSpeedChangeCmplt = TRUE;
    }

    // Check if the speed needs to be changed
    if (rightSpeedCurrent != rightSpeed) {
      // Check which PWM needs to be changed based on direction
      if (rightDirCurrent == MTR_DIR_FWD) {
        setRF(++rightSpeedCurrent);
      } else {
        setRR(-(--rightSpeedCurrent));
      }
    } else {
      // Update when finished
      rightSpeedChangeCmplt = TRUE;
    }

    // Delay to allow a gradual change in speed
    osDelay(MTR_SPEED_REACTION_PERIOD);
  }

  // Update data
  globalFlags.motorData.leftMotorSpeed = leftSpeedCurrent;
  globalFlags.motorData.rightMotorSpeed = rightSpeedCurrent;
}

/**
 * @brief Set the PWM compare on the LF channel (TIM1 CH4)
 * @param speed: PWM value of the channel
 */
static void setLF(uint8_t speed) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed*10);
}

/**
 * @brief Set the PWM compare on the LF channel (TIM1 CH2)
 * @param speed: PWM value of the channel
 */
static void setLR(uint8_t speed) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed*10);
}

/**
 * @brief Set the PWM compare on the LF channel (TIM1 CH3)
 * @param speed: PWM value of the channel
 */
static void setRF(uint8_t speed) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed*10);
}

/**
 * @brief Set the PWM compare on the LF channel (TIM1 CH1)
 * @param speed: PWM value of the channel
 */
static void setRR(uint8_t speed) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed*10);
}

