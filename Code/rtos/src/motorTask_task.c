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

#define MTR_SPEED_REACTION_PERIOD 1 // Period of changing motor speed by 1% [+-ms]

// == Private Function Declerations ==
static void interpretCommand(msgCommand_t rxCommand);
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
  disableMotors();
  enableMotors();
  setMotors(100, -100);

  /* Infinite loop */
  for (;;) {
    // Wait for messages
    fetchMessage(msgQUSARTOut, &rxMessage, osWaitForever);

    // Indentify the type of message
    switch (rxMessage.messageType) {
    case MSG_TYPE_COMMAND:
      // If we have received a command, decode and interpret it
      interpretCommand(decodeCommand(&rxMessage));
      break;
    default:
      break;
    }
  }
}

/**
 * @brief Interpret the command received, and act on it
 * @param rxCommand: Command received in the message
 */
static void interpretCommand(msgCommand_t rxCommand) {
  switch (rxCommand) {
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
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

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
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
}

/**
 * @brief Set the PWM compare on the LF channel (TIM1 CH2)
 * @param speed: PWM value of the channel
 */
static void setLR(uint8_t speed) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
}

/**
 * @brief Set the PWM compare on the LF channel (TIM1 CH3)
 * @param speed: PWM value of the channel
 */
static void setRF(uint8_t speed) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
}

/**
 * @brief Set the PWM compare on the LF channel (TIM1 CH1)
 * @param speed: PWM value of the channel
 */
static void setRR(uint8_t speed) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}

