/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | pwmMotor_lib.c
 * @brief     | Motor Control Library (PWM)
 * @pre       | mtr
 * @authors   | Team 13
 *
 * This library is used for control of two motors in both directions via PWM
 * to a quad-h-bridge (L293DNE). NOTE: Protection against shoot-through is
 * critical!
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

// == Includes ==
#include "pwmMotor_lib.h"

// == Defines ==


// == Global Variables ==


// == Helper Declarations ==
void setRF(uint8_t dutyCycle);
void setLF(uint8_t dutyCycle);
void setRR(uint8_t dutyCycle);
void setLR(uint8_t dutyCycle);


/**
 * @brief Initialise the timers for motor control
 * @param None
 * @retval None
 */

void mtr_motorTimInit(void) {
  // Left Motor Forward: PA11 - TIM1_CH4
  // Left Motor Reverse: PA3 - TIM15_CH2
  // Left Motor Enable: PB6
  // Right Motor Forward: PA8 - TIM1_CH1
  // Right Motor Reverse: PA2 - TIM15_CH1
  // Right Motor Enable: PB7

  mtr_motorENState = MTR_DISABLED;
  mtr_motorOpState = MTR_STOPPED;
  mtr_motorLibraryState = MTR_LIB_DISABLED;

  // Gather the RCC clocks for the timers we need
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);

  // Setup TIM7 as controlTimer to be used for PID control
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); // Enable the RCC clock for TIM6
  TIM_TimeBaseInitTypeDef controlTimerBaseInitStructure;

  TIM_Cmd(TIM7, DISABLE);
  TIM_DeInit(TIM7); // Reset the timer to default values
  TIM_TimeBaseStructInit(&controlTimerBaseInitStructure);

  controlTimerBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  controlTimerBaseInitStructure.TIM_Period = 0xFFFF; // Set the ARR to 100              | This gives us a max time of 65.535 sec
  controlTimerBaseInitStructure.TIM_Prescaler = 0xBB80; // Set the prescaler to 48000   | and a time-base of 1ms per LSB
  TIM_TimeBaseInit(TIM6, &controlTimerBaseInitStructure);

  TIM_DeInit(TIM1);
  TIM_DeInit(TIM15);

  //Initialise the Enable pins
  GPIO_InitTypeDef mtr_GPIOENInitStruct;
  mtr_GPIOENInitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  mtr_GPIOENInitStruct.GPIO_Mode = GPIO_Mode_OUT;
  mtr_GPIOENInitStruct.GPIO_OType = GPIO_OType_PP;
  mtr_GPIOENInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  mtr_GPIOENInitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_Init(GPIOB, &mtr_GPIOENInitStruct);

  // Double check they are low
  GPIO_ResetBits(GPIOB, GPIO_Pin_6);
  GPIO_ResetBits(GPIOB, GPIO_Pin_7);

  // Initialise the PWM GPIO pins
  GPIO_InitTypeDef mtr_GPIOPWMFInitStruct;
  mtr_GPIOPWMFInitStruct.GPIO_Mode = GPIO_Mode_AF; // Alternate Function for timer PWM output
  mtr_GPIOPWMFInitStruct.GPIO_OType = GPIO_OType_PP;
  mtr_GPIOPWMFInitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_11;
  mtr_GPIOPWMFInitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  mtr_GPIOPWMFInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &mtr_GPIOPWMFInitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);

  // Initialise the PWM for both TIM1 and TIM15
  TIM_TimeBaseInitTypeDef mtr_TIMTimebaseInitStruct;
  TIM_TimeBaseStructInit(&mtr_TIMTimebaseInitStruct); // Get the structure ready for init
  mtr_TIMTimebaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  mtr_TIMTimebaseInitStruct.TIM_Prescaler = 48; // Will give 1Mhz base
  mtr_TIMTimebaseInitStruct.TIM_Period = 1000; // 1KHz PWM frequency
  TIM_TimeBaseInit(TIM1, &mtr_TIMTimebaseInitStruct);
  TIM_TimeBaseInit(TIM15, &mtr_TIMTimebaseInitStruct);

  TIM_OCInitTypeDef mtr_TIMOCInitStruct;
  TIM_OCStructInit(&mtr_TIMOCInitStruct); // These are here to set everything to default to make the param assertion happy
  mtr_TIMOCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
  mtr_TIMOCInitStruct.TIM_OCMode = TIM_OCMode_PWM1; // With upcounting, this gives positive leading pulse and directly proportional duty cycle
  mtr_TIMOCInitStruct.TIM_OutputState = TIM_OutputState_Enable; // Enable the output?

  TIM_OC1Init(TIM1, &mtr_TIMOCInitStruct); // TIM1 Channel 1 output compare init
  TIM_OC4Init(TIM1, &mtr_TIMOCInitStruct); // TIM1 Channel 4 output compare init
  TIM_OC1Init(TIM15, &mtr_TIMOCInitStruct); // TIM15 Channel 1 output compare init
  TIM_OC2Init(TIM15, &mtr_TIMOCInitStruct); // TIM15 Channel 2 output compare init

  // Enable the capture compare on both TIM1 and TIM15
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
  TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable);

  TIM_CtrlPWMOutputs(TIM15, ENABLE);
  TIM_CCxCmd(TIM15, TIM_Channel_1, TIM_CCx_Enable);
  TIM_CCxCmd(TIM15, TIM_Channel_2, TIM_CCx_Enable);

  // Make sure the motors are off
  mtr_stop();

}


/**
 * @brief Set/reset the enable GPIOs for the chosen side of the H-Bridge
 * @param None
 * @retval None
 */

void mtr_leftEnable(void) {
  GPIO_SetBits(GPIOB, GPIO_Pin_6);
  mtr_motorENState = MTR_ENABLED;
}

void mtr_leftDisable(void) {
  GPIO_ResetBits(GPIOB, GPIO_Pin_6);
}

void mtr_rightEnable(void) {
  GPIO_SetBits(GPIOB, GPIO_Pin_7);
  mtr_motorENState = MTR_ENABLED;

}

void mtr_rightDisable(void) {
  GPIO_ResetBits(GPIOB, GPIO_Pin_7);
}

/**
 * @brief Bring the motors to the speed specified and cater for large changes in speed
 * @param direction: Which direction the motor should be driven
 *        leftSpeed: Speed of the left motor
 *        rightSpeed: Speed of the right motor
 * @retval None
 */

void mtr_setSpeed(mtr_motorOpState_t direction, uint16_t leftSpeed, uint16_t rightSpeed) {
  int16_t leftSpeedDiff = 0;
  int16_t rightSpeedDiff = 0;

  switch (direction) {
  case MTR_FORWARD:
    if (mtr_motorOpState != MTR_FORWARD) {
      mtr_stop(); // This will set everything to zero (including the CC)
      mtr_leftEnable();
      mtr_rightEnable();
      TIM_Cmd(TIM1, ENABLE);
      mtr_motorOpState = MTR_FORWARD;
    }

    leftSpeedDiff = leftSpeed - mtr_motorLCrtSpd;
    rightSpeedDiff = rightSpeed - mtr_motorRCrtSpd;

    if ((abs(leftSpeedDiff) >= MOTOR_ATTACK_THRESHOLD) || (abs(rightSpeedDiff) >= MOTOR_ATTACK_THRESHOLD)) {
      uint16_t changeTime = 0; // In 10 ms
      float leftSpeedAttack = 0;
      float rightSpeedAttack = 0;
      // Check to see which motor is the master motor (which is changing the most and will sit at the attack maximum
      if (abs(leftSpeedDiff) >= abs(rightSpeedDiff)) {
        changeTime = abs(leftSpeedDiff)/MOTOR_ATTACK; // Calculate how long the change will be given the attack
        leftSpeedAttack = MOTOR_ATTACK; // Left motor will be the master change
        rightSpeedAttack = rightSpeedDiff/changeTime; // Calculate the right speed attack given the calculated time of change

        uint16_t rightSpeedInt = 0;
        uint16_t changeCount = 0;
        uint16_t leftSpeedInitial = mtr_motorLCrtSpd;
        uint16_t rightSpeedInitial = mtr_motorRCrtSpd;
        // Make the change!
        for (changeCount = 0; changeCount < changeTime; changeCount++) {
          setLF(leftSpeedInitial + (changeCount*leftSpeedAttack)); // Set the left speed to the new value
          rightSpeedInt = rightSpeedInitial + (changeCount*rightSpeedAttack); // Convert to an int because is won't initially be
          setRF(rightSpeedInt); // Set the right speed to the new value
          delay(10000); // Delay for 10ms
        }
      } else { // Master motor must be the other one
        changeTime = abs(rightSpeedDiff)/MOTOR_ATTACK; // Calculate how long the change will be given the attack
        rightSpeedAttack = MOTOR_ATTACK; // Left motor will be the master change
        leftSpeedAttack = leftSpeedDiff/changeTime; // Calculate the right speed attack given the calculated time of change

        uint16_t leftSpeedInt = 0;
        uint16_t changeCount = 0;
        uint16_t leftSpeedInitial = mtr_motorLCrtSpd;
        uint16_t rightSpeedInitial = mtr_motorRCrtSpd;
        // Make the change!
        for (changeCount = 0; changeCount < changeTime; changeCount++) {
          setRF(rightSpeedInitial + (changeCount*rightSpeedAttack)); // Set the left speed to the new value
          leftSpeedInt = leftSpeedInitial + (changeCount*leftSpeedAttack); // Convert to an int because is won't initially be
          setLF(leftSpeedInt); // Set the right speed to the new value
          delay(10000); // Delay for 10ms
        }
      }
    } else { // Or just change the value
      setLF(leftSpeed);
      setRF(rightSpeed);
    }
    break;

  case MTR_REVERSE:
    if (mtr_motorOpState != MTR_REVERSE) {
      mtr_stop(); // This will set everything to zero (including the CC)
      mtr_leftEnable();
      mtr_rightEnable();
      TIM_Cmd(TIM15, ENABLE);
      mtr_motorOpState = MTR_REVERSE;
    }

    leftSpeedDiff = leftSpeed - mtr_motorLCrtSpd;
    rightSpeedDiff = rightSpeed - mtr_motorRCrtSpd;

    if ((abs(leftSpeedDiff) >= MOTOR_ATTACK_THRESHOLD) || (abs(rightSpeedDiff) >= MOTOR_ATTACK_THRESHOLD)) {
      uint16_t changeTime = 0; // In 10 ms
      float leftSpeedAttack = 0;
      float rightSpeedAttack = 0;
      // Check to see which motor is the master motor (which is changing the most and will sit at the attack maximum
      if (abs(leftSpeedDiff) >= abs(rightSpeedDiff)) {
        changeTime = abs(leftSpeedDiff)/MOTOR_ATTACK; // Calculate how long the change will be given the attack
        leftSpeedAttack = MOTOR_ATTACK; // Left motor will be the master change
        rightSpeedAttack = rightSpeedDiff/changeTime; // Calculate the right speed attack given the calculated time of change

        uint16_t rightSpeedInt = 0;
        uint16_t changeCount = 0;
        uint16_t leftSpeedInitial = mtr_motorLCrtSpd;
        uint16_t rightSpeedInitial = mtr_motorRCrtSpd;
        // Make the change!
        for (changeCount = 0; changeCount < changeTime; changeCount++) {
          setLR(leftSpeedInitial + (changeCount*leftSpeedAttack)); // Set the left speed to the new value
          rightSpeedInt = rightSpeedInitial + (changeCount*rightSpeedAttack); // Convert to an int because is won't initially be
          setRR(rightSpeedInt); // Set the right speed to the new value
          delay(10000); // Delay for 10ms
        }
      } else { // Master motor must be the other one
        changeTime = abs(rightSpeedDiff)/MOTOR_ATTACK; // Calculate how long the change will be given the attack
        rightSpeedAttack = MOTOR_ATTACK; // Left motor will be the master change
        leftSpeedAttack = leftSpeedDiff/changeTime; // Calculate the right speed attack given the calculated time of change

        uint16_t leftSpeedInt = 0;
        uint16_t changeCount = 0;
        uint16_t leftSpeedInitial = mtr_motorLCrtSpd;
        uint16_t rightSpeedInitial = mtr_motorRCrtSpd;
        // Make the change!
        for (changeCount = 0; changeCount < changeTime; changeCount++) {
          setRR(rightSpeedInitial + (changeCount*rightSpeedAttack)); // Set the left speed to the new value
          leftSpeedInt = leftSpeedInitial + (changeCount*leftSpeedAttack); // Convert to an int because is won't initially be
          setLR(leftSpeedInt); // Set the right speed to the new value
          delay(10000); // Delay for 10ms
        }
      }
    } else { // Or just change the value
      setLR(leftSpeed);
      setRR(rightSpeed);
    }
    break;
  default:
    mtr_stop();
  }
}

void mtr_rotate(mtr_motorOpState_t direction, uint16_t angle, uint16_t speed) {
  int16_t leftSpeedDiff = 0;
  int16_t rightSpeedDiff = 0;

  switch (direction) {
  case MTR_ROTATECW:
    if (mtr_motorOpState != MTR_ROTATECW) {
      mtr_stop(); // This will set everything to zero (including the CC)
      mtr_leftEnable();
      mtr_rightEnable();
      TIM_Cmd(TIM1, ENABLE);
      delay(10000); // Allow for any noise, spikes etc
      TIM_Cmd(TIM15, ENABLE);
      mtr_motorOpState = MTR_ROTATECW;
    }

    while(gyro_angleData(2) != angle) {

      leftSpeedDiff = speed - mtr_motorLCrtSpd;
      rightSpeedDiff = speed - mtr_motorRCrtSpd;

      if ((abs(leftSpeedDiff) >= MOTOR_ATTACK_THRESHOLD) || (abs(rightSpeedDiff) >= MOTOR_ATTACK_THRESHOLD)) {
        uint16_t changeTime = 0; // In 10 ms
        float leftSpeedAttack = 0;
        float rightSpeedAttack = 0;
        // Check to see which motor is the master motor (which is changing the most and will sit at the attack maximum
        if (abs(leftSpeedDiff) >= abs(rightSpeedDiff)) {
          changeTime = abs(leftSpeedDiff)/MOTOR_ATTACK; // Calculate how long the change will be given the attack
          leftSpeedAttack = MOTOR_ATTACK; // Left motor will be the master change
          rightSpeedAttack = rightSpeedDiff/changeTime; // Calculate the right speed attack given the calculated time of change

          uint16_t rightSpeedInt = 0;
          uint16_t changeCount = 0;
          uint16_t leftSpeedInitial = mtr_motorLCrtSpd;
          uint16_t rightSpeedInitial = mtr_motorRCrtSpd;
          // Make the change!
          for (changeCount = 0; changeCount < changeTime; changeCount++) {
            setLF(leftSpeedInitial + (changeCount*leftSpeedAttack)); // Set the left speed to the new value
            rightSpeedInt = rightSpeedInitial + (changeCount*rightSpeedAttack); // Convert to an int because is won't initially be
            setRR(rightSpeedInt); // Set the right speed to the new value
            delay(10000); // Delay for 10ms
          }
        } else { // Master motor must be the other one
          changeTime = abs(rightSpeedDiff)/MOTOR_ATTACK; // Calculate how long the change will be given the attack
          rightSpeedAttack = MOTOR_ATTACK; // Left motor will be the master change
          leftSpeedAttack = leftSpeedDiff/changeTime; // Calculate the right speed attack given the calculated time of change

          uint16_t leftSpeedInt = 0;
          uint16_t changeCount = 0;
          uint16_t leftSpeedInitial = mtr_motorLCrtSpd;
          uint16_t rightSpeedInitial = mtr_motorRCrtSpd;
          // Make the change!
          for (changeCount = 0; changeCount < changeTime; changeCount++) {
            setRR(rightSpeedInitial + (changeCount*rightSpeedAttack)); // Set the left speed to the new value
            leftSpeedInt = leftSpeedInitial + (changeCount*leftSpeedAttack); // Convert to an int because is won't initially be
            setLF(leftSpeedInt); // Set the right speed to the new value
            delay(10000); // Delay for 10ms
          }
        }
      } else { // Or just change the value
        setLF(speed);
        setRR(speed);
      }
    }
    break;

  case MTR_ROTATECCW:
    if (mtr_motorOpState != MTR_ROTATECCW) {
      mtr_stop(); // This will set everything to zero (including the CC)
      mtr_leftEnable();
      mtr_rightEnable();
      TIM_Cmd(TIM1, ENABLE);
      delay(10000); // Allow for any noise, spikes etc
      TIM_Cmd(TIM15, ENABLE);
      mtr_motorOpState = MTR_ROTATECCW;
    }

    while(gyro_angleData(2) != angle) {
      leftSpeedDiff = speed - mtr_motorLCrtSpd;
      rightSpeedDiff = speed - mtr_motorRCrtSpd;

      if ((abs(leftSpeedDiff) >= MOTOR_ATTACK_THRESHOLD) || (abs(rightSpeedDiff) >= MOTOR_ATTACK_THRESHOLD)) {
        uint16_t changeTime = 0; // In 10 ms
        float leftSpeedAttack = 0;
        float rightSpeedAttack = 0;
        // Check to see which motor is the master motor (which is changing the most and will sit at the attack maximum
        if (abs(leftSpeedDiff) >= abs(rightSpeedDiff)) {
          changeTime = abs(leftSpeedDiff)/MOTOR_ATTACK; // Calculate how long the change will be given the attack
          leftSpeedAttack = MOTOR_ATTACK; // Left motor will be the master change
          rightSpeedAttack = rightSpeedDiff/changeTime; // Calculate the right speed attack given the calculated time of change

          uint16_t rightSpeedInt = 0;
          uint16_t changeCount = 0;
          uint16_t leftSpeedInitial = mtr_motorLCrtSpd;
          uint16_t rightSpeedInitial = mtr_motorRCrtSpd;
          // Make the change!
          for (changeCount = 0; changeCount < changeTime; changeCount++) {
            setLR(leftSpeedInitial + (changeCount*leftSpeedAttack)); // Set the left speed to the new value
            rightSpeedInt = rightSpeedInitial + (changeCount*rightSpeedAttack); // Convert to an int because is won't initially be
            setRF(rightSpeedInt); // Set the right speed to the new value
            delay(10000); // Delay for 10ms
          }
        } else { // Master motor must be the other one
          changeTime = abs(rightSpeedDiff)/MOTOR_ATTACK; // Calculate how long the change will be given the attack
          rightSpeedAttack = MOTOR_ATTACK; // Left motor will be the master change
          leftSpeedAttack = leftSpeedDiff/changeTime; // Calculate the right speed attack given the calculated time of change

          uint16_t leftSpeedInt = 0;
          uint16_t changeCount = 0;
          uint16_t leftSpeedInitial = mtr_motorLCrtSpd;
          uint16_t rightSpeedInitial = mtr_motorRCrtSpd;
          // Make the change!
          for (changeCount = 0; changeCount < changeTime; changeCount++) {
            setRF(rightSpeedInitial + (changeCount*rightSpeedAttack)); // Set the left speed to the new value
            leftSpeedInt = leftSpeedInitial + (changeCount*leftSpeedAttack); // Convert to an int because is won't initially be
            setLR(leftSpeedInt); // Set the right speed to the new value
            delay(10000); // Delay for 10ms
          }
        }
      } else { // Or just change the value
        setLR(speed);
        setRF(speed);
      }
    }
    break;
  default:
    mtr_stop();
  }
}


/**
 * @brief Set the duty cycle of the PWM output to the Right Motor Forward Direction pin
 * @param duty: Duty cycle from 0-100%
 * @retval None
 */

void setRF(uint8_t duty) {
  uint16_t pwmDuty = duty*10;
  TIM_SetCompare1(TIM1, pwmDuty);
  mtr_motorRCrtSpd = duty;
}

/**
 * @brief Set the duty cycle of the PWM output to the Left Motor Forward Direction pin
 * @param duty: Duty cycle from 0-100%
 * @retval None
 */

void setLF(uint8_t duty) {
  uint16_t pwmDuty = duty*10;
  TIM_SetCompare4(TIM1, pwmDuty);
  mtr_motorLCrtSpd = duty;
}

/**
 * @brief Set the duty cycle of the PWM output to the Right Motor Reverse Direction pin
 * @param duty: Duty cycle from 0-100%
 * @retval None
 */

void setRR(uint8_t duty) {
  uint16_t pwmDuty = duty*10;
  TIM_SetCompare1(TIM15, pwmDuty);
  mtr_motorRCrtSpd = -duty;
}

/**
 * @brief Set the duty cycle of the PWM output to the Left Motor Reverse Direction pin
 * @param duty: Duty cycle from 0-100%
 * @retval None
 */

void setLR(uint8_t duty) {
  uint16_t pwmDuty = duty*10;
  TIM_SetCompare2(TIM15, pwmDuty);
  mtr_motorLCrtSpd = -duty;
}

/**
 * @brief Stop the motors. Disables the H-bridge and the timers and sets the PWM to zero for all directions
 * @param None
 * @retval None
 */

void mtr_stop() {
  mtr_leftDisable();
  mtr_rightDisable();
  mtr_motorENState = MTR_DISABLED;

  TIM_Cmd(TIM1, DISABLE);
  TIM_Cmd(TIM15, DISABLE);

  setLF(0);
  setRF(0);
  setLR(0);
  setRR(0);
  mtr_motorOpState = MTR_STOPPED;

  delay(5000); // Delay for 5ms just to make sure the timers aren't going at the same time.
}

