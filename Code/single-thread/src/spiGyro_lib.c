/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | spiGyro_lib.c
 * @brief     | LD3G20 3-Axis Gyro Library (SPI)
 * @pre       | gyr
 * @authors   | Team 13
 *
 * This library is for use with the STM LD3D20 3-axis gyro sensor for the
 * EEE3061W Mechatronics Design Project.  It covers most of the operation of
 * the sensor required for the robot.
 *
 * Traces can be output using trace_puts() for strings or trace_printf() for
 * formatted strings
 *
 * ============================================================================
 */

// == Includes ==
#include "spiGyro_lib.h"
#include "pwmMotor_lib.h"
#include "ledGPIOB_lib.h"

// == Defines ==

// == Declarations ==
uint32_t ledStripCount = 0;
uint32_t ledStripPWMValue = 1000;

/**
 * @brief Initialise the SPI2 peripheral for use with the L3GD20
 * @param None
 * @retval None
 */

void gyr_SPIInit(void) {
  gyroState = GYROSTATE_OFF;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOA, ENABLE);

  // CS: GYRO Chip select pin (PB12) which simply outputs 1 to disable chip and 0 to enable chip
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // CSN = PB12
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  gyroChipDeselect();

  // SPI Pins: PB13 - SCK, PB14 - MISO (SDO), PB15 - MOSI (SDI/SDA)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // SPI Initialization
  SPI_InitTypeDef SPI_InitStructure;

  SPI_I2S_DeInit(SPI2); // De-init the SPI2 to reset
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;

  SPI_Init(SPI2, &SPI_InitStructure);
  SPI_Cmd(SPI2, ENABLE);
}

/**
 * @brief Initialise all modules required for operation
 * @param None
 * @retval None
 */

void gyr_opInit(void) {
  // Setup TIM6 as opTimer to be used for gyro operation
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); // Enable the RCC clock for TIM6
  TIM_TimeBaseInitTypeDef opTimerBaseInitStructure;

  TIM_Cmd(TIM6, DISABLE);
  TIM_DeInit(TIM6); // Reset the timer to default values

  TIM_TimeBaseStructInit(&opTimerBaseInitStructure);

  opTimerBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  opTimerBaseInitStructure.TIM_Period = 0xFFFF; // Set the ARR to 100              | This gives us a max time of 65.535 sec
  opTimerBaseInitStructure.TIM_Prescaler = 0xBB80; // Set the prescaler to 48000   | and a time-base of 1ms per LSB

  TIM_TimeBaseInit(TIM6, &opTimerBaseInitStructure);
}

/**
 * @brief Carry out the setup sequence
 * @param None
 * @retval None
 */

void gyr_setupRegisters(void) {
  // Write config to slave registers
  gyr_writeSPIgyro(0x21, 0b00101001); // Set a high-range high-pass filter (to counter drift)
  gyr_writeSPIgyro(0x23, 0b10010000); // Set the BDU to enabled and FS to 500dps
  gyr_writeSPIgyro(0x24, 0b00000011); // Enable high-pass and low pass filter
  gyr_writeSPIgyro(0x20, 0b10100111); // Enable all axes, set ODR to 380Hz and LP2 to 50Hz
}

/**
 * @brief Calibrate the gyro based on temperature (and initial bias)
 * @param gyr_calType_t calibrationType: The type of calibration to be performed
 * @retval None
 */

void gyr_calibrate(gyr_calType_t calibrationType) {
  gyroState = GYROSTATE_CALIBRATING;
  float velSample[3];
  static uint32_t initialTemp;
  uint32_t currentTemp;
  gyroFSReg = (uint8_t) (gyr_writeSPIgyro(0b10100011, 0x00) & 0x30); // Determines what range the gyro is in (250dps, 500dps or 2000dps)

  switch (calibrationType) {
  case GYROCAL_FULL:
    lcd_two_line_write("FULL CALIBRATION", "IN PROGRESS");
    initialTemp = ats_getTemp();
    uint8_t i;

    // Calculate the sensitivity coefficient before any reads are done
    switch (gyroFSReg) {
    case 0x00: // 250 dps
      senseConst = 8.575 + 0.0000028*((float) currentTemp);
      break;
    case 0x10: // 500 dps
      senseConst = 17.50 + 0.0000056*0.25*((float) currentTemp); // Was wrong before (this should be the one we use)
      break;
    case 0x20: // 2000 dps
      senseConst = 68.6 + 0.0000224*((float) currentTemp);
      break;
    }

    for(i = 0; i < BIAS_SAMPLE_WIDTH; i++) {
      gyr_getGyro(gyro_velocityData); // Grab some velocities
      uint8_t j;
      for(j = 0; j < 3; j++) {
        velSample[j] += gyro_velocityData[j]; // Add the individual axes to the sample array
      }
      delay(10);
    }

    for(i = 0; i < 3; i++) { // Complete the average and update bias values
      zeroBias[i] = velSample[i] / BIAS_SAMPLE_WIDTH;
    }

    // Zero the angle data
    gyro_angleData[0] = 0;
    gyro_angleData[1] = 0;
    gyro_angleData[2] = 0;

    // Reset the timer, just in case the timer was already running
    TIM_SetCounter(TIM6, 0);

    break;

  case GYROCAL_INTERVAL:
    currentTemp = ats_getTemp();

    // Calculate the sensitivity coefficient before any reads are done
    switch (gyroFSReg) {
    case 0x00: // 250 dps
      senseConst = 8.575 + 0.0000028*((float) currentTemp);
      break;
    case 0x10: // 500 dps
      senseConst = 17.50 + 0.0000056*0.25*((float) currentTemp); // Was wrong before (this should be the one we use)
      break;
    case 0x20: // 2000 dps
      senseConst = 68.6 + 0.0000224*((float) currentTemp);
      break;
    }

//    // Adjust the bias based on the change in temp
//    float biasDrift = ((int32_t)(currentTemp - initialTemp))*0.00003; // Taken from the 250dps scale value (this is so tiny)
//    for(i = 0; i < 3; i++) {
//      zeroBias[i] -= biasDrift;
//    }

    initialTemp = currentTemp; // Set the new "bias-temperature" for the next calibration

    break;

    case GYROCAL_PLAIN_ZERO:
      // Zero the angle data
      gyro_angleData[0] = 0;
      gyro_angleData[1] = 0;
      gyro_angleData[2] = 0;

      // Reset the timer
      TIM_SetCounter(TIM6, 0);

      break;
  }
  gyroState = GYROSTATE_RUNNING;
}

/**
 * @brief Collect data from the gyro and call a 2s compliment conversion
 * @note This cannot be read from too fast, although it's unlikely to be read from too fast, so yeah...
 * @param out: Float pointer to output variable (degrees/s)
 * @retval None
 */

void gyr_getGyro(float* out) {
  uint8_t gyroXL = gyr_writeSPIgyro(0xA8, 0x0);
  uint8_t gyroXH = gyr_writeSPIgyro(0xA9, 0x0);
  uint8_t gyroYL = gyr_writeSPIgyro(0xAA, 0x0);
  uint8_t gyroYH = gyr_writeSPIgyro(0xAB, 0x0);
  uint8_t gyroZL = gyr_writeSPIgyro(0xAC, 0x0);
  uint8_t gyroZH = gyr_writeSPIgyro(0xAD, 0x0);

  uint8_t buffer[6];

  buffer[1] = gyroXL;
  buffer[0] = gyroXH;
  buffer[3] = gyroYL;
  buffer[2] = gyroYH;
  buffer[5] = gyroZL;
  buffer[4] = gyroZH;

  uint16_t t = 0;
  int i = 0;

  for (i = 0; i < 3; i++) {
    t = (((uint16_t) buffer[2 * i] << 8) | buffer[2 * i + 1]);
    int16_t temp2 = (int16_t) t;
    out[i] = (float) ((temp2 * senseConst / 1000.0) - zeroBias[i]);
  }
}

/**
 * @brief Convert the float input, truncate to 4 decimal places and trace to the debugger
 * @param input: Float array of gyro values obtained using getGyro();
 * @retval None
 */

void gyr_prettyTraceGyroVelocity(float *input) {
  char result[50];
  float value;
  int32_t int_d;
  int32_t frac_d;
  float frac_f;

  // X Value
  value = input[0];
  int_d = value;
  frac_f = value - int_d;
  frac_d = fabs(trunc(frac_f * 10000));

//  sprintf(result, "Gyro X value = %d.%d", int_d, frac_d);

#ifdef CAPTURE
  trace_puts(result);
#endif

  // Y Value
  value = input[1];
  int_d = value;
  frac_f = value - int_d;
  frac_d = fabs(trunc(frac_f * 10000));

  sprintf(result, "Gyro Y value = %d.%d", int_d, frac_d);

#ifdef CAPTURE
  trace_puts(result);
#endif

  // Z Value
  value = input[2];
  int_d = value;
  frac_f = value - int_d;
  frac_d = fabs(trunc(frac_f * 10000));

  sprintf(result, "Gyro Z value = %d.%d", int_d, frac_d);

#ifdef CAPTURE
  trace_puts(result);
#endif

}

/**
 * @brief Convert the float input, truncate to 3 decimal places and print to screen
 * @param input: Float array of gyro values obtained using getGyro();
 * @retval None
 */

void gyr_prettyLCDGyroVelocity(float *input) {
  char resultLine1[16];
  char resultLine2[16];
  float value;
  int32_t Xint_d;
  int32_t Xfrac_d;
  float Xfrac_f;
  int32_t Yint_d;
  int32_t Yfrac_d;
  float Yfrac_f;
  int32_t Zint_d;
  int32_t Zfrac_d;
  float Zfrac_f;

  // X Value
  value = input[0];
  Xint_d = value;
  Xfrac_f = value - Xint_d;
  Xfrac_d = fabs(trunc(Xfrac_f * 1000));

  // Y Value
  value = input[1];
  Yint_d = value;
  Yfrac_f = value - Yint_d;
  Yfrac_d = fabs(trunc(Yfrac_f * 1000));

  // Z Value
  value = input[2];
  Zint_d = value;
  Zfrac_f = value - Zint_d;
  Zfrac_d = fabs(trunc(Zfrac_f * 1000));

  // Format and Print
  sprintf(resultLine1, "X:%d.%d", Xint_d, Xfrac_d);
  sprintf(resultLine2, "Y:%d.%d", Yint_d, Yfrac_d);

  lcd_two_line_write(resultLine1, resultLine2);
}

/**
 * @brief Convert the float velocity and angle inputs, truncate to 3 decimal places and print to screen
 * @param velocity: Float array of velocity values obtained using getGyro();
 *        angle: Float array of angle values obtained using getAngle();
 *        axis: Axis desired of @ref gyr_gyroAxis_t
 * @retval None
 */

void gyr_prettyLCDAxis(float *velocity, float *angle, gyr_gyroAxis_t axis) {
  char velocityString[16];
  char angleString[16];
  float value;
  int32_t int_d;
  int32_t frac_d;
  float frac_f;

  // Format the velocity
  value = velocity[axis];
  int_d = value;
  frac_f = value - int_d;
  frac_d = fabs(trunc(frac_f * 1000));

  sprintf(velocityString, "V = %d.%d", int_d, frac_d);

  // Format the angle
  value = angle[axis];
  int_d = value;
  frac_f = value - int_d;
  frac_d = fabs(trunc(frac_f * 1000));

  sprintf(angleString, "A = %d.%d", int_d, frac_d);

  lcd_two_line_write(angleString, velocityString);
}

/**
 * @brief Select the gyro slave chip to start communication
 * @param None
 * @retval None
 */

void gyroChipSelect(void) {
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

/**
 * @brief Deselect the gyro slave chip to start communication
 * @param None
 * @retval None
 */

void gyroChipDeselect(void) {
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

/**
 * @brief Read from or write to the slave
 * @param regAdr: Slave register address; data: The data to send to the register at regAdr
 * @retval Register data from the slave
 */

uint8_t gyr_writeSPIgyro(uint8_t regAdr, uint8_t data) {
  uint8_t dummyVar; // Holds the dummy data from the address register (send to receive)
  uint32_t actualData;
  gyroChipSelect();
  delay(10);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) {
    // Wait for all transmissions to complete
  }

  uint16_t concatData = (uint16_t) regAdr | (uint16_t) data << 8; // Concatenate the address and the data
  SPI_I2S_SendData16(SPI2, concatData); // Send the data

  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) {
    // Wait for data
  }

  delay(200); // Delay for transmission, if you speed up the SPI then you can decrease this delay
  uint8_t badData = SPI_ReceiveData8(SPI2); // Unwanted data
  actualData = SPI_ReceiveData8(SPI2); // The actual data
  gyroChipDeselect();
  return (uint8_t) actualData;
}

/**
 * @brief Converts a 16-bit 2's complement binary number into a 16-bit signed decimal
 * @param val: 2's complement binary number to be converted
 * @retval Converted 16-bit signed decimal
 */

int16_t twosCompToDec16(uint16_t val) // For 16 bit (this is totally unnecessary)
{
  uint16_t v = val;
  int16_t test = (int16_t) val;
  uint16_t negative = (v << 15) >> 15;
  if(negative) {
    v = (~(v << 1) >> 1) | 0b10000000;
  }

  int16_t temp = (v);

  return (int16_t) temp;
}

/**
 * @brief Creates a delay for a set number of microseconds
 * @param delay_in_us: Delay in microseconds
 * @retval None
 */

void delay(uint32_t delay_in_us) {
  /* Hangs for specified number of microseconds. */
  volatile uint32_t counter = 0;
  delay_in_us *= 3;
  for (; counter < delay_in_us; counter++) {
    __asm("nop");
    __asm("nop");
  }
}

/**
 * @brief Check the WHO_AM_I register to see if SPI communications is up and riding
 * @param None
 * @retval None
 */

void gyr_checkSPIResponse(void) {
  uint8_t SPIResponse = (uint8_t) gyr_writeSPIgyro(0b10001111, 0b10101010);

#ifdef CAPTURE
  trace_printf("SPIgyro Responded with %u\n", (uint8_t) SPIResponse);
#endif

  char SPIResponseChar[16];
  sprintf(SPIResponseChar, "%u",(uint8_t) SPIResponse);
  lcd_two_line_write("Gyro said:", SPIResponseChar);
}

/**
 * @brief Bring the gyro out of power down mode and start the timers
 * @note If this is the first time the method is being called, the gyro will run a full calibration
 * @param None
 * @retval None
 */

void gyr_gyroStart(void) {
  static uint8_t firstRun = TRUE;
  uint16_t gyroReg1 = gyr_writeSPIgyro(0xA0, 0x0); // Check the gyro control register 1
  gyr_writeSPIgyro(0x20, (gyroReg1 | (1 << 3))); // Bring the gyro out of power down mode
  // If this is the first time the gyro is being started, get the zero calibration
  if (firstRun) {
    gyroState = GYROSTATE_WAITING_FOR_ZERO;
    uint32_t ledCountStandby = 0;
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) { // Wait for the pushbutton press
      // Indicate when we are waiting for a zero press
      if (ledCountStandby == 600000) {
        led_1On();
        ledCountStandby = 0;
      } else if (ledCountStandby == 10000) {
        led_1Off();
        ledCountStandby++;
      } else {
        ledCountStandby++;
      }
    }
    led_0On();
    led_1On();
    gyr_calibrate(GYROCAL_FULL);
    led_0Off();
    led_1Off();
    uint32_t ledStripCount = 0;
    uint32_t ledStripPWMValue = 1000;
    while (ledStripCount < 850) {
      delay(600);
      ledStripCount++;
      ledStripPWMValue--;
      TIM_SetCompare1(TIM16, ledStripPWMValue);
    }

    firstRun = FALSE;
  } else {
    gyr_calibrate(GYROCAL_INTERVAL);
  }

  TIM_Cmd(TIM6, ENABLE); // Start the timer

  gyroState = GYROSTATE_RUNNING;
  mtr_motorLibraryState = MTR_LIB_STANDBY;
}

/**
 * @brief Get the current angle from the zero reference
 * @param Float pointer to where the angle will be saved
 * @retval None
 */

void gyr_getAngle(float *out) {
  led_0On();
  uint8_t axis;
  gyr_getGyro(gyro_velocityData);
  uint32_t timestep = TIM_GetCounter(TIM6); // Grab the elapsed time since the last read
  TIM_SetCounter(TIM6, 0); // Reset the timer for the next read
  for (axis = 0; axis < 3; axis++) {
    if ((gyro_velocityData[axis] >= VELOCITY_THRESHOLD) || (gyro_velocityData[axis] <= -VELOCITY_THRESHOLD)) { // Only integrate if the angle is outside of the threshold
      // Euler integrate to get the new angle
      out[axis] += (float) gyro_velocityData[axis]*(((float) timestep)/1000);
    }
  }

  led_0Off();
}

