/**
 * ============================================================================
 * EEE3061W Design Project
 * ============================================================================
 *
 * @file      | spiGyro_lib.c
 * @brief     | LD3G20 3-Axis Gyro Library (SPI)
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

// == Defines ==

// == Declarations ==

/**
 * @brief Initialise the pins for the LEDs
 * @param None
 * @retval None
 */

void init_leds(void) {
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
      | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief Turn half of the LEDs on
 * @param None
 * @retval None
 */
// TODO: Move to correct file

void half_on() {
  GPIO_SetBits(GPIOB, GPIO_Pin_0);
  GPIO_ResetBits(GPIOB, GPIO_Pin_1);
  GPIO_SetBits(GPIOB, GPIO_Pin_2);
  GPIO_ResetBits(GPIOB, GPIO_Pin_3);
  GPIO_SetBits(GPIOB, GPIO_Pin_4);
  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
  GPIO_SetBits(GPIOB, GPIO_Pin_6);
  GPIO_ResetBits(GPIOB, GPIO_Pin_7);
}

/**
 * @brief Turn the other half of the LEDs on
 * @param None
 * @retval None
 */
// TODO: Move to correct file

void other_half_on() {
  GPIO_ResetBits(GPIOB, GPIO_Pin_0);
  GPIO_SetBits(GPIOB, GPIO_Pin_1);
  GPIO_ResetBits(GPIOB, GPIO_Pin_2);
  GPIO_SetBits(GPIOB, GPIO_Pin_3);
  GPIO_ResetBits(GPIOB, GPIO_Pin_4);
  GPIO_SetBits(GPIOB, GPIO_Pin_5);
  GPIO_ResetBits(GPIOB, GPIO_Pin_6);
  GPIO_SetBits(GPIOB, GPIO_Pin_7);
}

/**
 * @brief Initialise the SPI2 peripheral for use with the L3GD20
 * @param None
 * @retval None
 */

void init_spi() {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOA, ENABLE);

  // CS: Gyro chip select pin (PA8) which simply outputs 1 to disable chip and 0 to enable chip
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // CSN = PA8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // CS: EEPROM Chip select pin (PB12) which simply outputs 1 to disable chip and 0 to enable chip
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // CSN = PB12
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  EEPROMChipDeselect();
  gyroChipDeselect();

  // SPI Pins: PB13 - SCK, PB14 - MISO (SDO), PB15 - MOSI (SDI/SDA)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Set SPI pins to AF0 (SPI2xxx)
//  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_0);
//  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_0);
//  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_0);

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
 * @brief Carry out the setup sequence
 * @param None
 * @retval None
 */

void setup_gyro_registers(void) {
  // Write config to slave registers
  writeSPIgyro(0x23, 0b10000000); // Set the BDU to enabled
  writeSPIgyro(0x20, 0b11111111); // Switch the gyro into Normal Mode, enable all axes and set highest data transfer frequency

  // Check if the gyro is responding or not
//  uint8_t whoAmI = writeSPIgyro(0x8F, 0x0);
//  trace_printf("WHO_AM_I = %u", whoAmI);

//  if (whoAmI == 0b11010100) {
//    trace_puts("Gyro is saying hello!");
//  } else {
//    trace_puts("Gyro is not responding");
//  }
}

/**
 * @brief Collect data from the gyro and call a 2s compliment conversion
 * @param out: Float pointer to output variable
 * @retval None
 */

void getGyro(float* out) {
  uint8_t crtlB;

  crtlB = (uint8_t) writeSPIgyro(0b10100011, 0x00); // Determines what range the gyro is in (250dps, 500dps or 2000dps)
#ifdef CAPTURE
  trace_printf("Range = %d\n", crtlB);
#endif

  delay(200000);
  uint8_t status = writeSPIgyro(0xA7, 0x00);
  while (((status & 0b1000) == 0) || ((status & 0b10000000) == 1)) {
    // Wait for data to become available
  }

  uint8_t gyroXL = writeSPIgyro(0xA8, 0x0);
  uint8_t gyroXH = writeSPIgyro(0xA9, 0x0);
  uint8_t gyroYL = writeSPIgyro(0xAA, 0x0);
  uint8_t gyroYH = writeSPIgyro(0xAB, 0x0);
  uint8_t gyroZL = writeSPIgyro(0xAC, 0x0);
  uint8_t gyroZH = writeSPIgyro(0xAD, 0x0);

  uint8_t buffer[6];

  buffer[1] = gyroXL;
  buffer[0] = gyroXH;
  buffer[3] = gyroYL;
  buffer[2] = gyroYH;
  buffer[5] = gyroZL;
  buffer[4] = gyroZH;

  uint16_t t = 0;
  int i = 0;
  uint8_t temp = (uint8_t) (crtlB & 0x30);

  switch (temp) {
  case (uint8_t) 0x00: //250dps
    for (i = 0; i < 3; i++) {
      t = (((uint16_t) buffer[2 * i] << 8) | buffer[2 * i + 1]);
      int16_t temp2 = twosCompToDec16(t);
      out[i] = (float) ((temp2 * 8.75 / 1000.0));
    }
    break;
  case (uint8_t) 0x10: //500dps
    for (i = 0; i < 3; i++) {
      t = (((uint16_t) buffer[2 * i] << 8) | buffer[2 * i + 1]);
      int16_t temp2 = twosCompToDec16(t);
      out[i] = (float) ((temp2 * 17.5 / 1000.0));
    }
    break;
  case (uint8_t) 0x20: //2000dps
    for (i = 0; i < 3; i++) {
      t = (((uint16_t) buffer[2 * i] << 8) | buffer[2 * i + 1]);
      int16_t temp2 = twosCompToDec16(t);
      out[i] = (float) ((temp2 * 70 / 1000.0));
    }
    break;
  case (uint8_t) 0x30: //20000dps
    for (i = 0; i < 3; i++) {
      t = (((uint16_t) buffer[2 * i] << 8) | buffer[2 * i + 1]);
      int16_t temp2 = twosCompToDec16(t);
      out[i] = (float) ((temp2 * 70 / 1000.0));
    }
    break;
  }
}

/**
 * @brief Convert the float input, truncate to 4 decimal places and trace to the debugger
 * @param input: Float array of gyro values obtained using getGyro();
 * @retval None
 */

void prettyTraceGyro(float *input) {
  char result[50];
  float value;
  int32_t int_d;
  int32_t frac_d;
  float frac_f;

  // X Value
  value = gyro[0];
  int_d = value;
  frac_f = value - int_d;
  frac_d = fabs(trunc(frac_f * 10000));

  sprintf(result, "Gyro X value = %d.%d", int_d, frac_d);

#ifdef CAPTURE
  trace_puts(result);
#endif

  // Y Value
  value = gyro[1];
  int_d = value;
  frac_f = value - int_d;
  frac_d = fabs(trunc(frac_f * 10000));

  sprintf(result, "Gyro Y value = %d.%d", int_d, frac_d);

#ifdef CAPTURE
  trace_puts(result);
#endif

  // Z Value
  value = gyro[2];
  int_d = value;
  frac_f = value - int_d;
  frac_d = fabs(trunc(frac_f * 10000));

  sprintf(result, "Gyro Z value = %d.%d", int_d, frac_d);

#ifdef CAPTURE
  trace_puts(result);
#endif

}

/**
 * @brief Convert the float input, truncate to 4 decimal places and print to screen
 * @param input: Float array of gyro values obtained using getGyro();
 * @retval None
 */

void prettyLCDGyro(float *gyro) {
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
  value = gyro[0];
  Xint_d = value;
  Xfrac_f = value - Xint_d;
  Xfrac_d = fabs(trunc(Xfrac_f * 10000));

  // Y Value
  value = gyro[1];
  Yint_d = value;
  Yfrac_f = value - Yint_d;
  Yfrac_d = fabs(trunc(Yfrac_f * 10000));

  // Z Value
//  value = gyro[2];
//  Zint_d = value;
//  Zfrac_f = value - int_d;
//  Zfrac_d = fabs(trunc(frac_f * 10000));

  // Format and Print
  sprintf(resultLine1, "X:%d.%d", Xint_d, Xfrac_d);
  sprintf(resultLine2, "Y:%d.%d", Yint_d, Yfrac_d);

  lcd_two_line_write(resultLine1, resultLine2);
}

/**
 * @brief Select the gyro slave chip to start communication
 * @param None
 * @retval None
 */

void gyroChipSelect() {
  GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

/**
 * @brief Deselect the gyro slave chip to start communication
 * @param None
 * @retval None
 */

void gyroChipDeselect() {
  GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

/**
 * @brief Select the EEPROM slave chip to start communication
 * @param None
 * @retval None
 */

void EEPROMChipSelect() {
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

/**
 * @brief Deselect the EEPROM slave chip to start communication
 * @param None
 * @retval None
 */

void EEPROMChipDeselect() {
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

/**
 * @brief Read from or write to the slave
 * @param regAdr: Slave register address; data: The data to send to the register at regAdr
 * @retval Register data from the slave
 */

uint8_t writeSPIgyro(uint8_t regAdr, uint8_t data) {
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

int16_t twosCompToDec16(uint16_t val) // For 16 bit
{
  uint16_t v = val;
  int16_t temp = 0;
  uint8_t isNeg = FALSE;

  if ((v & (1 << 15))) {
    isNeg = TRUE;
    // Invert bits
    v = v^(0xFFFF);
    // Add 1
    v++;
  }

  // Convert to dec
  uint8_t pos;
  for (pos = 0; pos <= 15; pos++) {
    temp = temp + ((v & (1 << pos))*pow(2, 4));
  }

  if (isNeg) {
    temp = -temp;
  }

  return (int16_t) temp;
}

/**
 * @brief Creates a delay for a set number of microseconds
 * @param delay_in_us: Delay in microseconds
 * @retval None
 */

static void delay(uint32_t delay_in_us) {
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

void checkSPIResponse() {
  uint8_t SPIResponse = (uint8_t) writeSPIgyro(0b10001111, 0b10101010);

#ifdef CAPTURE
  trace_printf("SPIgyro Responded with %u\n", (uint8_t) SPIResponse);
#endif

  char SPIResponseChar[16];
  sprintf(SPIResponseChar, "%u",(uint8_t) SPIResponse);
  lcd_two_line_write("Gyro said:", SPIResponseChar);
}

