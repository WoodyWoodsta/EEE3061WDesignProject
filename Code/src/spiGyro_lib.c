//-f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg

#include "spiGyro_lib.h"

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

  // CS: Chip select pin (PB12) which simply outputs 1 to disable chip and 0 to enable chip
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // CSN = B12
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  chipDeSelect();

  // SPI Pins: PB13 - SCK, PB14 - MISO (SDO), PB15 - MOSI (SDI)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Set SPI pins to AF0 (SPI2xxx)
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_0);

  // SPI Initialization
  SPI_InitTypeDef SPI_InitStructure;

  SPI_I2S_DeInit(SPI2); // De-init the SPI2 to reset
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
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
  writeSPIgyro(0x23, 0b01000000); // Set the MSB to be first address
  writeSPIgyro(0x20, 0b00001111); // Switch the gyro into Normal Mode and enable all axes

  // Check if the gyro is responding or not
  uint8_t whoAmI = writeSPIgyro(0x0, 0x80);
  trace_printf("WHO_AM_I = %d", whoAmI);

  if (whoAmI == 0b11010100) {
    trace_puts("Gyro is saying hello!");
  } else {
    trace_puts("Gyro is not responding");
  }
}

/**
 * @brief Collect data from the gyro and call a 2s compliment conversion
 * @param out: Float pointer to output variable
 * @retval None
 */

void getGyro(float* out) {
  uint8_t crtlB;

  crtlB = (uint8_t) writeSPIgyro(0b10100011, 0x00); // Determines what range the gyro is in (250dps, 500dps or 2000dps)
  trace_printf("Range = %d", crtlB);

  uint8_t status = writeSPIgyro(0x27, 0x00);
  while (((status & 0b1000) == 0) || ((status & 0b10000000) == 1)) {
    // Wait for data to become available
  }

  uint8_t gyroXL = writeSPIgyro(0x28, 0x80);
  uint8_t gyroXH = writeSPIgyro(0x29, 0x80);
  uint8_t gyroYL = writeSPIgyro(0x2A, 0x80);
  uint8_t gyroYH = writeSPIgyro(0x2B, 0x00);
  uint8_t gyroZL = writeSPIgyro(0x2C, 0x00);
  uint8_t gyroZH = writeSPIgyro(0x2D, 0x00);

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
 * @brief Select the SPI slave chip to start communication
 * @param None
 * @retval None
 */

void chipSelect() {
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

/**
 * @brief Deselect the SPI slave chip to start communication
 * @param None
 * @retval None
 */

void chipDeSelect() {
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
  chipSelect();
  delay(10);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) {
    // Wait for all transmissions to complete
  }

  uint16_t concatData = (uint16_t) regAdr | (uint16_t) data << 8; // Concatenate the address and the data
  SPI_I2S_SendData16(SPI2, concatData); //send the data

  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) {
    //Wait for data
  }

  delay(200); // Delay for transmission, if you speed up the SPI then you can decrease this delay
  uint8_t badData = SPI_ReceiveData8(SPI2); // Unwanted data
  actualData = SPI_ReceiveData8(SPI2); // The actual data
  chipDeSelect();
  return (uint8_t) actualData;
}

int16_t twosCompToDec16(uint16_t val) // For 16 bit
{
  uint16_t v = val;
  int16_t temp = 0;
  uint8_t isNeg = FALSE;

  if ((v & (1 << 15))) {
    isNeg = TRUE;
  }

  // Subtract 1
  v--;

  // Convert to dec
  uint8_t pos;
  for (pos = 0; pos < 15; pos++) {
    temp = temp + ((v & (1 << pos))*pow(2, 4));
  }

  if (isNeg) {
    temp = -temp;
  }

  return (int16_t) temp;
}

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
  uint8_t SPIResponse = writeSPIgyro(0x0, 0x80);
  trace_printf("SPIgyro Responded with %u\n", (uint8_t) SPIResponse);
}

