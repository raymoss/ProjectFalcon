//SPI Library for STM32F4Discovery
#include "my_spi.h"
uint8_t SPITimeout;
/*Setting for SPI2 which is clocked from APB1 as 42 Mhz*/
void SPI_LowLevel_Init(void)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* Enable CS  GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);    //Using Pin 4 as Pin 3 is being used for enabling the accel.
  
  

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPI2);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //90/16=5 Mbps
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(SPI2, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI2, ENABLE);

  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
    GPIO_SetBits(GPIOE, GPIO_Pin_4);
}
uint8_t SPI_SendByte(uint8_t byte)
{
  /* Loop while DR register in not emplty */
 
  SPITimeout = SPI_TIMEOUT_MAX;
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
  {
    if((SPITimeout--) == 0){ 
        usart_printf(USARTx,"Timeout reached while checking TXE flag\n\r");
        return -1;
    
    }
	ms_delay(10);
  }
  
  /* Send a Byte through the SPI peripheral */
  SPI_I2S_SendData(SPI2, byte);
  
  /* Wait to receive a Byte */
  SPITimeout = SPI_TIMEOUT_MAX;
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if((SPITimeout--) == 0){ 
        usart_printf(USARTx,"Timeout reached while checking RXNE flag\n\r");
        return -1;
    }
    ms_delay(10);
  }


   /* Return the Byte read from the SPI bus */
  return (SPI_I2S_ReceiveData(SPI2));
}
uint8_t SPI_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  uint8_t status;  
  //GPIO_SetBits(GPIOE, GPIO_Pin_4);
//  if(NumByteToRead > 0x01)
//  {
//    ReadAddr |= (uint8_t)(READWRITE_CMD);
//  }
//  else
//  {
//    ReadAddr |= (uint8_t)READWRITE_CMD;
//  }
  /* Set chip select Low at the start of the transmission */
 // LIS3DSH_CS_LOW();
   //GPIO_ResetBits(GPIOE, GPIO_Pin_4); 
  /* Send the Address of the indexed register */
  status=SPI_SendByte(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to LIS3DSH (Slave device) */
    *pBuffer = SPI_SendByte(DUMMY_BYTE);
    //usart_printf(USARTx,"Printing the value %d\n\r",*pBuffer);
    NumByteToRead--;
    pBuffer++;
  }
    //GPIO_SetBits(GPIOE, GPIO_Pin_4);
  /* Set chip select High at the end of the transmission */ 
  //LIS3DSH_CS_HIGH();
return status;
}
uint8_t SPI_Write(const uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8_t status;  
  //GPIO_SetBits(GPIOE, GPIO_Pin_4);
  /* Configure the MS bit: 
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
//  if(NumByteToWrite > 0x01)
//  {
//    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
//  }
    //GPIO_ResetBits(GPIOE, GPIO_Pin_4);
  /* Set chip select Low at the start of the transmission */
  //LIS3DSH_CS_LOW();
  
  /* Send the Address of the indexed register */
  status=SPI_SendByte(WriteAddr);
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    SPI_SendByte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
    //GPIO_SetBits(GPIOE, GPIO_Pin_4);
  /* Set chip select High at the end of the transmission */ 
  //LIS3DSH_CS_HIGH();
return status;
}
