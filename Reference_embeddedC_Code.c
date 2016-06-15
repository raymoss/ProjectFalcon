Reference
Not sure when i will use this code but lets keep it. ITs 3 hrs of work right!!!

void i2cInitialize(I2C_TypeDef* I2C){
/*setting the frequency of I2C transmission as 2MHz. Change this if required.*/
int frequency,risetime
  I2C->CR2|= (uint16_t)(0b000010);;
initialise the clock of the peripheral as mentioned in the manual*/
if (I2C==I2C1)
  APB1ENR |= RCC_APB1ENR_I2C1EN ;
else if (I2C==I2C2)
  APB1ENR |= RCC_APB1ENR_I2C2EN;
else if (I2C==I2C3)
  APBIENR |= RCC_APB1ENR_I2C3EN;
/*Need to add an assertion statement for the error checking . */
/*Setting the rise time according to the formula = max rise time/(frequency of the clock ) +1 . */
frequency = I2C->CR2 & (uint16_t)(0x111111);
risetime= (1000/frequency) + 1 ;
I2C->TRISE |= (uint8)(risetime);
/*Enabling the peripheral*/
I2C->CR1 |= I2C_CR1_PE;
}

void i2cSend(uint8_t address,I2C_TypeDef* I2C,uint8_t data){
uint16_t value;
  /*Need to send the start bit */
  I2C->CR1 |= I2C_CR1_START ;
  /*Check start bit is generated*/
  value = I2C->SR1 ;
  if !(value&I2C_SR1_SB)
    continue ; /*Need to have a recovery mechanism*/
  I2C->DR = address;
  
  
}