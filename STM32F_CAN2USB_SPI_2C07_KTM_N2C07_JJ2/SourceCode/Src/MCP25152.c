#include "main.h"
#include "MCP25152.h"
#include "stdbool.h"

/* Pin need to be modified according to the setting. Modify below items for your SPI configurations */
extern SPI_HandleTypeDef        hspi2;
#define SPI_CAN                 &hspi2
#define SPI_TIMEOUT             10
//#define MCP2515_CS2_HIGH()   HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET)
//#define MCP2515_CS2_LOW()    HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET)

/* Prototypes */
static void SPI_Tx(uint8_t data);
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length);
static uint8_t SPI_Rx(void);
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length);

/* 1 byte read */
uint8_t MCP2515_ReadByte (uint8_t address)
{
  uint8_t retVal;
  
  MCP2515_CS2_LOW();
  
  SPI_Tx(MCP2515_READ);
  SPI_Tx(address);
  retVal = SPI_Rx();
      
  MCP2515_CS2_HIGH();
  
  return retVal;
}

/* 1 byte writing */
void MCP2515_WriteByte(uint8_t address, uint8_t data)
{    
  MCP2515_CS2_LOW();  
  
  SPI_Tx(MCP2515_WRITE);
  SPI_Tx(address);
  SPI_Tx(data);  
    
  MCP2515_CS2_HIGH();
}

/* MCP2515 reset */
bool MCP2515_Initialize(void)
{
  MCP2515_CS2_HIGH();    
  
  uint8_t loop = 10;
  
  do {
    /* SPI Ready Confirm */
    if(HAL_SPI_GetState(SPI_CAN) == HAL_SPI_STATE_READY)
      return true;
    
    loop--;
  } while(loop > 0); 
      
  return false;
}

/* MCP2515 to setting mode */
bool MCP2515_SetConfigMode(void)
{
  /* CANCTRL Register Configuration Mode setting */  
  MCP2515_WriteByte(MCP2515_CANCTRL, 0x80);
  
  uint8_t loop = 10;
  
  do {    
    /* Mode change confirmation */    
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x80)
      return true;
    
    loop--;
  } while(loop > 0); 
  
  return false;
}

/* MCP2515 to Normal Switch mode */
bool MCP2515_SetNormalMode(void)
{
  /* CANCTRL Register Normal Mode setting */  
  MCP2515_WriteByte(MCP2515_CANCTRL, 0x00);
  
  uint8_t loop = 10;
  
  do {    
    /* Mode change confirmation */    
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x00)
      return true;
    
    loop--;
  } while(loop > 0);
  
  return false;
}

/* MCP2515 to Sleep switch mode */
bool MCP2515_SetSleepMode(void)
{
  /* CANCTRL Register Sleep Mode setting */  
  MCP2515_WriteByte(MCP2515_CANCTRL, 0x20);
  
  uint8_t loop = 10;
  
  do {    
    /* Mode change confirmation */    
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x20)
      return true;
    
    loop--;
  } while(loop > 0);
  
  return false;
}

/* MCP2515 SPI-Reset */
void MCP2515_Reset(void)
{    
  MCP2515_CS2_LOW();
      
  SPI_Tx(MCP2515_RESET);
      
  MCP2515_CS2_HIGH();
}

/* Sequential Bytes read */
void MCP2515_ReadRxSequence(uint8_t instruction, uint8_t *data, uint8_t length)
{
  MCP2515_CS2_LOW();
  
  SPI_Tx(instruction);        
  SPI_RxBuffer(data, length);
    
  MCP2515_CS2_HIGH();
}

/* Sequential Bytes writing */
void MCP2515_WriteByteSequence(uint8_t startAddress, uint8_t endAddress, uint8_t *data)
{    
  MCP2515_CS2_LOW();
  
  SPI_Tx(MCP2515_WRITE);
  SPI_Tx(startAddress);
  SPI_TxBuffer(data, (endAddress - startAddress + 1));
  
  MCP2515_CS2_HIGH();
}

/* TxBuffer on Sequential Bytes writing */
void MCP2515_LoadTxSequence(uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data)
{    
  MCP2515_CS2_LOW();
  
  SPI_Tx(instruction);
  SPI_TxBuffer(idReg, 4);
  SPI_Tx(dlc);
  SPI_TxBuffer(data, dlc);
       
  MCP2515_CS2_HIGH();
}

/* TxBuffer on 1 Bytes writing */
void MCP2515_LoadTxBuffer(uint8_t instruction, uint8_t data)
{
  MCP2515_CS2_LOW();
  
  SPI_Tx(instruction);
  SPI_Tx(data);
        
  MCP2515_CS2_HIGH();
}

/* RTS through command TxBuffer send */
void MCP2515_RequestToSend(uint8_t instruction)
{
  MCP2515_CS2_LOW();
  
  SPI_Tx(instruction);
      
  MCP2515_CS2_HIGH();
}

/* MCP2515 Status Confirm */
uint8_t MCP2515_ReadStatus(void)
{
  uint8_t retVal;
  
  MCP2515_CS2_LOW();
  
  SPI_Tx(MCP2515_READ_STATUS);
  retVal = SPI_Rx();
        
  MCP2515_CS2_HIGH();
  
  return retVal;
}

/* MCP2515 RxStatus Register check */
uint8_t MCP2515_GetRxStatus(void)
{
  uint8_t retVal;
  
  MCP2515_CS2_LOW();
  
  SPI_Tx(MCP2515_RX_STATUS);
  retVal = SPI_Rx();
        
  MCP2515_CS2_HIGH();
  
  return retVal;
}

/* Register value change */
void MCP2515_BitModify(uint8_t address, uint8_t mask, uint8_t data)
{    
  MCP2515_CS2_LOW();
  
  SPI_Tx(MCP2515_BIT_MOD);
  SPI_Tx(address);
  SPI_Tx(mask);
  SPI_Tx(data);
        
  MCP2515_CS2_HIGH();
}

/* SPI Tx Wrapper function */
static void SPI_Tx(uint8_t data)
{
  HAL_SPI_Transmit(SPI_CAN, &data, 1, SPI_TIMEOUT);    
}

/* SPI Tx Wrapper function */
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Transmit(SPI_CAN, buffer, length, SPI_TIMEOUT);    
}

/* SPI Rx Wrapper function */
static uint8_t SPI_Rx(void)
{
  uint8_t retVal;
  HAL_SPI_Receive(SPI_CAN, &retVal, 1, SPI_TIMEOUT);
  return retVal;
}

/* SPI Rx Wrapper function */
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Receive(SPI_CAN, buffer, length, SPI_TIMEOUT);
}
