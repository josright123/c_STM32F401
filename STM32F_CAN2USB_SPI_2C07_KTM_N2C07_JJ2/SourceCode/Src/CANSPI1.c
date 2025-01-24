#include "CANSPI1.h"
#include "MCP25151.h"
#include "stdbool.h"

/** Local Function Prototypes */  
static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL);
static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) ;
static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t1 *passedidReg1);

/** Local Variables */ 
ctrl_status_t1 ctrlStatus1;
ctrl_error_status_t1 errorStatus1;
id_reg_t1 idReg1;

/** CAN SPI APIs */ 

/* Sleep Entering mode */
void CANSPI1_Sleep(void)
{
  /* Clear CAN bus wakeup interrupt */
  MCP25151_BitModify(MCP2515_CANINTF, 0x40, 0x00);        
  
  /* Enable CAN bus activity wakeup */
  MCP25151_BitModify(MCP2515_CANINTE, 0x40, 0x40);        
  
  MCP25151_SetSleepMode();
}

/* CAN Communication initialization */
bool CANSPI1_Initialize(void)
{
  RXF0 RXF0reg;
  RXF1 RXF1reg;
  RXF2 RXF2reg;
  RXF3 RXF3reg;
  RXF4 RXF4reg;
  RXF5 RXF5reg;
  RXM0 RXM0reg;
  RXM1 RXM1reg;
      
  /* Rx Mask values reset */
  RXM0reg.RXM0SIDH = 0x00;
  RXM0reg.RXM0SIDL = 0x00;
  RXM0reg.RXM0EID8 = 0x00;
  RXM0reg.RXM0EID0 = 0x00;
  
  RXM1reg.RXM1SIDH = 0x00;
  RXM1reg.RXM1SIDL = 0x00;
  RXM1reg.RXM1EID8 = 0x00;
  RXM1reg.RXM1EID0 = 0x00;
  
  /* Rx Filter values reset */
  RXF0reg.RXF0SIDH = 0x00;      
  RXF0reg.RXF0SIDL = 0x00;      //Starndard Filter
  RXF0reg.RXF0EID8 = 0x00;
  RXF0reg.RXF0EID0 = 0x00;
  
  RXF1reg.RXF1SIDH = 0x00;
  RXF1reg.RXF1SIDL = 0x08;      //Exntended Filter
  RXF1reg.RXF1EID8 = 0x00;
  RXF1reg.RXF1EID0 = 0x00;
  
  RXF2reg.RXF2SIDH = 0x00;
  RXF2reg.RXF2SIDL = 0x00;
  RXF2reg.RXF2EID8 = 0x00;
  RXF2reg.RXF2EID0 = 0x00;
  
  RXF3reg.RXF3SIDH = 0x00;
  RXF3reg.RXF3SIDL = 0x00;
  RXF3reg.RXF3EID8 = 0x00;
  RXF3reg.RXF3EID0 = 0x00;
  
  RXF4reg.RXF4SIDH = 0x00;
  RXF4reg.RXF4SIDL = 0x00;
  RXF4reg.RXF4EID8 = 0x00;
  RXF4reg.RXF4EID0 = 0x00;
  
  RXF5reg.RXF5SIDH = 0x00;
  RXF5reg.RXF5SIDL = 0x08;
  RXF5reg.RXF5EID8 = 0x00;
  RXF5reg.RXF5EID0 = 0x00;
  
  /* MCP2515 reset, SPI Communication status check */
  if(!MCP25151_Initialize())
    return false;
    
  /* Configuration Set to mode */
  if(!MCP25151_SetConfigMode())
    return false;
  
  /* Filter & Mask Value setting */
  MCP25151_WriteByteSequence(MCP2515_RXM0SIDH, MCP2515_RXM0EID0, &(RXM0reg.RXM0SIDH));
  MCP25151_WriteByteSequence(MCP2515_RXM1SIDH, MCP2515_RXM1EID0, &(RXM1reg.RXM1SIDH));
  MCP25151_WriteByteSequence(MCP2515_RXF0SIDH, MCP2515_RXF0EID0, &(RXF0reg.RXF0SIDH));
  MCP25151_WriteByteSequence(MCP2515_RXF1SIDH, MCP2515_RXF1EID0, &(RXF1reg.RXF1SIDH));
  MCP25151_WriteByteSequence(MCP2515_RXF2SIDH, MCP2515_RXF2EID0, &(RXF2reg.RXF2SIDH));
  MCP25151_WriteByteSequence(MCP2515_RXF3SIDH, MCP2515_RXF3EID0, &(RXF3reg.RXF3SIDH));
  MCP25151_WriteByteSequence(MCP2515_RXF4SIDH, MCP2515_RXF4EID0, &(RXF4reg.RXF4SIDH));
  MCP25151_WriteByteSequence(MCP2515_RXF5SIDH, MCP2515_RXF5EID0, &(RXF5reg.RXF5SIDH));
  
  /* Accept All (Standard + Extended) */
  MCP25151_WriteByte(MCP2515_RXB0CTRL, 0x04);    //Enable BUKT, Accept Filter 0
  MCP25151_WriteByte(MCP2515_RXB1CTRL, 0x01);    //Accept Filter 1
      
  /* 
  * tq = 2 * (brp(0) + 1) / 8000000 MHz = 0.25us
  * tbit = (SYNC_SEG(1 fixed) + PROP_SEG + PS1 + PS2)
  * tbit = 1tq + 1tq + 2tq + 4tq = 8tq
  * 8tq = 2us = 500kbps
  */
  
  /* 00(SJW 1tq) 000000 */  
  MCP25151_WriteByte(MCP2515_CNF1, 0x00);
  
  /* 1 0 001(1tq) 000(1tq) */  
  MCP25151_WriteByte(MCP2515_CNF2, 0x88);
  
  /* 0 0 000 011(4tq) */  
  MCP25151_WriteByte(MCP2515_CNF3, 0x03);
  
  /* Set to Normal mode */
  if(!MCP25151_SetNormalMode())
    return false;
  
  return true;
}

/* CAN Send message */
uint8_t CANSPI1_Transmit(uCAN_MSG *tempCanMsg) 
{
  uint8_t returnValue = 0;
  
  idReg1.tempSIDH = 0;
  idReg1.tempSIDL = 0;
  idReg1.tempEID8 = 0;
  idReg1.tempEID0 = 0;
  
  ctrlStatus1.ctrl_status1 = MCP25151_ReadStatus();
  
  /* Now Transmission Finds and transmits this unpending buffer. */
  if (ctrlStatus1.TXB0REQ != 1)
  {
    /* ID Type Convert to fit */
    convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg1);
    
    /* Tx Buffer Loading data to be sent to */
    MCP25151_LoadTxSequence(MCP2515_LOAD_TXB0SIDH, &(idReg1.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
    
    /* Tx Buffer Data transmission request */
    MCP25151_RequestToSend(MCP2515_RTS_TX0);
    
    returnValue = 1;
  }
  else if (ctrlStatus1.TXB1REQ != 1)
  {
    convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg1);
    
    MCP25151_LoadTxSequence(MCP2515_LOAD_TXB1SIDH, &(idReg1.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
    MCP25151_RequestToSend(MCP2515_RTS_TX1);
    
    returnValue = 1;
  }
  else if (ctrlStatus1.TXB2REQ != 1)
  {
    convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg1);
    
    MCP25151_LoadTxSequence(MCP2515_LOAD_TXB2SIDH, &(idReg1.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
    MCP25151_RequestToSend(MCP2515_RTS_TX2);
    
    returnValue = 1;
  }
  
  return (returnValue);
}

/* CAN Receive message */
uint8_t CANSPI1_Receive(uCAN_MSG *tempCanMsg) 
{
  uint8_t returnValue = 0;
  rx_reg_t rxReg;
  ctrl_rx_status_t rxStatus;
  
  rxStatus.ctrl_rx_status = MCP25151_GetRxStatus();
  
  /* Check if there are any messages received in the buffer */
  if (rxStatus.rxBuffer != 0)
  {
    /* Check which buffer contains messages and process */
    if ((rxStatus.rxBuffer == MSG_IN_RXB0)|(rxStatus.rxBuffer == MSG_IN_BOTH_BUFFERS))
    {
      MCP25151_ReadRxSequence(MCP2515_READ_RXB0SIDH, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));
    }
    else if (rxStatus.rxBuffer == MSG_IN_RXB1)
    {
      MCP25151_ReadRxSequence(MCP2515_READ_RXB1SIDH, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));
    }
    
    /* Extended type */
    if (rxStatus.msgType == dEXTENDED_CAN_MSG_ID_2_0B)
    {
      tempCanMsg->frame.idType = (uint8_t) dEXTENDED_CAN_MSG_ID_2_0B;
      tempCanMsg->frame.id = convertReg2ExtendedCANid(rxReg.RXBnEID8, rxReg.RXBnEID0, rxReg.RXBnSIDH, rxReg.RXBnSIDL);
    } 
    else 
    {
      /* Standard type */
      tempCanMsg->frame.idType = (uint8_t) dSTANDARD_CAN_MSG_ID_2_0B;
      tempCanMsg->frame.id = convertReg2StandardCANid(rxReg.RXBnSIDH, rxReg.RXBnSIDL);
    }
    
    tempCanMsg->frame.dlc   = rxReg.RXBnDLC;
    tempCanMsg->frame.data0 = rxReg.RXBnD0;
    tempCanMsg->frame.data1 = rxReg.RXBnD1;
    tempCanMsg->frame.data2 = rxReg.RXBnD2;
    tempCanMsg->frame.data3 = rxReg.RXBnD3;
    tempCanMsg->frame.data4 = rxReg.RXBnD4;
    tempCanMsg->frame.data5 = rxReg.RXBnD5;
    tempCanMsg->frame.data6 = rxReg.RXBnD6;
    tempCanMsg->frame.data7 = rxReg.RXBnD7;
    
    returnValue = 1;
  }
  
  return (returnValue);
}

/* Check if there are messages in the receive buffer */
uint8_t CANSPI1_messagesInBuffer(void)
{
  uint8_t messageCount = 0;
  
  ctrlStatus1.ctrl_status1 = MCP25151_ReadStatus();
  
  if(ctrlStatus1.RX0IF != 0)
  {
    messageCount++;
  }
  
  if(ctrlStatus1.RX1IF != 0)
  {
    messageCount++;
  }
  
  return (messageCount);
}

/* Check if CAN BUS is offline */
uint8_t CANSPI1_isBussOff(void)
{
  uint8_t returnValue = 0;
  
  errorStatus1.error_flag_reg1 = MCP25151_ReadByte(MCP2515_EFLG);
  
  if(errorStatus1.TXBO == 1)
  {
    returnValue = 1;
  }
  
  return (returnValue);
}

/* Rx Passive Error Check if it is in a state */
uint8_t CANSPI1_isRxErrorPassive(void)
{
  uint8_t returnValue = 0;
  
  errorStatus1.error_flag_reg1 = MCP25151_ReadByte(MCP2515_EFLG);
  
  if(errorStatus1.RXEP == 1)
  {
    returnValue = 1;
  }
  
  return (returnValue);
}

/* Tx Passive Error Check if it is in a state */
uint8_t CANSPI1_isTxErrorPassive(void)
{
  uint8_t returnValue = 0;
  
  errorStatus1.error_flag_reg1 = MCP25151_ReadByte(MCP2515_EFLG);
  
  if(errorStatus1.TXEP == 1)
  {
    returnValue = 1;
  }
  
  return (returnValue);
}

/* Register Save value Extended ID Function to convert to type */
static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) 
{
  uint32_t returnValue = 0;
  uint32_t ConvertedID = 0;
  uint8_t CAN_standardLo_ID_lo2bits;
  uint8_t CAN_standardLo_ID_hi3bits;
  
  CAN_standardLo_ID_lo2bits = (tempRXBn_SIDL & 0x03);
  CAN_standardLo_ID_hi3bits = (tempRXBn_SIDL >> 5);
  ConvertedID = (tempRXBn_SIDH << 3);
  ConvertedID = ConvertedID + CAN_standardLo_ID_hi3bits;
  ConvertedID = (ConvertedID << 2);
  ConvertedID = ConvertedID + CAN_standardLo_ID_lo2bits;
  ConvertedID = (ConvertedID << 8);
  ConvertedID = ConvertedID + tempRXBn_EIDH;
  ConvertedID = (ConvertedID << 8);
  ConvertedID = ConvertedID + tempRXBn_EIDL;
  returnValue = ConvertedID;    
  return (returnValue);
}

/* Register Save value Standard ID Function to convert to type */
static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) 
{
  uint32_t returnValue = 0;
  uint32_t ConvertedID;
  
  ConvertedID = (tempRXBn_SIDH << 3);
  ConvertedID = ConvertedID + (tempRXBn_SIDL >> 5);
  returnValue = ConvertedID;
  
  return (returnValue);
}

/* CAN ID to Register Function to store in */
static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t1 *passedidReg1) 
{
  uint8_t wipSIDL = 0;
  
  if (canIdType == dEXTENDED_CAN_MSG_ID_2_0B) 
  {
    //EID0
    passedidReg1->tempEID0 = 0xFF & tempPassedInID;
    tempPassedInID = tempPassedInID >> 8;
    
    //EID8
    passedidReg1->tempEID8 = 0xFF & tempPassedInID;
    tempPassedInID = tempPassedInID >> 8;
    
    //SIDL
    wipSIDL = 0x03 & tempPassedInID;
    tempPassedInID = tempPassedInID << 3;
    wipSIDL = (0xE0 & tempPassedInID) + wipSIDL;
    wipSIDL = wipSIDL + 0x08;
    passedidReg1->tempSIDL = 0xEB & wipSIDL;
    
    //SIDH
    tempPassedInID = tempPassedInID >> 8;
    passedidReg1->tempSIDH = 0xFF & tempPassedInID;
  } 
  else
  {
    passedidReg1->tempEID8 = 0;
    passedidReg1->tempEID0 = 0;
    tempPassedInID = tempPassedInID << 5;
    passedidReg1->tempSIDL = 0xFF & tempPassedInID;
    tempPassedInID = tempPassedInID >> 8;
    passedidReg1->tempSIDH = 0xFF & tempPassedInID;
  }
}
