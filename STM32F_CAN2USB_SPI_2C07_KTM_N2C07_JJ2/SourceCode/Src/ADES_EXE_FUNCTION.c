
#include "main.h"

#ifdef FSP400
void EXE_CmdHt66VoltageMode(void)
{
  //
  if(g_ucVoltageMode == 0) // 18 Voltage Mode
  {
    // Info HT66 
    g_ucHT66ActionFlagMCU = 1;
    g_ucHT66CommandMcu[0] = HT66_ADDRESS;
    g_ucHT66CommandMcu[1] = HT66_CMD_0x5D;
    g_ucHT66CommandMcu[2] = HT66_ADDR_0x17;
    g_ucHT66CommandMcu[3] = 0xF0;
    g_ucHT66CmdDoubleFlag = 1;
  }
  else if(g_ucVoltageMode == 1) // 24 Voltage Mode                
  {
    // Info HT66 
    g_ucHT66ActionFlagMCU = 1;
    g_ucHT66CommandMcu[0] = HT66_ADDRESS;
    g_ucHT66CommandMcu[1] = HT66_CMD_0x5D;
    g_ucHT66CommandMcu[2] = HT66_ADDR_0x17;
    g_ucHT66CommandMcu[3] = 0x0F;
    g_ucHT66CmdDoubleFlag = 1;
  }
  else if(g_ucVoltageMode == 2) // 15 Voltage Mode                
  {
    //// Info HT66 
    //g_ucHT66ActionFlagMCU = 1;
    //g_ucHT66CommandMcu[0] = HT66_ADDRESS;
    //g_ucHT66CommandMcu[1] = HT66_CMD_0x5D;
    //g_ucHT66CommandMcu[2] = HT66_ADDR_0x17;
    //g_ucHT66CommandMcu[3] = 0x0F;
    //g_ucHT66CmdDoubleFlag = 1;
  }
}
#endif
