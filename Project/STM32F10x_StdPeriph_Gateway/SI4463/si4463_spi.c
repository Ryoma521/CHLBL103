#include "si4463_spi.h"
#include "macro.h"
#include "radio_mcu.h"
#include "si4463_def.h"

U8 bSpi_ReadWriteSpi1(U8 biDataIn, U8 wrFlag)
{  
  uint8 x;
  uint8 status = 0;

  for(x=8;x>0;x--)
  {
    if(wrFlag==SPIWRITE)
    {
      if(biDataIn & 0x80)                                    // If bit is high...
      {
        //GPIOA->BSRR = GPIO_Pin_6;
        SI4463_SDI_Up();                                 // Set SIMO high...
      }
      else
      {
        //GPIOA->BRR = GPIO_Pin_6;
        SI4463_SDI_Down();                               // Set SIMO low...
      }
      biDataIn = biDataIn << 1;                          // Rotate bits
    }
    //DelayUs(10);
    SI4463_SCLK_Down();                                // Set clock low
    //DelayUs(10);
    SI4463_SCLK_Up();                                  // Set clock high
    //DelayUs(10);
    
    if(wrFlag==SPIREAD)
    {
    status = status << 1;
    status |= SI4463_SDO_Bit();
    }
  }

  SI4463_SCLK_Down();

  return status;
}

void vSpi_ReadDataSpi1(U16 biDataOutLength, U8 *paboDataOut)
{
  while (biDataOutLength--) {
    *paboDataOut++ = bSpi_ReadWriteSpi1(0xFF,SPIREAD);
  }
}

void vSpi_WriteDataSpi1(U16 biDataInLength, U8 *pabiDataIn)
{
  while (biDataInLength--) {
    bSpi_ReadWriteSpi1(*pabiDataIn++,SPIWRITE);
  }
}

