/**
 *  ----------------------------------------------------------------------------
 *  Copyright (c) 2012-13, Anaren Microwave, Inc.
 *
 *  For more information on licensing, please see Anaren Microwave, Inc's
 *  end user software licensing agreement: EULA.txt
 *
 *  ----------------------------------------------------------------------------
 *
 *  SimplexTransfer.c - acts as the Gateway node for the SimplexTransfer
 *  example. Receives packets from the Simplex End Point node(s) and stores it
 *  in a local packet.
 *
 *  @version    1.0.00
 *  @date       04 Feb 2013
 *  @author     BPB, air@anaren.com
 *
 *  assumptions
 *  ===========
 *  - this is being compiled exclusively with Gateway node(s).
 *  
 *  file dependency
 *  ===============
 *  string.h : defines memcpy which is used to copy one buffer to another
 *  API.h : defines the protocol API.
 *  
 *  revision history
 *  ================
 *  ver 1.0.00 : 04 Feb 2013
 *  - initial release
 */
#ifndef bool
#define bool unsigned char
#endif

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

#include <string.h>       // memcpy
#include "API.h"

#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>
#include <stdlib.h>



#include "macro.h"
#include "radio.h"
#include "node.h"     
#include "uartapp.h" 
#include "timeapp.h" 
#include "ledapp.h"
extern SEGMENT_VARIABLE_SEGMENT_POINTER(pRadioConfiguration, tRadioConfiguration) ;
extern SEGMENT_VARIABLE( Si446xCmd, union si446x_cmd_reply_union);
// -----------------------------------------------------------------------------
/**
 *  Defines, enumerations, and structure definitions
 */

#define ST(X) do { X } while (0)

/**
 *  sPacket - an example packet. The sequence number is used to demonstrate
 *  communication by sending the same message (payload) and incrementing the
 *  sequence number on each transmission.
 */
struct sPacket
{
  unsigned char seqNum;     // Packet sequence number
  unsigned char payload[7]; // Packet payload
};

// -----------------------------------------------------------------------------
/**
 *  Callback function prototypes
 */

/**
 *  TransferComplete - acts as the callback function for the Protocol Data
 *  Transfer Complete event. When a frame is received that meets addressing
 *  requirements, the Protocol Data Transfer Complete event is triggered and a
 *  callback must be implemented to receive the frame payload (packet).
 *
 *  Note: Please refer to API.h for more information on the TransferComplete
 *  callback for a Gateway node.
 */
unsigned char TransferComplete(bool dataRequest,
                               unsigned char *data,
                               unsigned char length);

// -----------------------------------------------------------------------------
/**
 *  Global data
 */

/**
 *  The following instance of sProtocolSetupInfo is used to initialize the
 *  protocol with required parameters. The Gateway node role has the following
 *  parameters [optional parameters are marked with an asterix (*)].
 *        
 *  { channel_list, pan_id, local_address, LinkRequest(*), TransferComplete(*) }
 *
 *  Note: Parameters marked with the asterix (*) may be assigned "NULL" if they
 *  are not needed.
 */
static const struct sProtocolSetupInfo gProtocolSetupInfo = {
  { PROTOCOL_CHANNEL_LIST },// Physical channel list
  { 0x01,0x01 },                 // Physical address PAN identifier
  { 0x01,0x02,0x03,0x04 },                 // Physical address
  NULL,                     // Protocol Link Request callback (not used)
  TransferComplete          // Protocol Data Transfer Complete callback
};

static struct sPacket gPacket = {
  0x00,                     // Set the initial sequence number value to 0
  ""                        // Set the initial payload to an empty string
};

// -----------------------------------------------------------------------------

unsigned char TransferComplete(bool dataRequest,
                               unsigned char *data,
                               unsigned char length)
{
  // Cast the received data pointer to a packet structure pointer so that it may
  // be accessed using the structure member notation.
  //struct sPacket *p = (struct sPacket*)data;
  
  // Retrieve the sequence number and copy the payload from the protocol into
  // the local application packet (gPacket).
  //gPacket.seqNum = p->seqNum;
  //memcpy(gPacket.payload, p->payload, length-1);
  RfRxData2PubTx(data, length);
  
  return 0;
}

/**
 *  main - main application loop. Sets up platform and then goes to sleep for
 *  the lifetime of execution.
 *
 *    @return   Exit code of the main application, however, this application
 *              should never exit.
 */
int main(void)
{
  RCC_ClocksTypeDef RCC_ClockFreq;
  RCC_GetClocksFreq(&RCC_ClockFreq);
  
  if (SysTick_Config(RCC_ClockFreq.HCLK_Frequency/1000))
  { 
    /* Capture error */ 
    while (1);
  }
  
  Il_Hw_Init(); 
  
  Init_SI4463_Pin();
  
  vRadio_Init(); 
  
  ClkSwitch2HseSystemInit();

  RCC_GetClocksFreq(&RCC_ClockFreq);
  
  if (SysTick_Config(RCC_ClockFreq.HCLK_Frequency/1000))
  { 
    /* Capture error */ 
    while (1);
  }
  
  Il_Hw_Init(); 
  
  Uart_Init();
    
  Init_SI4463_Pin();
  
//TimingBaseInit(50000);
  
//  while(1)
//  {
////  //LedD4StaInvert();  
////    GPIOB->BSRR  = 0x00000040;
////    GPIOB->BRR  = 0x00000040;
//  }
  
  //EXTILine_TimingSync_Config();
  
  //si446x_get_int_status(0u, 0u, 0u);
  uint8_t testBuff[6]={0x05,0xD0,0x01,0x02,0x03,0xD6};
  
  //UartSendByte(testBuff, 6); 
  
  vRadio_StartRX(pRadioConfiguration->Radio_ChannelNumber);
  
  RadioGotoRxSta();
  
  SI4463_Enable_NIRQ_Int();

  if (!ProtocolInit(&gProtocolSetupInfo))
  {
    return false;
  }
  
  while(1)
  {
    if(GetPubTxBufCount()>0x00)
    {
      PubTxDataProcess();
    }
    //NopDelayMs(50);
    //si446x_get_modem_status();  
    
    if(GetPubRxBufCount()>0x00)
    {
      PubRxDataProcess();
    }
    
  }
}


/**
 *  GDO0Isr - GDO0 interrupt service routine. This service routine will always
 *  be a I/O interrupt service routine. Therefore, it is important to pass the
 *  port flag to the ProtocolEngine so that the protocol can determine if a GDO0
 *  event has occurred.
 */
//#pragma vector=GDO0_VECTOR
void GDO0Isr(void)
{
  /**
   *  Store the port interrupt flag register so that it may be used to determine
   *  what caused the interrupt (e.g. did a GDO0 interrupt occur? The protocol 
   *  needs this information to determine what to do next...).
   */
  register volatile unsigned char event; //= GDO0_EVENT;
  
  /**
   *  Notify the protocol of a port event. The protocol will determine if it is
   *  associated with the GDO0 pin and act accordingly (e.g. if GDO0 has not
   *  triggered an interrupt then the protocol will not run, otherwise it will
   *  continue where it left off).
   *
   *  Note: Clearing of the GDO0 event (interrupt flag bit) is handled 
   *  internally. It is important that the application does not clear the GDO0
   *  event in this ISR.
   */
  ProtocolEngine(event);
}

// Note: No hardware timer interrupt required for this example because the 
// Gateway node does not use it for anything at this time.
