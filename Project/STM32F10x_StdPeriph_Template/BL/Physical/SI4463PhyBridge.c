/**
 *  ----------------------------------------------------------------------------
 *  Copyright (c) 2012-13, Anaren Microwave, Inc.
 *
 *  For more information on licensing, please see Anaren Microwave, Inc's
 *  end user software licensing agreement: EULA.txt.
 *
 *  ----------------------------------------------------------------------------
 *
 *  A110x2500PhyBridge.c - physical bridge implementation using A110x2500-based 
 *		modules.
 *
 *  @version    1.0.01
 *  @date       17 Oct 2012
 *  @author     B Blincoe, bblincoe@anaren.com
 *
 *  assumptions
 *  ===========
 *  none
 *
 *  file dependency
 *  ===============
 *  A110x2500PhyBridge : provides interface function prototypes and global 
 *		definitions.
 *		string.h : provides function for setting a block of memory (memset).
 *
 *  revision history
 *  ================
 *  ver 1.0.01 : 17 Oct 2012
 *  - updated internal documentation; comments revised
 *  - added a test example (stub) to perform various frame operations
 *  ver 1.0.00 : 24 Jul 2012 
 *  - initial release
 */
#include "SI4463PhyBridge.h"
#include <string.h>         // memset   
#include "SimplexTransferSI4463Config.h"

// -----------------------------------------------------------------------------
/**
 *  Defines, enumerations, and structure definitions
 */

#if defined( SI4463_MODULE )
#define PHYINFO                             
#define PHYINFO_CAST(phyInfo)               
#define A1101GetLookup(entry)               A110LR09GetLookup(entry)
#define A1101GetLookupSize()                A110LR09GetLookupSize()
#define A1101Init(phyInfo, spi, gdo)\
  A110LR09Init(phyInfo, spi, gdo)
#define A1101Configure(phyInfo, config)\
  A110LR09Configure(phyInfo, config)
#define A1101Wakeup(phyInfo)                A110LR09Wakeup(phyInfo)
#define A1101SetPktctrl1(phyInfo, pktctrl1) A110LR09SetPktctrl1(phyInfo, pktctrl1)
#define A1101SetMcsm0(phyInfo, mcsm0)       A110LR09SetMcsm0(phyInfo, mcsm0)
#define A1101SetAddr(phyInfo, addr)         A110LR09SetAddr(phyInfo, addr)
#define A1101SetChannr(phyInfo, channr)     A110LR09SetChannr(phyInfo, channr)
#define A1101SetPaTable(phyInfo, power)     A110LR09SetPaTable(phyInfo, power)
#define A1101GetRssiDbm(phyInfo)            A110LR09GetRssiDbm(phyInfo);
#define A1101ConvertRssiToDbm(phyInfo, rssi)\
  A110LR09ConvertRssiToDbm(phyInfo, rssi)    
#endif

// Note: This is used to calculate SYNC word timeout. It is common for all
// supported A110X/2500-based platforms.
#define PHY_PREAMBLE_LENGTH   4             // Length of preamble in bytes
#define PHY_SYNC_LENGTH       4             // Length of sync in bytes

// -----------------------------------------------------------------------------
/**
 *  Global data
 */





// CC1101 GDOx interface


#if defined( A1101R04_MODULE )
static struct sA1101R04PhyInfo gA1101r04PhyInfo;
static struct sA1101R04PhyInfo *gPhyInfo = &gA1101r04PhyInfo;
#elif defined( A1101R08_MODULE )
static struct sA1101R08PhyInfo gA1101r08PhyInfo;
static struct sA1101R08PhyInfo *gPhyInfo = &gA1101r08PhyInfo;
#elif defined( A1101R09_MODULE )
static struct sA1101R09PhyInfo gA1101r09PhyInfo;
static struct sA1101R09PhyInfo *gPhyInfo = &gA1101r09PhyInfo;
#elif defined( A110LR09_MODULE )
//static struct sA110LR09PhyInfo gA110Lr09PhyInfo;
//static struct sA110LR09PhyInfo *gPhyInfo = &gA110Lr09PhyInfo;
#elif defined( A2500R24_MODULE )
static struct sA2500R24PhyInfo gA2500r24PhyInfo;
static struct sA2500R24PhyInfo *gPhyInfo = &gA2500r24PhyInfo;
#endif

// Physical device and associated data stream
static struct sPhyDevice gPhyDevice;

// -----------------------------------------------------------------------------
/**
 *  Private interface
 */

#if defined( PROTOCOL_ENDPOINT ) && defined( PROTOCOL_USE_RX_TIMEOUT )
/**
 *  PhyCalculateRxTimeout - calculate the number of ticks required to register
 *  as an Rx timeout. The value is calculated as follows,
 *
 *      TIME = ((((PREAMBLE + SYNC) * 8) + (PAYLOAD * 8)) * (1 / BAUD)) + OFFSET
 *
 *  The calculation below differs only because time needs to be in milliseconds
 *  and the baud rate is stored along with a scale factor to represent the true
 *  baud rate. For instance, 1.2kBaud is stored as 12 with a scale factor of
 *  100.
 *
 *    @param  baudRate      Scaled baud rate value.
 *    @param  scaleFactor   Baud rate scale factor to recover true baud rate.
 */
void PhyCalculateRxTimeout(unsigned int baudRate, unsigned int scaleFactor)
{
  gPhyDevice.timer.rxTimeout.compare = ((PHY_PREAMBLE_LENGTH + PHY_SYNC_LENGTH) * 8);
  gPhyDevice.timer.rxTimeout.compare = (PHY_MAX_TXFIFO_SIZE * 8);
  gPhyDevice.timer.rxTimeout.compare *= (1000 / baudRate);// Convert time to millisecond ticks
  gPhyDevice.timer.rxTimeout.compare /= scaleFactor;      // Scale the result
  gPhyDevice.timer.rxTimeout.compare += 1;                // Round up to the next timer tick
  
  // Add an offset. Double the compare value and add processing overhead estimate.
  // TODO: Determine an appropriate processing overhead estimate value.
  gPhyDevice.timer.rxTimeout.compare <<= 2;
}
#endif


#if defined( PROTOCOL_ENDPOINT ) && defined( PROTOCOL_USE_RX_TIMEOUT )
/**
 *  PhyTimerEnableRxTimeout - enable and set the SYNC timeout counter.
 */
void PhyTimerEnableRxTimeout()
{
  gPhyDevice.timer.rxTimeout.counter = gPhyDevice.timer.rxTimeout.compare;
  gPhyDevice.timer.rxTimeout.enable = true;
  PhyTimerStart();
}
#endif

#if defined( PROTOCOL_ENDPOINT ) && defined( PROTOCOL_USE_RX_TIMEOUT )
/**
 *  PhyTimerDisableRxTimeout - disable and clear SYNC timeout counter.
 */
void PhyTimerDisableRxTimeout()
{
  PhyTimerStop();
  gPhyDevice.timer.rxTimeout.enable = false;
  gPhyDevice.timer.rxTimeout.counter = 0;
}
#endif

/**
 *  PhyActiveMode - put the Physical hardware into an active state.
 */
void PhyActiveMode(void)
{
  PROTOCOL_CRITICAL_SECTION(A1101Wakeup(PHYINFO_CAST(gPhyDevice.phyInfo)));
}

/**
 *  PhyDataStreamBuild - build a data stream. Populate header, data field, and
 *  footer information. Write to the TX FIFO of the physical hardware.
 *
 *    @param  phyInfo     Physical information structure.
 *    @param  dataField   Datafield being encapsulated inside a data stream.
 *    @param  length      Length of the datafield in bytes.
 */
void PhyDataStreamBuild(struct sCC1101PhyInfo *phyInfo, 
                        unsigned char *dataField, 
                        unsigned char length)
{       
  /**
   *  Note: The length of the data stream is the address and the data field. 
   *  Length does not include itself into the total! The address is required
   *  as this physical implementation uses this for filtering. Broadcast
   *  addresses may be used at any time.
   */
  gPhyDevice.stream.header.length = length;
  gPhyDevice.stream.dataField = dataField;

  // Flush the TX FIFO before writing any new data to it.
  CC1101FlushTxFifo(phyInfo);  
      
  // Write the length field to the TX FIFO.
  CC1101WriteTxFifo(phyInfo, 
                    &gPhyDevice.stream.header.length,
                    1);
  // Write the address and data field to the TX FIFO.
  CC1101WriteTxFifo(phyInfo,
                    gPhyDevice.stream.dataField,
                    gPhyDevice.stream.header.length);
}

/**
 *  PhyGetDataStream - strip off the Physical header/footer information and
 *  retrieve the data field.
 */
void PhyGetDataStream(unsigned char *buf, int len)
{

  gPhyDevice.stream.header.length=len;
  
  if(len<=PROTOCOL_DATASTREAM_MAX_SIZE)
  {
    for(int i=0;i<len;i++)
    {
      gPhyDevice.stream.dataField[i]=buf[i];
    } 
    gPhyDevice.stream.footer.rssi=0x00;
    gPhyDevice.stream.footer.status=0x00;
    //signed char rssi = gPhyDevice.stream.footer.rssi;
    //gPhyDevice.stream.footer.rssi = (signed int)(A1101ConvertRssiToDbm(phyInfo, rssi) + 1) >> 1;
    gPhyDevice.status.DataStreamAvailable(gPhyDevice.stream.dataField, 
                                                              gPhyDevice.stream.header.length);
  }
  else
  {
    gPhyDevice.stream.header.length = 0;
  }
}

void PhySendDataStreamIsOver()
{
  gPhyDevice.status.transmitting = false;  
  gPhyDevice.status.DataStreamSent();  
}



// -----------------------------------------------------------------------------
/**
 *  Public interface
 */

bool PhyInit(unsigned char(*DataStreamSent)(void), 
             unsigned char(*DataStreamAvailable)(unsigned char *dataField, unsigned char length))
{ 
  //PHYINFO phyInfo;
  
  // Initialize the physical bridge device and data stream structures.
  gPhyDevice.phyInfo = NULL;
  gPhyDevice.status.transmitting = false;
  gPhyDevice.timer.Generic = NULL;

  gPhyDevice.stream.header.length = 0;
  gPhyDevice.stream.dataField = NULL;
  gPhyDevice.stream.footer.rssi = 0;
  gPhyDevice.stream.footer.status = 0;
  
  // Initialize the physical layer structures and hardware.
  //phyInfo = PHYINFO_CAST(gPhyDevice.phyInfo);
  
  #if defined( PROTOCOL_ENDPOINT ) && defined( PROTOCOL_USE_RX_TIMEOUT )
  // Calculate SYNC timeout value.
//  PhyCalculateRxTimeout(phyInfo->module.lookup->baudRate.value,
//                          phyInfo->module.lookup->baudRate.scaleFactor);
  #endif
  
  // Register a callback routine for the upper layer when sending data stream
  // completes.
  if (DataStreamSent != NULL)
  {
    gPhyDevice.status.DataStreamSent = DataStreamSent;
  }
  
  // Register a callback routine for the upper layer when the data stream
  // becomes available.
  if (DataStreamAvailable != NULL)
  {
    gPhyDevice.status.DataStreamAvailable = DataStreamAvailable;
  }
  
//  if (!A1101Init(phyInfo, &gA1101Spi, gA1101Gdo))
//  {
//    return false;
//  }

  // Set the default local device address to broadcast.
//  A1101SetAddr(phyInfo, 0x00);
  
  return true;
}

void PhyEnable()
{
  //CC1101GdoEnable(PHYINFO_CAST(gPhyDevice.phyInfo)->cc1101.gdo[0]);
}

void PhyDisable()
{
  //CC1101GdoDisable(PHYINFO_CAST(gPhyDevice.phyInfo)->cc1101.gdo[0]);
}

// -----------------------------------------------------------------------------
// Physical configuration

bool PhyConfigure(unsigned char config)
{
  return true;
}

void PhyEnableAddressFilter(unsigned char deviceAddr)
{
  
}

void PhyDisableAddressFilter()
{  
}

bool PhySetChannel(unsigned char channel)
{
  // Set physical hardware to an active state.
  
}

void PhySetOutputPower(tPower power)
{

}

// -----------------------------------------------------------------------------
// Physical status

// TODO: Determine correct implementation for this function. The current version
// does not take into account issues with timing (when RSSI is valid).
//tPower PhyGetInstantSignalStrength()
//{
//  PHYINFO phyInfo = PHYINFO_CAST(gPhyDevice.phyInfo);
//  tPower dBm;
//  
//  // Set physical hardware to an active state.
//  PhyActiveMode();
//  
//  // Put the radio into receive and read the instantaneous RSSI value.
//  CC1101ReceiverOn(&phyInfo->cc1101);
//  
//  // TODO: Implement averaging of the RSSI value.
//  dBm = A1101R09GetRssiDbm(phyInfo, 0);
//  
//  CC1101Idle(&phyInfo->cc1101);
//  
//  return dBm;
//}

struct sPhyDataStreamFooter* PhyGetDataStreamStatus()
{
  return &gPhyDevice.stream.footer;
}

// -----------------------------------------------------------------------------
// Physical operation
               
void PhyIdle()
{

}

void PhyCalibrate()
{

}

void PhyReceiverOn(unsigned char *dataField)
{  
  // Set the data buffer being used for received data.
  gPhyDevice.stream.dataField = dataField;

  
  #if defined( PROTOCOL_ENDPOINT ) && defined( PROTOCOL_USE_RX_TIMEOUT )
  // Start the timeout timer.
  /**
   *  Timeout = ((preamble length + sync length) * 8) * (1 / baud)
   *  Note: The preamble and sync lengths are in bytes. They must be converted 
   *  to bits (by multiplying by 8).
   */
  //PhyTimerEnableRxTimeout();
  #endif
}

bool PhyTransmit(unsigned char *dataField, 
                 unsigned char count)
{
  if (count > PROTOCOL_FRAME_MAX_PAYLOAD_LENGTH)
  {
    /**
     *  Error: data stream is too large to transmit. This implementation does 
     *  not support fragmentation of the data stream. Size must be less than or 
     *  equal to the physical hardware TX FIFO.
     */
    return false;
  }

  if (!gPhyDevice.status.transmitting)
  {
    // Build the data stream and write to the TX FIFO.
    //PhyDataStreamBuild(&phyInfo->cc1101, dataField, count);
    //发送数据写入FIFO
    
    /**
     *  Set the flag before strobing the radio to transmit. This will prevent 
     *  any state issues from occurring if short data streams are being sent and 
     *  the radio finishes prior to setting this flag.
     */
    gPhyDevice.status.transmitting = true;
    vRadio_StartTx_Variable_Packet(0u,dataField,count);
    //CC1101Transmit(&phyInfo->cc1101);
    //开始发送
    
    return true;
  }
  
  // Unable to transmit the message. There may be a transmit operation currently
  // being performed.
  return false;
}

void PhyLowPowerMode()
{
  //PROTOCOL_CRITICAL_SECTION(CC1101Sleep(&PHYINFO_CAST(gPhyDevice.phyInfo)->cc1101));
}

void PhyTimerInit(unsigned char(*GenericTimer)(void))
{
  gPhyDevice.timer.running = false;
  gPhyDevice.timer.Generic = GenericTimer;
  
  //A110x2500HwTimerInit();
  //计时器初始化；
}

void PhyTimerStart()
{
  if (!gPhyDevice.timer.running)
  {
    PROTOCOL_CRITICAL_SECTION
    (
      //A110x2500HwTimerStart();
      //计时器开始计时；
      gPhyDevice.timer.running = true
    );
  }
}

void PhyTimerStop()
{
  if (gPhyDevice.timer.running)
  {
    PROTOCOL_CRITICAL_SECTION
    (
      // Clear any currently pending timer interrupts and stop counting.
      //A110x2500HwTimerStop();
     //计时器停止计时；
      gPhyDevice.timer.running = false
    );
  }
}

#if defined( PROTOCOL_ENDPOINT ) && defined( PROTOCOL_USE_RX_TIMEOUT )
void PhySyncTimerInit(unsigned char(*RxTimeout)(void))
{
  gPhyDevice.timer.rxTimeout.enable = false;
  gPhyDevice.timer.rxTimeout.counter = 0;
  gPhyDevice.timer.rxTimeout.RxTimeout = RxTimeout;
}
#endif

unsigned char PhySyncEopIsr(volatile unsigned char event)
{
  unsigned char statusMessage = 0;          // Message from callback routine
    
  // Is the RF event occuring due to reception or transmission completion?
  if (gPhyDevice.status.transmitting)
  {
    /**
    *  Note: GDO0 is issued prior to the transmitter being completely
    *  finished. The state machine will remain in TX_END until transmission
    *  completes. The following waits for TX_END to correct the hardware
    *  behavior.
    */   
    
    // Transmitting data stream has completed.
    gPhyDevice.status.transmitting = false;
    statusMessage = gPhyDevice.status.DataStreamSent();
  }
  else
  {
    // Receiving data stream has completed; read it from the RX FIFO.    
    //PhyGetDataStream();
    statusMessage = gPhyDevice.status.DataStreamAvailable(gPhyDevice.stream.dataField, 
                                                              gPhyDevice.stream.header.length);
  }
  return statusMessage;
}



unsigned char PhyTimerIsr()
{
  #if defined( PROTOCOL_ENDPOINT ) && defined( PROTOCOL_USE_RX_TIMEOUT )
  // If enabled, service the sync timeout timer.
  if (gPhyDevice.timer.rxTimeout.enable)
  {
    if (--gPhyDevice.timer.rxTimeout.counter == 0)
    {
      // Disable sync timeout counter.
      PhyTimerDisableRxTimeout();
      if (gPhyDevice.timer.rxTimeout.RxTimeout != NULL)
      {
        return gPhyDevice.timer.rxTimeout.RxTimeout();
      }
    }
  }
  #endif
  
  // TODO: Determine if this is the correct location for this call. Can we
  // enable global interrupts earlier? Should we enable them later?
  
  // Service the generic timer.
  if (gPhyDevice.timer.Generic != NULL)
  {
    return gPhyDevice.timer.Generic();
  }
  
  return 0;
}

// -----------------------------------------------------------------------------
/**
 *  Test stub - test functionality of A1101-based physical bridge.
 */

/**
 *  To test this module, define the following in your compiler preprocessor
 *  definitions: "TEST_A1101_PHY_BRIDGE".
 *
 *  It is strongly suggested that you leave the test stub in this source file.
 *  This stub will allow you to easily test your implementation using unit tests 
 *  defined and by adding more to suit your application needs.
 */
#ifdef TEST_A1101_PHY_BRIDGE

/**
 *  Test Example - test the functionality of the A1101-based physical bridge.
 *
 *  @version    1.0.00
 *  @date       17 Oct 2012
 *  @author     BPB, air@anaren.com
 *  @platform   Texas Instruments MSP430 (MSP430G2553/LaunchPad Dev Kit)
 *  @compiler   IAR C/C++ Compiler for MSP430
 *              5.50.1 (5.50.1.50465)
 *
 *  assumptions
 *  ===========
 *  none
 *
 *  file dependency
 *  ===============
 *  intrinsics.h : processor intrinsic functions
 *  - This file also depend on the compiler that is listed above.
 *  msp430g2553.h : processor register definitions
 *
 *  revision history
 *  ================
 *  ver 1.0.00 : 02 Oct 2012
 *  - initial release
 */
// msp430 intrinsic functions (compiler specific).
#include "intrinsics.h"
// msp430g2553 peripheral and register definitions.
#include "msp430g2553.h"

// -----------------------------------------------------------------------------

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;
  
  while (true)
  {
    // TODO: Implement a test example for the A1101 physical bridge module.
    __no_operation();
  }
}


#endif  /* TEST_A1101_PHY_BRIDGE */
