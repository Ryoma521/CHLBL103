/**
 *  ----------------------------------------------------------------------------
 *  Copyright (c) 2012-13, Anaren Microwave, Inc.
 *
 *  For more information on licensing, please see Anaren Microwave, Inc's
 *  end user software licensing agreement: EULA.txt.
 *
 *  ----------------------------------------------------------------------------
 *
 *  SimplexTransfer.c - performs a simplex transfer using the SimpleTransfer()
 *  API call. For each message sent, a packet sequence number is incremented.
 *  
 *
 *  @version    1.0.00
 *  @date       04 Feb 2013
 *  @author     BPB, air@anaren.com
 *
 *  assumptions
 *  ===========
 *  - this is being compiled exclusively with End Point node(s).
 *  
 *  file dependency
 *  ===============
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

// -----------------------------------------------------------------------------
/**
 *  Defines, enumerations, and structure definitions
 */

#define ST(X) do { X } while (0)

/**
 *  Abstract hardware based on the supported example platform being used.
 * 
 *  Currently supported platforms:
 *    - TI MSP430G2553 + AIR A110x2500 Booster Pack (BPEXP430G2x53.c)
 */
#if defined( __MSP430G2553__ )
#define HardwareInit()\
  ST\
  (\
    WDTCTL = WDTPW | WDTHOLD;\
    BCSCTL1 = CALBC1_8MHZ;\
    DCOCTL = CALDCO_8MHZ;\
  )
#define McuSleep()    _BIS_SR(LPM4_bits | GIE)  // Go to low power mode 4
#define McuWakeup()   _BIC_SR(LPM4_EXIT);       // Wake up from low power mode 4
#define GDO0_VECTOR   PORT2_VECTOR
#define GDO0_EVENT    P2IFG
#endif

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
 *  Global data
 */

/**
 *  The following instance of sProtocolSetupInfo is used to initialize the
 *  protocol with required parameters. The End Point node role has the following
 *  parameters [optional parameters are marked with an asterix (*)].
 *        
 *  { channel_list, pan_id, local_address, Backup(*), TransferComplete(*) }
 *
 *  Note: Parameters marked with the asterix (*) may be assigned "NULL" if they
 *  are not needed.
 */
static const struct sProtocolSetupInfo gProtocolSetupInfo = {
  { PROTOCOL_CHANNEL_LIST },// Physical channel list
  { 0x01,0x01 },                 // Physical address PAN identifier
  { 0x02,0x02,0x03,0x04},                 // Physical address
  NULL,                     // Protocol Backup callback (not used)
  NULL                      // Protocol Data Transfer Complete callback (not used)
};

static struct sPacket gPacket = {
  0x00,                     // Set the initial sequence number value to 0
  "Hello"                   // Set the initial payload to a "Hello" string
};

// -----------------------------------------------------------------------------

/**
 *  PlatformInit - sets up platform and protocol hardware. Also configures the
 *  protocol using the setup structure data.
 *
 *    @return   Success of the operation. If true, the protocol has been setup
 *              successfully. If false, an error has occurred during protocol
 *              setup.
 */
bool PlatformInit(void)
{
  // Disable global interrupts during hardware initialization to prevent any
  // unwanted interrupts from occurring.
  MCU_DISABLE_INTERRUPT();
  
  // Setup basic platform hardware (e.g. watchdog, clocks).
  HardwareInit();
  
  // Attempt to initialize protocol hardware and information using the provided
  // setup structure data.
  if (!ProtocolInit(&gProtocolSetupInfo))
  {
    return false;
  }
  
  // Re-enable global interrupts for normal operation.
  MCU_ENABLE_INTERRUPT();
  
  return true;
}

<<<<<<< HEAD
void RTC_Configuration(void)
{
  /* RTC clock source configuration ------------------------------------------*/
  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  /* Reset Backup Domain */
  BKP_DeInit();
  
  /* Enable the LSI OSC */
  //RCC_LSIConfig(RCC_LSI_ON);
  RCC_LSICmd(ENABLE);
  /* Wait till LSE is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* RTC configuration -------------------------------------------------------*/
  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

  /* Set the RTC time base to 1s */
  //RTC_SetPrescaler(32767);  
  RTC_SetPrescaler(40000);  
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();

  /* Enable the RTC Alarm interrupt */
  RTC_ITConfig(RTC_IT_ALR, ENABLE);
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
}


void EnterIntoStopMode(void)
{
  USART2_Config_After_Stop();
  
  /* Enable USART2 Receive and Transmit interrupts */
  USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);  // 开启 串口空闲IDEL 中断
  
  RCC_ClocksTypeDef RCC_ClockFreqx;
  ClkSwitch2HsiSystemInit();
  RCC_GetClocksFreq(&RCC_ClockFreqx);
  
  if (SysTick_Config(RCC_ClockFreqx.HCLK_Frequency/1000))
  { 
    /* Capture error */ 
    while (1);
  }
  
  si446x_shutdown();
  DelayMs(5000);

//   
//  /* Enable the USART2 */
//  USART_Cmd(USART2, DISABLE);  // 开启串口
//  /* Enable USARTy DMA TX request */
//  USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);  // 开启串口DMA发送
//  USART_DMACmd(USART2, USART_DMAReq_Rx, DISABLE); 
//  USART_ClearFlag(USART2, USART_IT_IDLE);
//  USART_ClearFlag(USART2, USART_DMAReq_Tx);
//  USART_ClearFlag(USART2, USART_DMAReq_Rx);
    
  SI4463_Disable_NIRQ_Int();
//  Il_Hw_LowPower();
//  LowPower_SI4463_Pin();

  //PWR_EnterSTANDBYMode(); 
  //Uart_Init();
    
  //USART_WakeUpConfig(USART2,USART_WakeUp_IdleLine);//静默模式设置 1、USART_WakeUp_IdleLine 空闲总线唤醒//2、USART_WakeUp_AddressMark地址标记唤醒
  //USART_SetAddress(USART2, 0xAD);           //设置地址从机1
  //USART_ReceiverWakeUpCmd(USART2,ENABLE);          //使能接收唤醒
  
//  /* Disable the SysTick timer */
//  SysTick->CTRL &= (~SysTick_CTRL_ENABLE);
//
//  /* Enable Power Interface clock */
//  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
//	
//  /* configure MCU to go in stop/standby mode after WFI instruction and not in sleep */
//  SCB->SCR |= SCB_SCR_SLEEPDEEP;
//	
//  /* configure MCU to go in stop mode after WFI instruction and not in standby */
//  PWR->CR &= (~PWR_CR_PDDS);
//  
//  /* Configure MCU to go in stop mode with regulator in low power mode */
//  PWR->CR |= PWR_CR_LPDS;
//
//  /* Disable VREFINT to save current */
//  PWR->CR |= PWR_CR_DBP;
//
//  /* Disable PVDE to save current */
//  PWR->CR &= (~PWR_CR_PVDE);
//
//  /* Wait for interrupt instruction, device go to sleep mode */
//  __WFI();  
  
  /* Enable PWR and BKP clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  
  /* Configure RTC clock source and prescaler */
  RTC_Configuration();
  
  RCC_HSICmd(DISABLE);
  RCC_HSEConfig(RCC_HSE_OFF);
  RCC_LSEConfig(RCC_LSE_OFF);

  Il_Hw_LowPower();
  LedD3Off(); 
  LedD4Off();     
  LowPower_SI4463_Pin();
  
  USART2_Config();
  //while(1);
  
  PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);

  if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
  {
    /* Clear Wake Up flag */
    PWR_ClearFlag(PWR_FLAG_WU);
  }
}


=======
>>>>>>> parent of 302dcd7... E: Low Power Version
/**
 *  main - main application loop. Sets up platform and then performs simple
 *  transfers (simplex) while incrementing the sequence number for the lifetime 
 *  of execution.
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
  
  vRadio_Init(); 
  
//TimingBaseInit(50000);
  
//  while(1)
//  {
////  //LedD4StaInvert();  
////    GPIOB->BSRR  = 0x00000040;
////    GPIOB->BRR  = 0x00000040;
//  }
  
  //EXTILine_TimingSync_Config();
  
  //si446x_get_int_status(0u, 0u, 0u);
  uint8_t testBuff[64]={0x05,0xD0,0x01,0x02,0x03,0xD6,};
  
  //UartSendByte(testBuff, 6); 
  
  vRadio_StartRX(pRadioConfiguration->Radio_ChannelNumber);
  
  RadioGotoRxSta();
  
  SI4463_Enable_NIRQ_Int();

  if (!ProtocolInit(&gProtocolSetupInfo))
  {
    return false;
  }
  
  while(true)
  {
//    while(1)
//    {
//      vSampleCode_SendFixPacket(testBuff,64);
//      DelayMs(5);
//    }
    
    if(GetPubRxBufCount()>0x00)
    {
      //UartSendByte(timMark, 1);
      //UartSendByte(uartRxDataBuff, uartRxCount);
      //uartDataProcess();
      PubRxDataProcess();
    }  
    
    if(GetPubTxBufCount()>0x00)
    {
      PubTxDataProcess();
    }  
    
    if (!ProtocolBusy())
    {
      // Increment the sequence number for the next transmission.
      gPacket.seqNum++;
<<<<<<< HEAD
    } 
    
    if(GetPubTxBufCount()==0x00&&GetPubRxBufCount()==0x00&&GetUsartDmaTxSta()==IDLE)
    {
      EnterIntoStopMode();
      
      Init_SI4463_Pin();
      vRadio_Init(); 
      vRadio_StartRX(pRadioConfiguration->Radio_ChannelNumber);
      RadioGotoRxSta();
      SI4463_Enable_NIRQ_Int();
=======
>>>>>>> parent of 302dcd7... E: Low Power Version
    }
  }
  
//  while (true)
//  {
//    // Perform a simple transfer of the packet.
//    if (!ProtocolSimpleTransfer((unsigned char*)&gPacket, sizeof(struct sPacket)))
//    {
//      // Put the microcontroller into a low power state (sleep). Remain here
//      // until the ISR wakes up the processor.
//      //McuSleep();
//    }
//    
//    /**
//     *  Check if the protocol is busy. If it is, a new transfer cannot occur
//     *  until it becomes ready for the next instruction. Do not increment the
//     *  sequence number until the protocol is ready. This prevents incrementing
//     *  the sequence number more than once between transmissions.
//     */
//    if (!ProtocolBusy())
//    {
//      // Increment the sequence number for the next transmission.
//      gPacket.seqNum++;
//    }
//  }
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
  register volatile unsigned char event;// = GDO0_EVENT;
  
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

  // Wake up the microcontroller to continue normal operation upon exiting the
  // ISR.
  McuWakeup();
}

// Note: No hardware timer interrupt required for this example because the 
// End Point node does not perform half duplex transfers.
