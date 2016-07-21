/**/
/**/
#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>
#include <stdlib.h>

#include "radio_mcu.h"
#include "radio.h"
#include "macro.h"
#include "si4463.h"
#include "si4463_def.h"
#include "radio_config.h"
#include "radio_comm.h"
#include "node.h" 

#include "uartapp.h"
#include "timeapp.h"


uint8_t uartRxPacketHead=0x00;
uint16_t uartRxPacketLength=0x00;
uint8_t uartRxLenCount=0x00;
uint16_t uartRxCount=0x0000;
uint8_t* uartRxDataBuff=NULL;

uint8_t uartRxFlag=0x00; //0: no data received, 1: data received
uint8_t uartRxDataCheckFail=0x00; // 1: check fail;
uint8_t SdkLenVer[]={0x00,0x00};

struct PxUsartBuf pub_rx, pub_tx;
uint8_t DmaUartTxBuf[UartFrameMaxLen];
uint8_t DmaUartRxBuf[UartFrameMaxLen];

enum UsartDmaTxSta usart_dma_tx_sta=IDLE;
extern SEGMENT_VARIABLE_SEGMENT_POINTER(pRadioConfiguration, tRadioConfiguration) ;


void uartRxData(USART_TypeDef* USARTx)
{
  if(uartRxFlag==0x00)
  {
    if(uartRxPacketHead==0xAA) //Packet head is received
    {
      if(uartRxLenCount<2)
      {       
        if(uartRxLenCount<1)
        {
          uartRxCount++;
          uartRxLenCount++;
          SdkLenVer[0]=USART_ReceiveData(USARTx);     
        }
        else
        {
          if(uartRxLenCount<2)
          {
            uartRxCount++;
            uartRxLenCount++;
            SdkLenVer[1]=USART_ReceiveData(USARTx); 
            
            uartRxPacketLength=SdkLenVer[1];            
            uartRxPacketLength=(uartRxPacketLength<<8)+SdkLenVer[0];            
            
            if(uartRxPacketLength>0x0000&&uartRxPacketLength<=0x00FF)
            {
              uartRxDataBuff=(uint8_t*)malloc(uartRxPacketLength+6);
              uartRxCount+=5;
              *uartRxDataBuff=((uartRxPacketLength+6-2)>>8);
              *(uartRxDataBuff+1)=(uartRxPacketLength+6-2);
              *(uartRxDataBuff+2)=0xD3;
              *(uartRxDataBuff+3)=0x00;
              *(uartRxDataBuff+4)=PX_NUM;
              *(uartRxDataBuff+5)=0xAA;
              *(uartRxDataBuff+6)=SdkLenVer[0];
              *(uartRxDataBuff+7)=SdkLenVer[1];
            }
            else
            {
              uartRxPacketHead=0x00;
            }          
          }        
        }
      }
      else
      {
        if(uartRxCount<uartRxPacketLength+5)
        {
          *(uartRxDataBuff+uartRxCount)=USART_ReceiveData(USARTx);
          uartRxCount++;
          if(uartRxCount==(uartRxPacketLength+5))
          {
            U8 CheckByte=0x00;
            for(int ii=2;ii<uartRxCount;ii++)
            {
              CheckByte+=*(uartRxDataBuff+ii);          
            }
            *(uartRxDataBuff+uartRxCount)=(CheckByte-3); //0xD3 is not 0xD0;
            uartRxCount++;            
            uartRxFlag=0x01;  
            
//            UartSendByte(uartRxDataBuff, uartRxCount);
//            uartRxReset();
//            uartRxFlag=0x00; 
          }
        }
        else
        {
         uartRxReset();
        }
      }      
    }
    else
    {
      if(USART_ReceiveData(USARTx)==0xAA)
      {
        uartRxCount++;
        uartRxPacketHead=USART_ReceiveData(USARTx);
        uartRxDataCheckFail=0x00;
      }
      else
      {      
        uartRxPacketHead=0x00;       
      }
    }
  }
  else
  {
    USART_ReceiveData(USARTx);
  } 
}

void uartRxReset(void)
{
  uartRxPacketHead=0x00;
  uartRxPacketLength=0x00;
  if(uartRxDataBuff!=NULL)
  {
    //U8 ErrFlag[]={0xEE,0xF4};
    //UartSendByte(ErrFlag, 2); 
    free(uartRxDataBuff);
  }
  uartRxCount=0x00;
  uartRxFlag=0x00;
  uartRxLenCount=0x00;
}

uint8_t uartRxDataCheck(uint8_t* dataBuf, uint16_t dataLen)
{
  uint16_t tmpDataLen=(*(dataBuf+2));
  tmpDataLen=(tmpDataLen<<8)+(*(dataBuf+3));
  if(dataLen==tmpDataLen)
  {
    return 0x01;
  }
  else
  {
    return 0x00;
  }
  
  uint8_t tmp=0x00;
  for(uint16_t i=0;i<dataLen-1;i++) //include ctrlword;
  {
    tmp+=(*(dataBuf+i+2)); //check from ctrlword to data;
  }
  if(tmp==*(dataBuf+dataLen+1))
  {
    return 0x01;
  }
  else
  {
    return 0x00;
  }

}

uint8_t PxRfRxDataCheck(uint8_t* dataBuf, uint16_t dataLen)
{
  uint8_t tmp=0x00;
  for(uint16_t i=0;i<dataLen-1;i++) //include ctrlword;
  {
    tmp+=(*(dataBuf+i)); //check from ctrlword to data;
  }
  if(tmp==*(dataBuf+dataLen-1))
  {
    return 0x01;
  }
  else
  {
    return 0x00;
  }

}

extern uint8 PKT_Sent_Flag;

void uartDataProcess(void)
{  
  U16 uDataLen=uartRxDataBuff[0];
  uDataLen=(uDataLen<<8)+uartRxDataBuff[1];
  //UartSendByte(uartRxDataBuff, 2);
  
  if(uDataLen>0x00FF)
  {
    U8 ErrFlag[]={0xEE,0x01};
    UartSendByte(&ErrFlag, 2);
    uartRxReset();
    return;
  }  

  switch(uartRxDataBuff[2])
  {
  case 0xC0:
    uartRxReset();
    //back C1
    break;
  case 0xC2:
    uartRxReset();
    //back C3
    break;
  case 0xD0:
    //back D1
    //send data by RF
    //vRadio_SlotStartTx_Variable_Packet(0u,uartRxDataBuff[0]+1);    
    vRadio_StartTx_Variable_Packet(0u,uartRxDataBuff,uDataLen+2);
    while(PKT_Sent_Flag)
    {  
      
    }
    uartRxReset();
    break;
  case 0xD3:
    vRadio_StartTx_Variable_Packet(0u,uartRxDataBuff,uDataLen+2);
    while(PKT_Sent_Flag)
    {  
      
    }
    uartRxReset();
    break;
  default:
    uartRxReset();
    break;
  }  
}

void PubRxDataProcess(void)
{ 
  uint16_t length=pub_rx.Buf[0][0]; 
  length=(length<<8)+pub_rx.Buf[0][1];   
  
  if(length>=0x0002&&length<UartFrameMaxLen)
  {    
    switch(pub_rx.Buf[0][2])
    {
    case 0xC0:
      //back C1
      QueryResp();
      pub_rx.BufCount--;
      break;
    case 0xC2:
      //back C3
      CfgAndResp();    
      pub_rx.BufCount--;
      break;
    case 0xD0:
      //back D1
      //send data by RF
      if (!ProtocolSimpleTransfer((unsigned char*)&(pub_rx.Buf[0][3]), length-2)) //ȥ������Э��Packet Length��Ctrl Word�� Address��Check��
      {
        // Put the microcontroller into a low power state (sleep). Remain here
        // until the ISR wakes up the processor.
        //McuSleep();
      }
      //vRadio_SlotStartTx_Variable_Packet(0u,uartRxDataBuff[0]+1);
      //vRadio_StartTx_Variable_Packet(0u,uartRxDataBuff,uDataLen+2);
      break;
    case 0xD3:
      //vRadio_StartTx_Variable_Packet(0u,pub_rx.Buf[0],length+6);
      break;
    default:
      break;
    }   
  } 
}

void PubTxDataProcess(void)
{ 
  if(usart_dma_tx_sta==IDLE)
  {
    uint16_t length=pub_tx.Buf[0][2]; 
    length=(length<<8)+pub_tx.Buf[0][3]+4; 
    if(length<UartFrameMaxLen)
    {
      PubTx2DmaTxBuf(length);
      LumMod_Uart_Start_DMA_Tx(length);
    }
  }
}


void PubRxShift(void)
{     
 if(pub_rx.BufCount>0&&pub_rx.BufCount<=PxUsartBufLen)
  {
    /* Enable USART2 Receive and Transmit interrupts */
    USART_ITConfig(LUMMOD_UART, USART_IT_IDLE, DISABLE);  // ���� ���ڿ���IDEL �ж�
    for(int i=0;i<pub_rx.BufCount;i++)
    {
      for(int j=0;j<UartFrameMaxLen;j++)
      {
        pub_rx.Buf[i][j]=pub_rx.Buf[i+1][j];      
      }
    }
    pub_rx.BufCount--;
    USART_ITConfig(LUMMOD_UART, USART_IT_IDLE, ENABLE);  // ���� ���ڿ���IDEL �ж� 
  }
  
}

void PubTxShift(void)
{
 
 if(pub_tx.BufCount>0&&pub_tx.BufCount<=PxUsartBufLen)
  {
    SI4463_Disable_NIRQ_Int(); 
    for(int i=0;i<pub_tx.BufCount;i++)
    {
      for(int j=0;j<UartFrameMaxLen;j++)
      {
        pub_tx.Buf[i][j]=pub_tx.Buf[i+1][j];      
      }
    }
    pub_tx.BufCount--;
    SI4463_Enable_NIRQ_Int() ;
  }

}

void buff2Packet(struct uartRxPacket* pkt)
{  
  (*pkt).packetLength=uartRxPacketLength;
  (*pkt).ctrlWord=*uartRxDataBuff;
  (*pkt).addr[0]=*(uartRxDataBuff+1);
  (*pkt).addr[1]=*(uartRxDataBuff+2);
  if(uartRxPacketLength>4)
  {
    for(int i=0;i<uartRxPacketLength-4;i++)
    {
      *((*pkt).data+i)=*(uartRxDataBuff+3+i);
    }
  }
  (*pkt).verifyByte=*(uartRxDataBuff+uartRxPacketLength-1);
  uartRxReset();
}


void PubRxTxInit(void)
{
  pub_rx.BufCount=0;
  pub_tx.BufCount=0;
}

void Uart_Init(void)
{
  PubRxTxInit();
  
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  /* System Clocks Configuration */
//= System Clocks Configuration ====================================================================//
   
  /* Enable GPIO clock */
  //RCC_APB2PeriphClockCmd(LUMMOD_UART_GPIO_CLK ,  ENABLE ); // ������������IO�˿ڵ�ʱ��
  /* Enable USART Clock */
  //RCC_APB1PeriphClockCmd(LUMMOD_UART_CLK, ENABLE); // ��ʼ����ʱ��
  
   
//=NVIC_Configuration==============================================================================//
 
  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
  
  /* Enable the DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = LUMMOD_UART_Tx_DMA_IRQ;   // ����DMAͨ�����ж�����
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // ���ȼ�����
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USART Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = LUMMOD_UART_IRQn;     // �����ж�����
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
   
//=GPIO_Configuration==============================================================================//
 
    //GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);  // ������û����Ĭ��IO�ڣ����Խ���������ӳ�䣬������Ը����Լ���Ӳ���������ѡ��
   
//    /* Configure USART3 Rx as input floating */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // ���ڽ���IO�ڵ�����
//    GPIO_InitStructure.GPIO_Pin = LUMMOD_UART_RxPin;
//    GPIO_Init(LUMMOD_UART_GPIO, &GPIO_InitStructure);
// 
//    /* Configure USART3 Tx as alternate function push-pull */
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // ���ڷ���IO�ڵ�����
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // �������óɸ�����ʽ���������   
//    GPIO_InitStructure.GPIO_Pin = LUMMOD_UART_TxPin;
//    GPIO_Init(LUMMOD_UART_GPIO, &GPIO_InitStructure);
 
    DMA_Uart_Init();   // ���� DMA ����
 
    /* USART Format configuration ------------------------------------------------------*/
 
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;    // ���ڸ�ʽ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
    /* Configure USART2 */
    USART_InitStructure.USART_BaudRate = 38400;  //  ����������
    //USART_Init(LUMMOD_UART, &USART_InitStructure);
    STM_EVAL_COMInit(COM2, &USART_InitStructure);
    
    /* Enable USART2 Receive and Transmit interrupts */
    USART_ITConfig(LUMMOD_UART, USART_IT_IDLE, ENABLE);  // ���� ���ڿ���IDEL �ж�
   
    /* Enable the USART2 */
    USART_Cmd(LUMMOD_UART, ENABLE);  // ��������
    /* Enable USARTy DMA TX request */
    USART_DMACmd(LUMMOD_UART, USART_DMAReq_Tx, ENABLE);  // ��������DMA����
    USART_DMACmd(LUMMOD_UART, USART_DMAReq_Rx, ENABLE); // ��������DMA����
}

void DMA_Uart_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
   
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // ����DMA1ʱ��
   
   
//=DMA_Configuration==============================================================================//
 
/*--- LUMMOD_UART_Tx_DMA_Channel DMA Config ---*/
 
    DMA_Cmd(LUMMOD_UART_Tx_DMA_Channel, DISABLE);                           // ��DMAͨ��
    DMA_DeInit(LUMMOD_UART_Tx_DMA_Channel);                                 // �ָ�ȱʡֵ
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&LUMMOD_UART->DR);// ���ô��ڷ������ݼĴ���
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)DmaUartTxBuf;         // ���÷��ͻ������׵�ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // ��������λĿ�꣬�ڴ滺���� -> ����Ĵ���
    DMA_InitStructure.DMA_BufferSize = 0;                     // ��Ҫ���͵��ֽ�����������ʵ��������Ϊ0����Ϊ��ʵ��Ҫ���͵�ʱ�򣬻��������ô�ֵ
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // �����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // �ڴ滺������ַ���ӵ���
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // �������ݿ��8λ��1���ֽ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // �ڴ����ݿ��8λ��1���ֽ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // ���δ���ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // ���ȼ�����
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // �ر��ڴ浽�ڴ��DMAģʽ
    DMA_Init(LUMMOD_UART_Tx_DMA_Channel, &DMA_InitStructure);               // д������
    DMA_ClearFlag(LUMMOD_UART_Tx_DMA_FLAG);                                 // ���DMA���б�־
    DMA_Cmd(LUMMOD_UART_Tx_DMA_Channel, DISABLE); // �ر�DMA
    DMA_ITConfig(LUMMOD_UART_Tx_DMA_Channel, DMA_IT_TC, ENABLE);            // ��������DMAͨ���ж�
   
/*--- LUMMOD_UART_Rx_DMA_Channel DMA Config ---*/
 
    DMA_Cmd(LUMMOD_UART_Rx_DMA_Channel, DISABLE);                           // ��DMAͨ��
    DMA_DeInit(LUMMOD_UART_Rx_DMA_Channel);                                 // �ָ�ȱʡֵ
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&LUMMOD_UART->DR);// ���ô��ڽ������ݼĴ���
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)DmaUartRxBuf;         // ���ý��ջ������׵�ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // ��������Ϊ����Դ������Ĵ��� -> �ڴ滺����
    DMA_InitStructure.DMA_BufferSize = UartFrameMaxLen;                     // ��Ҫ�����ܽ��յ����ֽ���
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // �����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // �ڴ滺������ַ���ӵ���
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // �������ݿ��8λ��1���ֽ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // �ڴ����ݿ��8λ��1���ֽ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // ���δ���ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // ���ȼ�����
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // �ر��ڴ浽�ڴ��DMAģʽ
    DMA_Init(LUMMOD_UART_Rx_DMA_Channel, &DMA_InitStructure);               // д������
    DMA_ClearFlag(LUMMOD_UART_Rx_DMA_FLAG);                                 // ���DMA���б�־
    DMA_Cmd(LUMMOD_UART_Rx_DMA_Channel, ENABLE);                            // ��������DMAͨ�����ȴ���������   
}

void USART_IDLE_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)  // �����ж�
    {
        LumMod_Uart_DMA_Rx_Data();
        USART_ReceiveData( USART2 ); // Clear IDLE interrupt flag bit
    }
<<<<<<< HEAD
    //USART_ClearFlag(USART2, USART_IT_IDLE);
    //USART_ClearFlag(USART2, USART_DMAReq_Tx);
    //USART_ClearFlag(USART2, USART_DMAReq_Rx);
=======
>>>>>>> parent of 302dcd7... E: Low Power Version
}

void LumMod_Uart_DMA_Rx_Data(void)
{
    DMA_Cmd(LUMMOD_UART_Rx_DMA_Channel, DISABLE);       // �ر�DMA ����ֹ����
    DMA_ClearFlag( LUMMOD_UART_Rx_DMA_FLAG );           // ��DMA��־λ
    uint16_t DmaUsartRxLen = UartFrameMaxLen - DMA_GetCurrDataCounter(LUMMOD_UART_Rx_DMA_Channel); //��ý��յ����ֽ���
    DmaRxBuf2PubRx(DmaUsartRxLen);
    LUMMOD_UART_Rx_DMA_Channel->CNDTR = UartFrameMaxLen;    //  ���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ
    DMA_Cmd(LUMMOD_UART_Rx_DMA_Channel, ENABLE);        
    /* DMA �������ȴ����ݡ�ע�⣬����жϷ�������֡�����ʺܿ죬MCU����������˴ν��յ������ݣ��ж��ַ������ݵĻ������ﲻ�ܿ������������ݻᱻ���ǡ���2�ַ�ʽ�����
    1. �����¿�������DMAͨ��֮ǰ����LumMod_Rx_Buf��������������ݸ��Ƶ�����һ�������У�Ȼ���ٿ���DMA��Ȼ�����ϴ����Ƴ��������ݡ�
    2. ����˫���壬��LumMod_Uart_DMA_Rx_Data�����У���������DMA_MemoryBaseAddr �Ļ�������ַ����ô�´ν��յ������ݾͻᱣ�浽�µĻ������У������ڱ����ǡ�*/
    //OSMboxPost(mbLumModule_Rx,  LumMod_Rx_Buf); // ���ͽ��յ������ݱ�־����ǰ̨�����ѯ
}

void DmaRxBuf2PubRx(uint16_t PacketLen)
{
  if(PacketLen<=UartFrameMaxLen&&pub_rx.BufCount<PxUsartBufLen&&DmaUartRxBuf[0]==0xAD&&DmaUartRxBuf[1]==0xAD&&uartRxDataCheck(&DmaUartRxBuf[2],PacketLen-4))
  {
    for(int i=0;i<PacketLen;i++)
    {
      pub_rx.Buf[pub_rx.BufCount][i]=DmaUartRxBuf[i+2]; //ȥ��start header 0xAD 0xAD
    }
    pub_rx.BufCount++;
  } 
}

void RfRxData2PubTx(u8 *buf, u32 len)
{
  if(len<=(UartFrameMaxLen-6)&&pub_tx.BufCount<PxUsartBufLen)
  {
    pub_tx.Buf[pub_tx.BufCount][0]=0xAD;
    pub_tx.Buf[pub_tx.BufCount][1]=0xAD;
    
    pub_tx.Buf[pub_tx.BufCount][2]=(U8)((len+2)>>8);
    pub_tx.Buf[pub_tx.BufCount][3]=(U8)(len+2);
    pub_tx.Buf[pub_tx.BufCount][4]=0xD0;

    
    for(int i=0;i<len;i++)
    {
      pub_tx.Buf[pub_tx.BufCount][i+5]=buf[i];
    }
        
    uint8_t tmp=0x00;
    for(uint16_t j=0;j<len+2-1;j++) //include ctrlword;
    {
      tmp+=pub_tx.Buf[pub_tx.BufCount][j+4]; //check from ctrlword to data;
    }
    
    pub_tx.Buf[pub_tx.BufCount][len+5]=tmp;
    
    pub_tx.BufCount++;
  } 
}

void CfgResp2PubTx(unsigned char CfgStatus)
{
    pub_tx.Buf[pub_tx.BufCount][0]=0xAD;
    pub_tx.Buf[pub_tx.BufCount][1]=0xAD;
    
    pub_tx.Buf[pub_tx.BufCount][2]=0x00;
    pub_tx.Buf[pub_tx.BufCount][3]=0x05;
    pub_tx.Buf[pub_tx.BufCount][4]=0xC3;
    pub_tx.Buf[pub_tx.BufCount][5]=0x00; //Res
    pub_tx.Buf[pub_tx.BufCount][6]=0x00; //Res
    pub_tx.Buf[pub_tx.BufCount][7]=CfgStatus; //Res
    
    uint8_t tmp=0x00;
    for(uint16_t j=0;j<5-1;j++) //include ctrlword;
    {
      tmp+=pub_tx.Buf[pub_tx.BufCount][j+4]; //check from ctrlword to data;
    }
    
    pub_tx.Buf[pub_tx.BufCount][8]=tmp;
    
    pub_tx.BufCount++;
}

void QueryResp(void)
{
    pub_tx.Buf[pub_tx.BufCount][0]=0xAD;
    pub_tx.Buf[pub_tx.BufCount][1]=0xAD;
    
    pub_tx.Buf[pub_tx.BufCount][2]=0x00;
    pub_tx.Buf[pub_tx.BufCount][3]=0x0C;
    pub_tx.Buf[pub_tx.BufCount][4]=0xC1;
    pub_tx.Buf[pub_tx.BufCount][5]=0x00; //Res
    pub_tx.Buf[pub_tx.BufCount][6]=0x00; //Res
    pub_tx.Buf[pub_tx.BufCount][7]=PhyGetChannel()/2; //FreqIndex
    pub_tx.Buf[pub_tx.BufCount][8]=PhyGetPowerReg(); //PowerReg
    pub_tx.Buf[pub_tx.BufCount][9]=0x00; //PHY ADDR
    pub_tx.Buf[pub_tx.BufCount][10]=0x00; //PHY ADDR
    pub_tx.Buf[pub_tx.BufCount][11]=0x00; //PHY ADDR
    pub_tx.Buf[pub_tx.BufCount][12]=0x00; //PHY ADDR
    pub_tx.Buf[pub_tx.BufCount][13]=0x00; //PAN ID
    pub_tx.Buf[pub_tx.BufCount][14]=0x00; //PAN ID
    
    uint8_t tmp=0x00;
    for(uint16_t j=0;j<12-1;j++) //include ctrlword;
    {
      tmp+=pub_tx.Buf[pub_tx.BufCount][j+4]; //check from ctrlword to data;
    }
    
    pub_tx.Buf[pub_tx.BufCount][15]=tmp;
    
    pub_tx.BufCount++;
}

void DMA_Channel7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_FLAG_TC7))
    {
        LumMod_Uart_DAM_Tx_Over();
    }
}

void LumMod_Uart_DAM_Tx_Over(void)
{
    DMA_ClearFlag(LUMMOD_UART_Tx_DMA_FLAG);         // �����־
    DMA_Cmd(LUMMOD_UART_Tx_DMA_Channel, DISABLE);   // �ر�DMAͨ��
    usart_dma_tx_sta=IDLE;
    //OSMboxPost(mbLumModule_Tx, (void*)1);           // ���ñ�־λ���������õ���UCOSII �����Ը����Լ�����������޸�

}

void LumMod_Uart_Start_DMA_Tx(uint16_t size)
{
  if(usart_dma_tx_sta==IDLE)
  {  
    usart_dma_tx_sta=BUSY;
    LUMMOD_UART_Tx_DMA_Channel->CNDTR = (uint16_t)size; // ����Ҫ���͵��ֽ���Ŀ
    DMA_Cmd(LUMMOD_UART_Tx_DMA_Channel, ENABLE);        //��ʼDMA����
  }
}

enum UsartDmaTxSta GetUsartDmaTxSta(void)
{
  return usart_dma_tx_sta;
}

void PubTx2DmaTxBuf(uint16_t PacketLen)
{
  if(PacketLen<=UartFrameMaxLen&&pub_tx.BufCount>0&&usart_dma_tx_sta==IDLE)
  {
    for(int i=0;i<PacketLen;i++)
    {
      DmaUartTxBuf[i]=pub_tx.Buf[0][i];
    }
    PubTxShift();
  } 
}

uint16_t GetPubRxBufCount(void)
{
  return pub_rx.BufCount;
}

uint16_t GetPubTxBufCount(void)
{
  return pub_tx.BufCount;
}

void CfgAndResp(void)
{  
  if(PhySetChannel(pub_rx.Buf[0][5])&&PhySetOutputPower(pub_rx.Buf[0][6]))
  {
    //0xAD 0xAD	0x05	0xC3	0x00 0x00	0x00	0xXX
    
    CfgResp2PubTx(0x00);
  }
  else
  {
    CfgResp2PubTx(0x01);
  }     
}
<<<<<<< HEAD


void USART2_Config(void)
{
  EXTI_ClearFlag(EXTI_Line3);
  EXTI_ClearITPendingBit(EXTI_Line3);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_AF
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
    
  EXTI_InitStructure.EXTI_Line    = EXTI_Line3;                 //�ⲿ�ж���
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;        //�ж�ģʽ
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;       //�жϴ�����ʽ
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     //���ж�
  EXTI_Init(&EXTI_InitStructure);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);   //ѡ�� GPIO�ܽ������ⲿ�ж���·

  
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);               // ��ռʽ���ȼ���
  NVIC_InitStructure.NVIC_IRQChannel            = EXTI3_IRQn; //ָ���ж�Դ
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//����USART Rx����MCU
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          // ָ����Ӧ���ȼ���1
  NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
      /* USART Format configuration ------------------------------------------------------*/
 
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;    // ���ڸ�ʽ����
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;
//    USART_InitStructure.USART_Parity = USART_Parity_No;
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
// 
//    /* Configure USART2 */
//    USART_InitStructure.USART_BaudRate = 38400;  //  ����������
//    //USART_Init(LUMMOD_UART, &USART_InitStructure);
//    STM_EVAL_COMInit(COM2, &USART_InitStructure);
//    
//    /* Enable USART2 Receive and Transmit interrupts */
//    USART_ITConfig(LUMMOD_UART, USART_IT_RXNE, ENABLE);  // ���� ���ڿ���IDEL �ж�
//   
//    /* Enable the USART2 */
//    USART_Cmd(LUMMOD_UART, ENABLE);  // ��������

}

void Exit_LowPower_StopMode_ByUSART2(void)
{
//  RCC_ClocksTypeDef RCC_Clocks;
//  SystemInit();
//  //SetHSISysClock();//
//  PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);
//  SetSysClock_AfterWakeUp();  
//  USART_ClearFlag(USART2, USART_FLAG_RXNE);

    
  //SystemInit();
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
  Uart_Init();  
  Il_Hw_Init();
  

}

void USART2_Config_After_Stop(void)//PA3
{
  NVIC_InitTypeDef NVIC_InitStructure;
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);               // ��ռʽ���ȼ���
  NVIC_InitStructure.NVIC_IRQChannel            = EXTI3_IRQn; //ָ���ж�Դ
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//����USART Rx����MCU
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          // ָ����Ӧ���ȼ���1
  NVIC_InitStructure.NVIC_IRQChannelCmd         = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
}
=======
>>>>>>> parent of 302dcd7... E: Low Power Version
