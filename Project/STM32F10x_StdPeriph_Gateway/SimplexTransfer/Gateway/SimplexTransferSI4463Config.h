#ifndef SIMPLEX_TRANSFER_SI4463_CONFIG_H
#define SIMPLEX_TRANSFER_SI4463_CONFIG_H
/**
 *  ----------------------------------------------------------------------------
 *  Copyright (c) 2012-13, Anaren Microwave, Inc.
 *
 *  For more information on licensing, please see Anaren Microwave, Inc's
 *  end user software licensing agreement: EULA.txt.
 *
 *  ----------------------------------------------------------------------------
 *
 *  SimplexTransferLR09Config.h - provides End Point node configuration details 
 *  for the protocol.
 *
 *  @version    1.0.00
 *  @date       16 Jan 2013
 *  @author     BPB, air@anaren.com
 *
 *  Note: This file should be preincluded into the project. Please see your
 *  compiler for specific instructions on preincluding a header file.
 */
#include <intrinsics.h>

#ifdef __MSP430G2553__
#include <msp430g2553.h>
#else
//#error "Error: Selected microcontroller is not supported. Please select the MSP430G2553."
#endif

#ifndef ST
#define ST(X) do { X } while (0)
#endif

#ifndef NULL
#define NULL  (void*)0
#endif

// -----------------------------------------------------------------------------
/**
 *  Microcontroller global interrupt control support
 */

#if defined( __IAR_SYSTEMS_ICC__ )
#define MCU_DISABLE_INTERRUPT()      
#define MCU_ENABLE_INTERRUPT()        
#define MCU_CRITICAL_SECTION(code) code 
#else
#define MCU_DISABLE_INTERRUPT()       
#define MCU_ENABLE_INTERRUPT()        
#define MCU_CRITICAL_SECTION(code) code
#endif

// ----------------------------------------------------------------------------- 
/**
 *  Protocol platform characteristics
 */

#define A110LR09_MODULE                 // Use A110LR09 radio module
#define RF_SPI_CSN_1                    // Default CSn position
#define RF_GDO0_1                       // Default GDO0 position
#define TIMER1_A                        // Protocol uses Timer1_A3
    
// -----------------------------------------------------------------------------
/**
 *  Physical radio characteristics
 */

#define A110LR09_FCC_2FSK_1_2_KBAUD       // Configuration => 2FSK, 1.2kBaud, 902MHz
#define A110LR09_POWER_7_0_DBM            // Power table setting => 7.0dBm
#define PHY_MAX_TXFIFO_SIZE         64    // Physical hardware absolute FIFO size

// -----------------------------------------------------------------------------
/**
 *  Protocol characteristics
 */

#define PROTOCOL_GATEWAY                        // Node role
#define PROTOCOL_CHANNEL_LIST               0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40   // Physical channel list (comma seperated)
#define PROTOCOL_CHANNEL_LIST_SIZE          21   // Physical channel list size
#define PROTOCOL_PHYADDRESS_PANID_SIZE      2   // Physical address PAN identifier size
#define PROTOCOL_PHYADDRESS_ADDRESS_SIZE    4   // Physical address size
#define PROTOCOL_FRAME_MAX_PAYLOAD_LENGTH   0x00FF   // Maximum frame payload length

#endif  /* SIMPLEX_TRANSFER_LR09_CONFIG_H */
