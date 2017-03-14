/*******************************************************************************
* File Name: WIFIdev_Channel.h
* Version 2.50
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_UART_WIFIdev_Channel_H)
#define CY_UART_WIFIdev_Channel_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define WIFIdev_Channel_RX_ENABLED                     (1u)
#define WIFIdev_Channel_TX_ENABLED                     (1u)
#define WIFIdev_Channel_HD_ENABLED                     (0u)
#define WIFIdev_Channel_RX_INTERRUPT_ENABLED           (1u)
#define WIFIdev_Channel_TX_INTERRUPT_ENABLED           (0u)
#define WIFIdev_Channel_INTERNAL_CLOCK_USED            (1u)
#define WIFIdev_Channel_RXHW_ADDRESS_ENABLED           (0u)
#define WIFIdev_Channel_OVER_SAMPLE_COUNT              (8u)
#define WIFIdev_Channel_PARITY_TYPE                    (0u)
#define WIFIdev_Channel_PARITY_TYPE_SW                 (0u)
#define WIFIdev_Channel_BREAK_DETECT                   (0u)
#define WIFIdev_Channel_BREAK_BITS_TX                  (13u)
#define WIFIdev_Channel_BREAK_BITS_RX                  (13u)
#define WIFIdev_Channel_TXCLKGEN_DP                    (1u)
#define WIFIdev_Channel_USE23POLLING                   (1u)
#define WIFIdev_Channel_FLOW_CONTROL                   (0u)
#define WIFIdev_Channel_CLK_FREQ                       (0u)
#define WIFIdev_Channel_TX_BUFFER_SIZE                 (4u)
#define WIFIdev_Channel_RX_BUFFER_SIZE                 (32u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(WIFIdev_Channel_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define WIFIdev_Channel_CONTROL_REG_REMOVED            (0u)
#else
    #define WIFIdev_Channel_CONTROL_REG_REMOVED            (1u)
#endif /* End WIFIdev_Channel_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct WIFIdev_Channel_backupStruct_
{
    uint8 enableState;

    #if(WIFIdev_Channel_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End WIFIdev_Channel_CONTROL_REG_REMOVED */

} WIFIdev_Channel_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void WIFIdev_Channel_Start(void) ;
void WIFIdev_Channel_Stop(void) ;
uint8 WIFIdev_Channel_ReadControlRegister(void) ;
void WIFIdev_Channel_WriteControlRegister(uint8 control) ;

void WIFIdev_Channel_Init(void) ;
void WIFIdev_Channel_Enable(void) ;
void WIFIdev_Channel_SaveConfig(void) ;
void WIFIdev_Channel_RestoreConfig(void) ;
void WIFIdev_Channel_Sleep(void) ;
void WIFIdev_Channel_Wakeup(void) ;

/* Only if RX is enabled */
#if( (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) )

    #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)
        #define WIFIdev_Channel_EnableRxInt()  CyIntEnable (WIFIdev_Channel_RX_VECT_NUM)
        #define WIFIdev_Channel_DisableRxInt() CyIntDisable(WIFIdev_Channel_RX_VECT_NUM)
        CY_ISR_PROTO(WIFIdev_Channel_RXISR);
    #endif /* WIFIdev_Channel_RX_INTERRUPT_ENABLED */

    void WIFIdev_Channel_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void WIFIdev_Channel_SetRxAddress1(uint8 address) ;
    void WIFIdev_Channel_SetRxAddress2(uint8 address) ;

    void  WIFIdev_Channel_SetRxInterruptMode(uint8 intSrc) ;
    uint8 WIFIdev_Channel_ReadRxData(void) ;
    uint8 WIFIdev_Channel_ReadRxStatus(void) ;
    uint8 WIFIdev_Channel_GetChar(void) ;
    uint16 WIFIdev_Channel_GetByte(void) ;
    uint8 WIFIdev_Channel_GetRxBufferSize(void)
                                                            ;
    void WIFIdev_Channel_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define WIFIdev_Channel_GetRxInterruptSource   WIFIdev_Channel_ReadRxStatus

#endif /* End (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) */

/* Only if TX is enabled */
#if(WIFIdev_Channel_TX_ENABLED || WIFIdev_Channel_HD_ENABLED)

    #if(WIFIdev_Channel_TX_INTERRUPT_ENABLED)
        #define WIFIdev_Channel_EnableTxInt()  CyIntEnable (WIFIdev_Channel_TX_VECT_NUM)
        #define WIFIdev_Channel_DisableTxInt() CyIntDisable(WIFIdev_Channel_TX_VECT_NUM)
        #define WIFIdev_Channel_SetPendingTxInt() CyIntSetPending(WIFIdev_Channel_TX_VECT_NUM)
        #define WIFIdev_Channel_ClearPendingTxInt() CyIntClearPending(WIFIdev_Channel_TX_VECT_NUM)
        CY_ISR_PROTO(WIFIdev_Channel_TXISR);
    #endif /* WIFIdev_Channel_TX_INTERRUPT_ENABLED */

    void WIFIdev_Channel_SetTxInterruptMode(uint8 intSrc) ;
    void WIFIdev_Channel_WriteTxData(uint8 txDataByte) ;
    uint8 WIFIdev_Channel_ReadTxStatus(void) ;
    void WIFIdev_Channel_PutChar(uint8 txDataByte) ;
    void WIFIdev_Channel_PutString(const char8 string[]) ;
    void WIFIdev_Channel_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void WIFIdev_Channel_PutCRLF(uint8 txDataByte) ;
    void WIFIdev_Channel_ClearTxBuffer(void) ;
    void WIFIdev_Channel_SetTxAddressMode(uint8 addressMode) ;
    void WIFIdev_Channel_SendBreak(uint8 retMode) ;
    uint8 WIFIdev_Channel_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define WIFIdev_Channel_PutStringConst         WIFIdev_Channel_PutString
    #define WIFIdev_Channel_PutArrayConst          WIFIdev_Channel_PutArray
    #define WIFIdev_Channel_GetTxInterruptSource   WIFIdev_Channel_ReadTxStatus

#endif /* End WIFIdev_Channel_TX_ENABLED || WIFIdev_Channel_HD_ENABLED */

#if(WIFIdev_Channel_HD_ENABLED)
    void WIFIdev_Channel_LoadRxConfig(void) ;
    void WIFIdev_Channel_LoadTxConfig(void) ;
#endif /* End WIFIdev_Channel_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_WIFIdev_Channel) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    WIFIdev_Channel_CyBtldrCommStart(void) CYSMALL ;
    void    WIFIdev_Channel_CyBtldrCommStop(void) CYSMALL ;
    void    WIFIdev_Channel_CyBtldrCommReset(void) CYSMALL ;
    cystatus WIFIdev_Channel_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus WIFIdev_Channel_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_WIFIdev_Channel)
        #define CyBtldrCommStart    WIFIdev_Channel_CyBtldrCommStart
        #define CyBtldrCommStop     WIFIdev_Channel_CyBtldrCommStop
        #define CyBtldrCommReset    WIFIdev_Channel_CyBtldrCommReset
        #define CyBtldrCommWrite    WIFIdev_Channel_CyBtldrCommWrite
        #define CyBtldrCommRead     WIFIdev_Channel_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_WIFIdev_Channel) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define WIFIdev_Channel_BYTE2BYTE_TIME_OUT (25u)
    #define WIFIdev_Channel_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define WIFIdev_Channel_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define WIFIdev_Channel_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define WIFIdev_Channel_SET_SPACE      (0x00u)
#define WIFIdev_Channel_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (WIFIdev_Channel_TX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) )
    #if(WIFIdev_Channel_TX_INTERRUPT_ENABLED)
        #define WIFIdev_Channel_TX_VECT_NUM            (uint8)WIFIdev_Channel_TXInternalInterrupt__INTC_NUMBER
        #define WIFIdev_Channel_TX_PRIOR_NUM           (uint8)WIFIdev_Channel_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* WIFIdev_Channel_TX_INTERRUPT_ENABLED */

    #define WIFIdev_Channel_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define WIFIdev_Channel_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define WIFIdev_Channel_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(WIFIdev_Channel_TX_ENABLED)
        #define WIFIdev_Channel_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (WIFIdev_Channel_HD_ENABLED) */
        #define WIFIdev_Channel_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (WIFIdev_Channel_TX_ENABLED) */

    #define WIFIdev_Channel_TX_STS_COMPLETE            (uint8)(0x01u << WIFIdev_Channel_TX_STS_COMPLETE_SHIFT)
    #define WIFIdev_Channel_TX_STS_FIFO_EMPTY          (uint8)(0x01u << WIFIdev_Channel_TX_STS_FIFO_EMPTY_SHIFT)
    #define WIFIdev_Channel_TX_STS_FIFO_FULL           (uint8)(0x01u << WIFIdev_Channel_TX_STS_FIFO_FULL_SHIFT)
    #define WIFIdev_Channel_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << WIFIdev_Channel_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (WIFIdev_Channel_TX_ENABLED) || (WIFIdev_Channel_HD_ENABLED)*/

#if( (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) )
    #if(WIFIdev_Channel_RX_INTERRUPT_ENABLED)
        #define WIFIdev_Channel_RX_VECT_NUM            (uint8)WIFIdev_Channel_RXInternalInterrupt__INTC_NUMBER
        #define WIFIdev_Channel_RX_PRIOR_NUM           (uint8)WIFIdev_Channel_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* WIFIdev_Channel_RX_INTERRUPT_ENABLED */
    #define WIFIdev_Channel_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define WIFIdev_Channel_RX_STS_BREAK_SHIFT             (0x01u)
    #define WIFIdev_Channel_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define WIFIdev_Channel_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define WIFIdev_Channel_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define WIFIdev_Channel_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define WIFIdev_Channel_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define WIFIdev_Channel_RX_STS_MRKSPC           (uint8)(0x01u << WIFIdev_Channel_RX_STS_MRKSPC_SHIFT)
    #define WIFIdev_Channel_RX_STS_BREAK            (uint8)(0x01u << WIFIdev_Channel_RX_STS_BREAK_SHIFT)
    #define WIFIdev_Channel_RX_STS_PAR_ERROR        (uint8)(0x01u << WIFIdev_Channel_RX_STS_PAR_ERROR_SHIFT)
    #define WIFIdev_Channel_RX_STS_STOP_ERROR       (uint8)(0x01u << WIFIdev_Channel_RX_STS_STOP_ERROR_SHIFT)
    #define WIFIdev_Channel_RX_STS_OVERRUN          (uint8)(0x01u << WIFIdev_Channel_RX_STS_OVERRUN_SHIFT)
    #define WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define WIFIdev_Channel_RX_STS_ADDR_MATCH       (uint8)(0x01u << WIFIdev_Channel_RX_STS_ADDR_MATCH_SHIFT)
    #define WIFIdev_Channel_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << WIFIdev_Channel_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define WIFIdev_Channel_RX_HW_MASK                     (0x7Fu)
#endif /* End (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) */

/* Control Register definitions */
#define WIFIdev_Channel_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define WIFIdev_Channel_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define WIFIdev_Channel_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define WIFIdev_Channel_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define WIFIdev_Channel_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define WIFIdev_Channel_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define WIFIdev_Channel_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define WIFIdev_Channel_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define WIFIdev_Channel_CTRL_HD_SEND               (uint8)(0x01u << WIFIdev_Channel_CTRL_HD_SEND_SHIFT)
#define WIFIdev_Channel_CTRL_HD_SEND_BREAK         (uint8)(0x01u << WIFIdev_Channel_CTRL_HD_SEND_BREAK_SHIFT)
#define WIFIdev_Channel_CTRL_MARK                  (uint8)(0x01u << WIFIdev_Channel_CTRL_MARK_SHIFT)
#define WIFIdev_Channel_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << WIFIdev_Channel_CTRL_PARITY_TYPE0_SHIFT)
#define WIFIdev_Channel_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << WIFIdev_Channel_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define WIFIdev_Channel_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define WIFIdev_Channel_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define WIFIdev_Channel_SEND_BREAK                         (0x00u)
#define WIFIdev_Channel_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define WIFIdev_Channel_REINIT                             (0x02u)
#define WIFIdev_Channel_SEND_WAIT_REINIT                   (0x03u)

#define WIFIdev_Channel_OVER_SAMPLE_8                      (8u)
#define WIFIdev_Channel_OVER_SAMPLE_16                     (16u)

#define WIFIdev_Channel_BIT_CENTER                         (WIFIdev_Channel_OVER_SAMPLE_COUNT - 2u)

#define WIFIdev_Channel_FIFO_LENGTH                        (4u)
#define WIFIdev_Channel_NUMBER_OF_START_BIT                (1u)
#define WIFIdev_Channel_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define WIFIdev_Channel_TXBITCTR_BREAKBITS8X   ((WIFIdev_Channel_BREAK_BITS_TX * WIFIdev_Channel_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define WIFIdev_Channel_TXBITCTR_BREAKBITS ((WIFIdev_Channel_BREAK_BITS_TX * WIFIdev_Channel_OVER_SAMPLE_COUNT) - 1u)

#define WIFIdev_Channel_HALF_BIT_COUNT   \
                            (((WIFIdev_Channel_OVER_SAMPLE_COUNT / 2u) + (WIFIdev_Channel_USE23POLLING * 1u)) - 2u)
#if (WIFIdev_Channel_OVER_SAMPLE_COUNT == WIFIdev_Channel_OVER_SAMPLE_8)
    #define WIFIdev_Channel_HD_TXBITCTR_INIT   (((WIFIdev_Channel_BREAK_BITS_TX + \
                            WIFIdev_Channel_NUMBER_OF_START_BIT) * WIFIdev_Channel_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define WIFIdev_Channel_RXBITCTR_INIT  ((((WIFIdev_Channel_BREAK_BITS_RX + WIFIdev_Channel_NUMBER_OF_START_BIT) \
                            * WIFIdev_Channel_OVER_SAMPLE_COUNT) + WIFIdev_Channel_HALF_BIT_COUNT) - 1u)

#else /* WIFIdev_Channel_OVER_SAMPLE_COUNT == WIFIdev_Channel_OVER_SAMPLE_16 */
    #define WIFIdev_Channel_HD_TXBITCTR_INIT   ((8u * WIFIdev_Channel_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define WIFIdev_Channel_RXBITCTR_INIT      (((7u * WIFIdev_Channel_OVER_SAMPLE_COUNT) - 1u) + \
                                                      WIFIdev_Channel_HALF_BIT_COUNT)
#endif /* End WIFIdev_Channel_OVER_SAMPLE_COUNT */

#define WIFIdev_Channel_HD_RXBITCTR_INIT                   WIFIdev_Channel_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 WIFIdev_Channel_initVar;
#if (WIFIdev_Channel_TX_INTERRUPT_ENABLED && WIFIdev_Channel_TX_ENABLED)
    extern volatile uint8 WIFIdev_Channel_txBuffer[WIFIdev_Channel_TX_BUFFER_SIZE];
    extern volatile uint8 WIFIdev_Channel_txBufferRead;
    extern uint8 WIFIdev_Channel_txBufferWrite;
#endif /* (WIFIdev_Channel_TX_INTERRUPT_ENABLED && WIFIdev_Channel_TX_ENABLED) */
#if (WIFIdev_Channel_RX_INTERRUPT_ENABLED && (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED))
    extern uint8 WIFIdev_Channel_errorStatus;
    extern volatile uint8 WIFIdev_Channel_rxBuffer[WIFIdev_Channel_RX_BUFFER_SIZE];
    extern volatile uint8 WIFIdev_Channel_rxBufferRead;
    extern volatile uint8 WIFIdev_Channel_rxBufferWrite;
    extern volatile uint8 WIFIdev_Channel_rxBufferLoopDetect;
    extern volatile uint8 WIFIdev_Channel_rxBufferOverflow;
    #if (WIFIdev_Channel_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 WIFIdev_Channel_rxAddressMode;
        extern volatile uint8 WIFIdev_Channel_rxAddressDetected;
    #endif /* (WIFIdev_Channel_RXHW_ADDRESS_ENABLED) */
#endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED && (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define WIFIdev_Channel__B_UART__AM_SW_BYTE_BYTE 1
#define WIFIdev_Channel__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define WIFIdev_Channel__B_UART__AM_HW_BYTE_BY_BYTE 3
#define WIFIdev_Channel__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define WIFIdev_Channel__B_UART__AM_NONE 0

#define WIFIdev_Channel__B_UART__NONE_REVB 0
#define WIFIdev_Channel__B_UART__EVEN_REVB 1
#define WIFIdev_Channel__B_UART__ODD_REVB 2
#define WIFIdev_Channel__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define WIFIdev_Channel_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define WIFIdev_Channel_NUMBER_OF_STOP_BITS    (1u)

#if (WIFIdev_Channel_RXHW_ADDRESS_ENABLED)
    #define WIFIdev_Channel_RX_ADDRESS_MODE    (0u)
    #define WIFIdev_Channel_RX_HW_ADDRESS1     (0u)
    #define WIFIdev_Channel_RX_HW_ADDRESS2     (0u)
#endif /* (WIFIdev_Channel_RXHW_ADDRESS_ENABLED) */

#define WIFIdev_Channel_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << WIFIdev_Channel_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << WIFIdev_Channel_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << WIFIdev_Channel_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << WIFIdev_Channel_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << WIFIdev_Channel_RX_STS_BREAK_SHIFT) \
                                        | (0 << WIFIdev_Channel_RX_STS_OVERRUN_SHIFT))

#define WIFIdev_Channel_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << WIFIdev_Channel_TX_STS_COMPLETE_SHIFT) \
                                        | (1 << WIFIdev_Channel_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << WIFIdev_Channel_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << WIFIdev_Channel_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef WIFIdev_Channel_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define WIFIdev_Channel_CONTROL_REG \
                            (* (reg8 *) WIFIdev_Channel_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define WIFIdev_Channel_CONTROL_PTR \
                            (  (reg8 *) WIFIdev_Channel_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End WIFIdev_Channel_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(WIFIdev_Channel_TX_ENABLED)
    #define WIFIdev_Channel_TXDATA_REG          (* (reg8 *) WIFIdev_Channel_BUART_sTX_TxShifter_u0__F0_REG)
    #define WIFIdev_Channel_TXDATA_PTR          (  (reg8 *) WIFIdev_Channel_BUART_sTX_TxShifter_u0__F0_REG)
    #define WIFIdev_Channel_TXDATA_AUX_CTL_REG  (* (reg8 *) WIFIdev_Channel_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define WIFIdev_Channel_TXDATA_AUX_CTL_PTR  (  (reg8 *) WIFIdev_Channel_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define WIFIdev_Channel_TXSTATUS_REG        (* (reg8 *) WIFIdev_Channel_BUART_sTX_TxSts__STATUS_REG)
    #define WIFIdev_Channel_TXSTATUS_PTR        (  (reg8 *) WIFIdev_Channel_BUART_sTX_TxSts__STATUS_REG)
    #define WIFIdev_Channel_TXSTATUS_MASK_REG   (* (reg8 *) WIFIdev_Channel_BUART_sTX_TxSts__MASK_REG)
    #define WIFIdev_Channel_TXSTATUS_MASK_PTR   (  (reg8 *) WIFIdev_Channel_BUART_sTX_TxSts__MASK_REG)
    #define WIFIdev_Channel_TXSTATUS_ACTL_REG   (* (reg8 *) WIFIdev_Channel_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define WIFIdev_Channel_TXSTATUS_ACTL_PTR   (  (reg8 *) WIFIdev_Channel_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(WIFIdev_Channel_TXCLKGEN_DP)
        #define WIFIdev_Channel_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) WIFIdev_Channel_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define WIFIdev_Channel_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) WIFIdev_Channel_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define WIFIdev_Channel_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) WIFIdev_Channel_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define WIFIdev_Channel_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) WIFIdev_Channel_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define WIFIdev_Channel_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) WIFIdev_Channel_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define WIFIdev_Channel_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) WIFIdev_Channel_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define WIFIdev_Channel_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) WIFIdev_Channel_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define WIFIdev_Channel_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) WIFIdev_Channel_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define WIFIdev_Channel_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) WIFIdev_Channel_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define WIFIdev_Channel_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) WIFIdev_Channel_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* WIFIdev_Channel_TXCLKGEN_DP */

#endif /* End WIFIdev_Channel_TX_ENABLED */

#if(WIFIdev_Channel_HD_ENABLED)

    #define WIFIdev_Channel_TXDATA_REG             (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxShifter_u0__F1_REG )
    #define WIFIdev_Channel_TXDATA_PTR             (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxShifter_u0__F1_REG )
    #define WIFIdev_Channel_TXDATA_AUX_CTL_REG     (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define WIFIdev_Channel_TXDATA_AUX_CTL_PTR     (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define WIFIdev_Channel_TXSTATUS_REG           (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__STATUS_REG )
    #define WIFIdev_Channel_TXSTATUS_PTR           (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__STATUS_REG )
    #define WIFIdev_Channel_TXSTATUS_MASK_REG      (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__MASK_REG )
    #define WIFIdev_Channel_TXSTATUS_MASK_PTR      (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__MASK_REG )
    #define WIFIdev_Channel_TXSTATUS_ACTL_REG      (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define WIFIdev_Channel_TXSTATUS_ACTL_PTR      (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End WIFIdev_Channel_HD_ENABLED */

#if( (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) )
    #define WIFIdev_Channel_RXDATA_REG             (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxShifter_u0__F0_REG )
    #define WIFIdev_Channel_RXDATA_PTR             (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxShifter_u0__F0_REG )
    #define WIFIdev_Channel_RXADDRESS1_REG         (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxShifter_u0__D0_REG )
    #define WIFIdev_Channel_RXADDRESS1_PTR         (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxShifter_u0__D0_REG )
    #define WIFIdev_Channel_RXADDRESS2_REG         (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxShifter_u0__D1_REG )
    #define WIFIdev_Channel_RXADDRESS2_PTR         (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxShifter_u0__D1_REG )
    #define WIFIdev_Channel_RXDATA_AUX_CTL_REG     (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define WIFIdev_Channel_RXBITCTR_PERIOD_REG    (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define WIFIdev_Channel_RXBITCTR_PERIOD_PTR    (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define WIFIdev_Channel_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define WIFIdev_Channel_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define WIFIdev_Channel_RXBITCTR_COUNTER_REG   (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxBitCounter__COUNT_REG )
    #define WIFIdev_Channel_RXBITCTR_COUNTER_PTR   (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxBitCounter__COUNT_REG )

    #define WIFIdev_Channel_RXSTATUS_REG           (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__STATUS_REG )
    #define WIFIdev_Channel_RXSTATUS_PTR           (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__STATUS_REG )
    #define WIFIdev_Channel_RXSTATUS_MASK_REG      (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__MASK_REG )
    #define WIFIdev_Channel_RXSTATUS_MASK_PTR      (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__MASK_REG )
    #define WIFIdev_Channel_RXSTATUS_ACTL_REG      (* (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define WIFIdev_Channel_RXSTATUS_ACTL_PTR      (  (reg8 *) WIFIdev_Channel_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) */

#if(WIFIdev_Channel_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define WIFIdev_Channel_INTCLOCK_CLKEN_REG     (* (reg8 *) WIFIdev_Channel_IntClock__PM_ACT_CFG)
    #define WIFIdev_Channel_INTCLOCK_CLKEN_PTR     (  (reg8 *) WIFIdev_Channel_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define WIFIdev_Channel_INTCLOCK_CLKEN_MASK    WIFIdev_Channel_IntClock__PM_ACT_MSK
#endif /* End WIFIdev_Channel_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(WIFIdev_Channel_TX_ENABLED)
    #define WIFIdev_Channel_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End WIFIdev_Channel_TX_ENABLED */

#if(WIFIdev_Channel_HD_ENABLED)
    #define WIFIdev_Channel_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End WIFIdev_Channel_HD_ENABLED */

#if( (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) )
    #define WIFIdev_Channel_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define WIFIdev_Channel_WAIT_1_MS      WIFIdev_Channel_BL_CHK_DELAY_MS   

#define WIFIdev_Channel_TXBUFFERSIZE   WIFIdev_Channel_TX_BUFFER_SIZE
#define WIFIdev_Channel_RXBUFFERSIZE   WIFIdev_Channel_RX_BUFFER_SIZE

#if (WIFIdev_Channel_RXHW_ADDRESS_ENABLED)
    #define WIFIdev_Channel_RXADDRESSMODE  WIFIdev_Channel_RX_ADDRESS_MODE
    #define WIFIdev_Channel_RXHWADDRESS1   WIFIdev_Channel_RX_HW_ADDRESS1
    #define WIFIdev_Channel_RXHWADDRESS2   WIFIdev_Channel_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define WIFIdev_Channel_RXAddressMode  WIFIdev_Channel_RXADDRESSMODE
#endif /* (WIFIdev_Channel_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define WIFIdev_Channel_initvar                    WIFIdev_Channel_initVar

#define WIFIdev_Channel_RX_Enabled                 WIFIdev_Channel_RX_ENABLED
#define WIFIdev_Channel_TX_Enabled                 WIFIdev_Channel_TX_ENABLED
#define WIFIdev_Channel_HD_Enabled                 WIFIdev_Channel_HD_ENABLED
#define WIFIdev_Channel_RX_IntInterruptEnabled     WIFIdev_Channel_RX_INTERRUPT_ENABLED
#define WIFIdev_Channel_TX_IntInterruptEnabled     WIFIdev_Channel_TX_INTERRUPT_ENABLED
#define WIFIdev_Channel_InternalClockUsed          WIFIdev_Channel_INTERNAL_CLOCK_USED
#define WIFIdev_Channel_RXHW_Address_Enabled       WIFIdev_Channel_RXHW_ADDRESS_ENABLED
#define WIFIdev_Channel_OverSampleCount            WIFIdev_Channel_OVER_SAMPLE_COUNT
#define WIFIdev_Channel_ParityType                 WIFIdev_Channel_PARITY_TYPE

#if( WIFIdev_Channel_TX_ENABLED && (WIFIdev_Channel_TXBUFFERSIZE > WIFIdev_Channel_FIFO_LENGTH))
    #define WIFIdev_Channel_TXBUFFER               WIFIdev_Channel_txBuffer
    #define WIFIdev_Channel_TXBUFFERREAD           WIFIdev_Channel_txBufferRead
    #define WIFIdev_Channel_TXBUFFERWRITE          WIFIdev_Channel_txBufferWrite
#endif /* End WIFIdev_Channel_TX_ENABLED */
#if( ( WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED ) && \
     (WIFIdev_Channel_RXBUFFERSIZE > WIFIdev_Channel_FIFO_LENGTH) )
    #define WIFIdev_Channel_RXBUFFER               WIFIdev_Channel_rxBuffer
    #define WIFIdev_Channel_RXBUFFERREAD           WIFIdev_Channel_rxBufferRead
    #define WIFIdev_Channel_RXBUFFERWRITE          WIFIdev_Channel_rxBufferWrite
    #define WIFIdev_Channel_RXBUFFERLOOPDETECT     WIFIdev_Channel_rxBufferLoopDetect
    #define WIFIdev_Channel_RXBUFFER_OVERFLOW      WIFIdev_Channel_rxBufferOverflow
#endif /* End WIFIdev_Channel_RX_ENABLED */

#ifdef WIFIdev_Channel_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define WIFIdev_Channel_CONTROL                WIFIdev_Channel_CONTROL_REG
#endif /* End WIFIdev_Channel_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(WIFIdev_Channel_TX_ENABLED)
    #define WIFIdev_Channel_TXDATA                 WIFIdev_Channel_TXDATA_REG
    #define WIFIdev_Channel_TXSTATUS               WIFIdev_Channel_TXSTATUS_REG
    #define WIFIdev_Channel_TXSTATUS_MASK          WIFIdev_Channel_TXSTATUS_MASK_REG
    #define WIFIdev_Channel_TXSTATUS_ACTL          WIFIdev_Channel_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(WIFIdev_Channel_TXCLKGEN_DP)
        #define WIFIdev_Channel_TXBITCLKGEN_CTR        WIFIdev_Channel_TXBITCLKGEN_CTR_REG
        #define WIFIdev_Channel_TXBITCLKTX_COMPLETE    WIFIdev_Channel_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define WIFIdev_Channel_TXBITCTR_PERIOD        WIFIdev_Channel_TXBITCTR_PERIOD_REG
        #define WIFIdev_Channel_TXBITCTR_CONTROL       WIFIdev_Channel_TXBITCTR_CONTROL_REG
        #define WIFIdev_Channel_TXBITCTR_COUNTER       WIFIdev_Channel_TXBITCTR_COUNTER_REG
    #endif /* WIFIdev_Channel_TXCLKGEN_DP */
#endif /* End WIFIdev_Channel_TX_ENABLED */

#if(WIFIdev_Channel_HD_ENABLED)
    #define WIFIdev_Channel_TXDATA                 WIFIdev_Channel_TXDATA_REG
    #define WIFIdev_Channel_TXSTATUS               WIFIdev_Channel_TXSTATUS_REG
    #define WIFIdev_Channel_TXSTATUS_MASK          WIFIdev_Channel_TXSTATUS_MASK_REG
    #define WIFIdev_Channel_TXSTATUS_ACTL          WIFIdev_Channel_TXSTATUS_ACTL_REG
#endif /* End WIFIdev_Channel_HD_ENABLED */

#if( (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) )
    #define WIFIdev_Channel_RXDATA                 WIFIdev_Channel_RXDATA_REG
    #define WIFIdev_Channel_RXADDRESS1             WIFIdev_Channel_RXADDRESS1_REG
    #define WIFIdev_Channel_RXADDRESS2             WIFIdev_Channel_RXADDRESS2_REG
    #define WIFIdev_Channel_RXBITCTR_PERIOD        WIFIdev_Channel_RXBITCTR_PERIOD_REG
    #define WIFIdev_Channel_RXBITCTR_CONTROL       WIFIdev_Channel_RXBITCTR_CONTROL_REG
    #define WIFIdev_Channel_RXBITCTR_COUNTER       WIFIdev_Channel_RXBITCTR_COUNTER_REG
    #define WIFIdev_Channel_RXSTATUS               WIFIdev_Channel_RXSTATUS_REG
    #define WIFIdev_Channel_RXSTATUS_MASK          WIFIdev_Channel_RXSTATUS_MASK_REG
    #define WIFIdev_Channel_RXSTATUS_ACTL          WIFIdev_Channel_RXSTATUS_ACTL_REG
#endif /* End  (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) */

#if(WIFIdev_Channel_INTERNAL_CLOCK_USED)
    #define WIFIdev_Channel_INTCLOCK_CLKEN         WIFIdev_Channel_INTCLOCK_CLKEN_REG
#endif /* End WIFIdev_Channel_INTERNAL_CLOCK_USED */

#define WIFIdev_Channel_WAIT_FOR_COMLETE_REINIT    WIFIdev_Channel_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_WIFIdev_Channel_H */


/* [] END OF FILE */
