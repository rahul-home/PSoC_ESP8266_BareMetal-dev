/*******************************************************************************
* File Name: LOG.h
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


#if !defined(CY_UART_LOG_H)
#define CY_UART_LOG_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define LOG_RX_ENABLED                     (0u)
#define LOG_TX_ENABLED                     (1u)
#define LOG_HD_ENABLED                     (0u)
#define LOG_RX_INTERRUPT_ENABLED           (1u)
#define LOG_TX_INTERRUPT_ENABLED           (0u)
#define LOG_INTERNAL_CLOCK_USED            (1u)
#define LOG_RXHW_ADDRESS_ENABLED           (0u)
#define LOG_OVER_SAMPLE_COUNT              (8u)
#define LOG_PARITY_TYPE                    (0u)
#define LOG_PARITY_TYPE_SW                 (0u)
#define LOG_BREAK_DETECT                   (0u)
#define LOG_BREAK_BITS_TX                  (13u)
#define LOG_BREAK_BITS_RX                  (13u)
#define LOG_TXCLKGEN_DP                    (1u)
#define LOG_USE23POLLING                   (1u)
#define LOG_FLOW_CONTROL                   (0u)
#define LOG_CLK_FREQ                       (0u)
#define LOG_TX_BUFFER_SIZE                 (4u)
#define LOG_RX_BUFFER_SIZE                 (32u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(LOG_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define LOG_CONTROL_REG_REMOVED            (0u)
#else
    #define LOG_CONTROL_REG_REMOVED            (1u)
#endif /* End LOG_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct LOG_backupStruct_
{
    uint8 enableState;

    #if(LOG_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End LOG_CONTROL_REG_REMOVED */

} LOG_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void LOG_Start(void) ;
void LOG_Stop(void) ;
uint8 LOG_ReadControlRegister(void) ;
void LOG_WriteControlRegister(uint8 control) ;

void LOG_Init(void) ;
void LOG_Enable(void) ;
void LOG_SaveConfig(void) ;
void LOG_RestoreConfig(void) ;
void LOG_Sleep(void) ;
void LOG_Wakeup(void) ;

/* Only if RX is enabled */
#if( (LOG_RX_ENABLED) || (LOG_HD_ENABLED) )

    #if (LOG_RX_INTERRUPT_ENABLED)
        #define LOG_EnableRxInt()  CyIntEnable (LOG_RX_VECT_NUM)
        #define LOG_DisableRxInt() CyIntDisable(LOG_RX_VECT_NUM)
        CY_ISR_PROTO(LOG_RXISR);
    #endif /* LOG_RX_INTERRUPT_ENABLED */

    void LOG_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void LOG_SetRxAddress1(uint8 address) ;
    void LOG_SetRxAddress2(uint8 address) ;

    void  LOG_SetRxInterruptMode(uint8 intSrc) ;
    uint8 LOG_ReadRxData(void) ;
    uint8 LOG_ReadRxStatus(void) ;
    uint8 LOG_GetChar(void) ;
    uint16 LOG_GetByte(void) ;
    uint8 LOG_GetRxBufferSize(void)
                                                            ;
    void LOG_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define LOG_GetRxInterruptSource   LOG_ReadRxStatus

#endif /* End (LOG_RX_ENABLED) || (LOG_HD_ENABLED) */

/* Only if TX is enabled */
#if(LOG_TX_ENABLED || LOG_HD_ENABLED)

    #if(LOG_TX_INTERRUPT_ENABLED)
        #define LOG_EnableTxInt()  CyIntEnable (LOG_TX_VECT_NUM)
        #define LOG_DisableTxInt() CyIntDisable(LOG_TX_VECT_NUM)
        #define LOG_SetPendingTxInt() CyIntSetPending(LOG_TX_VECT_NUM)
        #define LOG_ClearPendingTxInt() CyIntClearPending(LOG_TX_VECT_NUM)
        CY_ISR_PROTO(LOG_TXISR);
    #endif /* LOG_TX_INTERRUPT_ENABLED */

    void LOG_SetTxInterruptMode(uint8 intSrc) ;
    void LOG_WriteTxData(uint8 txDataByte) ;
    uint8 LOG_ReadTxStatus(void) ;
    void LOG_PutChar(uint8 txDataByte) ;
    void LOG_PutString(const char8 string[]) ;
    void LOG_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void LOG_PutCRLF(uint8 txDataByte) ;
    void LOG_ClearTxBuffer(void) ;
    void LOG_SetTxAddressMode(uint8 addressMode) ;
    void LOG_SendBreak(uint8 retMode) ;
    uint8 LOG_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define LOG_PutStringConst         LOG_PutString
    #define LOG_PutArrayConst          LOG_PutArray
    #define LOG_GetTxInterruptSource   LOG_ReadTxStatus

#endif /* End LOG_TX_ENABLED || LOG_HD_ENABLED */

#if(LOG_HD_ENABLED)
    void LOG_LoadRxConfig(void) ;
    void LOG_LoadTxConfig(void) ;
#endif /* End LOG_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_LOG) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    LOG_CyBtldrCommStart(void) CYSMALL ;
    void    LOG_CyBtldrCommStop(void) CYSMALL ;
    void    LOG_CyBtldrCommReset(void) CYSMALL ;
    cystatus LOG_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus LOG_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_LOG)
        #define CyBtldrCommStart    LOG_CyBtldrCommStart
        #define CyBtldrCommStop     LOG_CyBtldrCommStop
        #define CyBtldrCommReset    LOG_CyBtldrCommReset
        #define CyBtldrCommWrite    LOG_CyBtldrCommWrite
        #define CyBtldrCommRead     LOG_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_LOG) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define LOG_BYTE2BYTE_TIME_OUT (25u)
    #define LOG_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define LOG_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define LOG_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define LOG_SET_SPACE      (0x00u)
#define LOG_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (LOG_TX_ENABLED) || (LOG_HD_ENABLED) )
    #if(LOG_TX_INTERRUPT_ENABLED)
        #define LOG_TX_VECT_NUM            (uint8)LOG_TXInternalInterrupt__INTC_NUMBER
        #define LOG_TX_PRIOR_NUM           (uint8)LOG_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* LOG_TX_INTERRUPT_ENABLED */

    #define LOG_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define LOG_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define LOG_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(LOG_TX_ENABLED)
        #define LOG_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (LOG_HD_ENABLED) */
        #define LOG_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (LOG_TX_ENABLED) */

    #define LOG_TX_STS_COMPLETE            (uint8)(0x01u << LOG_TX_STS_COMPLETE_SHIFT)
    #define LOG_TX_STS_FIFO_EMPTY          (uint8)(0x01u << LOG_TX_STS_FIFO_EMPTY_SHIFT)
    #define LOG_TX_STS_FIFO_FULL           (uint8)(0x01u << LOG_TX_STS_FIFO_FULL_SHIFT)
    #define LOG_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << LOG_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (LOG_TX_ENABLED) || (LOG_HD_ENABLED)*/

#if( (LOG_RX_ENABLED) || (LOG_HD_ENABLED) )
    #if(LOG_RX_INTERRUPT_ENABLED)
        #define LOG_RX_VECT_NUM            (uint8)LOG_RXInternalInterrupt__INTC_NUMBER
        #define LOG_RX_PRIOR_NUM           (uint8)LOG_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* LOG_RX_INTERRUPT_ENABLED */
    #define LOG_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define LOG_RX_STS_BREAK_SHIFT             (0x01u)
    #define LOG_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define LOG_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define LOG_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define LOG_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define LOG_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define LOG_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define LOG_RX_STS_MRKSPC           (uint8)(0x01u << LOG_RX_STS_MRKSPC_SHIFT)
    #define LOG_RX_STS_BREAK            (uint8)(0x01u << LOG_RX_STS_BREAK_SHIFT)
    #define LOG_RX_STS_PAR_ERROR        (uint8)(0x01u << LOG_RX_STS_PAR_ERROR_SHIFT)
    #define LOG_RX_STS_STOP_ERROR       (uint8)(0x01u << LOG_RX_STS_STOP_ERROR_SHIFT)
    #define LOG_RX_STS_OVERRUN          (uint8)(0x01u << LOG_RX_STS_OVERRUN_SHIFT)
    #define LOG_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << LOG_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define LOG_RX_STS_ADDR_MATCH       (uint8)(0x01u << LOG_RX_STS_ADDR_MATCH_SHIFT)
    #define LOG_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << LOG_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define LOG_RX_HW_MASK                     (0x7Fu)
#endif /* End (LOG_RX_ENABLED) || (LOG_HD_ENABLED) */

/* Control Register definitions */
#define LOG_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define LOG_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define LOG_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define LOG_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define LOG_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define LOG_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define LOG_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define LOG_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define LOG_CTRL_HD_SEND               (uint8)(0x01u << LOG_CTRL_HD_SEND_SHIFT)
#define LOG_CTRL_HD_SEND_BREAK         (uint8)(0x01u << LOG_CTRL_HD_SEND_BREAK_SHIFT)
#define LOG_CTRL_MARK                  (uint8)(0x01u << LOG_CTRL_MARK_SHIFT)
#define LOG_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << LOG_CTRL_PARITY_TYPE0_SHIFT)
#define LOG_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << LOG_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define LOG_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define LOG_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define LOG_SEND_BREAK                         (0x00u)
#define LOG_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define LOG_REINIT                             (0x02u)
#define LOG_SEND_WAIT_REINIT                   (0x03u)

#define LOG_OVER_SAMPLE_8                      (8u)
#define LOG_OVER_SAMPLE_16                     (16u)

#define LOG_BIT_CENTER                         (LOG_OVER_SAMPLE_COUNT - 2u)

#define LOG_FIFO_LENGTH                        (4u)
#define LOG_NUMBER_OF_START_BIT                (1u)
#define LOG_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define LOG_TXBITCTR_BREAKBITS8X   ((LOG_BREAK_BITS_TX * LOG_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define LOG_TXBITCTR_BREAKBITS ((LOG_BREAK_BITS_TX * LOG_OVER_SAMPLE_COUNT) - 1u)

#define LOG_HALF_BIT_COUNT   \
                            (((LOG_OVER_SAMPLE_COUNT / 2u) + (LOG_USE23POLLING * 1u)) - 2u)
#if (LOG_OVER_SAMPLE_COUNT == LOG_OVER_SAMPLE_8)
    #define LOG_HD_TXBITCTR_INIT   (((LOG_BREAK_BITS_TX + \
                            LOG_NUMBER_OF_START_BIT) * LOG_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define LOG_RXBITCTR_INIT  ((((LOG_BREAK_BITS_RX + LOG_NUMBER_OF_START_BIT) \
                            * LOG_OVER_SAMPLE_COUNT) + LOG_HALF_BIT_COUNT) - 1u)

#else /* LOG_OVER_SAMPLE_COUNT == LOG_OVER_SAMPLE_16 */
    #define LOG_HD_TXBITCTR_INIT   ((8u * LOG_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define LOG_RXBITCTR_INIT      (((7u * LOG_OVER_SAMPLE_COUNT) - 1u) + \
                                                      LOG_HALF_BIT_COUNT)
#endif /* End LOG_OVER_SAMPLE_COUNT */

#define LOG_HD_RXBITCTR_INIT                   LOG_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 LOG_initVar;
#if (LOG_TX_INTERRUPT_ENABLED && LOG_TX_ENABLED)
    extern volatile uint8 LOG_txBuffer[LOG_TX_BUFFER_SIZE];
    extern volatile uint8 LOG_txBufferRead;
    extern uint8 LOG_txBufferWrite;
#endif /* (LOG_TX_INTERRUPT_ENABLED && LOG_TX_ENABLED) */
#if (LOG_RX_INTERRUPT_ENABLED && (LOG_RX_ENABLED || LOG_HD_ENABLED))
    extern uint8 LOG_errorStatus;
    extern volatile uint8 LOG_rxBuffer[LOG_RX_BUFFER_SIZE];
    extern volatile uint8 LOG_rxBufferRead;
    extern volatile uint8 LOG_rxBufferWrite;
    extern volatile uint8 LOG_rxBufferLoopDetect;
    extern volatile uint8 LOG_rxBufferOverflow;
    #if (LOG_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 LOG_rxAddressMode;
        extern volatile uint8 LOG_rxAddressDetected;
    #endif /* (LOG_RXHW_ADDRESS_ENABLED) */
#endif /* (LOG_RX_INTERRUPT_ENABLED && (LOG_RX_ENABLED || LOG_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define LOG__B_UART__AM_SW_BYTE_BYTE 1
#define LOG__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define LOG__B_UART__AM_HW_BYTE_BY_BYTE 3
#define LOG__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define LOG__B_UART__AM_NONE 0

#define LOG__B_UART__NONE_REVB 0
#define LOG__B_UART__EVEN_REVB 1
#define LOG__B_UART__ODD_REVB 2
#define LOG__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define LOG_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define LOG_NUMBER_OF_STOP_BITS    (1u)

#if (LOG_RXHW_ADDRESS_ENABLED)
    #define LOG_RX_ADDRESS_MODE    (0u)
    #define LOG_RX_HW_ADDRESS1     (0u)
    #define LOG_RX_HW_ADDRESS2     (0u)
#endif /* (LOG_RXHW_ADDRESS_ENABLED) */

#define LOG_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((0 << LOG_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << LOG_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << LOG_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << LOG_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << LOG_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << LOG_RX_STS_BREAK_SHIFT) \
                                        | (0 << LOG_RX_STS_OVERRUN_SHIFT))

#define LOG_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << LOG_TX_STS_COMPLETE_SHIFT) \
                                        | (1 << LOG_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << LOG_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << LOG_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef LOG_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define LOG_CONTROL_REG \
                            (* (reg8 *) LOG_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define LOG_CONTROL_PTR \
                            (  (reg8 *) LOG_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End LOG_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(LOG_TX_ENABLED)
    #define LOG_TXDATA_REG          (* (reg8 *) LOG_BUART_sTX_TxShifter_u0__F0_REG)
    #define LOG_TXDATA_PTR          (  (reg8 *) LOG_BUART_sTX_TxShifter_u0__F0_REG)
    #define LOG_TXDATA_AUX_CTL_REG  (* (reg8 *) LOG_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define LOG_TXDATA_AUX_CTL_PTR  (  (reg8 *) LOG_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define LOG_TXSTATUS_REG        (* (reg8 *) LOG_BUART_sTX_TxSts__STATUS_REG)
    #define LOG_TXSTATUS_PTR        (  (reg8 *) LOG_BUART_sTX_TxSts__STATUS_REG)
    #define LOG_TXSTATUS_MASK_REG   (* (reg8 *) LOG_BUART_sTX_TxSts__MASK_REG)
    #define LOG_TXSTATUS_MASK_PTR   (  (reg8 *) LOG_BUART_sTX_TxSts__MASK_REG)
    #define LOG_TXSTATUS_ACTL_REG   (* (reg8 *) LOG_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define LOG_TXSTATUS_ACTL_PTR   (  (reg8 *) LOG_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(LOG_TXCLKGEN_DP)
        #define LOG_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) LOG_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define LOG_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) LOG_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define LOG_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) LOG_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define LOG_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) LOG_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define LOG_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) LOG_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define LOG_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) LOG_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define LOG_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) LOG_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define LOG_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) LOG_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define LOG_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) LOG_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define LOG_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) LOG_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* LOG_TXCLKGEN_DP */

#endif /* End LOG_TX_ENABLED */

#if(LOG_HD_ENABLED)

    #define LOG_TXDATA_REG             (* (reg8 *) LOG_BUART_sRX_RxShifter_u0__F1_REG )
    #define LOG_TXDATA_PTR             (  (reg8 *) LOG_BUART_sRX_RxShifter_u0__F1_REG )
    #define LOG_TXDATA_AUX_CTL_REG     (* (reg8 *) LOG_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define LOG_TXDATA_AUX_CTL_PTR     (  (reg8 *) LOG_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define LOG_TXSTATUS_REG           (* (reg8 *) LOG_BUART_sRX_RxSts__STATUS_REG )
    #define LOG_TXSTATUS_PTR           (  (reg8 *) LOG_BUART_sRX_RxSts__STATUS_REG )
    #define LOG_TXSTATUS_MASK_REG      (* (reg8 *) LOG_BUART_sRX_RxSts__MASK_REG )
    #define LOG_TXSTATUS_MASK_PTR      (  (reg8 *) LOG_BUART_sRX_RxSts__MASK_REG )
    #define LOG_TXSTATUS_ACTL_REG      (* (reg8 *) LOG_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define LOG_TXSTATUS_ACTL_PTR      (  (reg8 *) LOG_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End LOG_HD_ENABLED */

#if( (LOG_RX_ENABLED) || (LOG_HD_ENABLED) )
    #define LOG_RXDATA_REG             (* (reg8 *) LOG_BUART_sRX_RxShifter_u0__F0_REG )
    #define LOG_RXDATA_PTR             (  (reg8 *) LOG_BUART_sRX_RxShifter_u0__F0_REG )
    #define LOG_RXADDRESS1_REG         (* (reg8 *) LOG_BUART_sRX_RxShifter_u0__D0_REG )
    #define LOG_RXADDRESS1_PTR         (  (reg8 *) LOG_BUART_sRX_RxShifter_u0__D0_REG )
    #define LOG_RXADDRESS2_REG         (* (reg8 *) LOG_BUART_sRX_RxShifter_u0__D1_REG )
    #define LOG_RXADDRESS2_PTR         (  (reg8 *) LOG_BUART_sRX_RxShifter_u0__D1_REG )
    #define LOG_RXDATA_AUX_CTL_REG     (* (reg8 *) LOG_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define LOG_RXBITCTR_PERIOD_REG    (* (reg8 *) LOG_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define LOG_RXBITCTR_PERIOD_PTR    (  (reg8 *) LOG_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define LOG_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) LOG_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define LOG_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) LOG_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define LOG_RXBITCTR_COUNTER_REG   (* (reg8 *) LOG_BUART_sRX_RxBitCounter__COUNT_REG )
    #define LOG_RXBITCTR_COUNTER_PTR   (  (reg8 *) LOG_BUART_sRX_RxBitCounter__COUNT_REG )

    #define LOG_RXSTATUS_REG           (* (reg8 *) LOG_BUART_sRX_RxSts__STATUS_REG )
    #define LOG_RXSTATUS_PTR           (  (reg8 *) LOG_BUART_sRX_RxSts__STATUS_REG )
    #define LOG_RXSTATUS_MASK_REG      (* (reg8 *) LOG_BUART_sRX_RxSts__MASK_REG )
    #define LOG_RXSTATUS_MASK_PTR      (  (reg8 *) LOG_BUART_sRX_RxSts__MASK_REG )
    #define LOG_RXSTATUS_ACTL_REG      (* (reg8 *) LOG_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define LOG_RXSTATUS_ACTL_PTR      (  (reg8 *) LOG_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (LOG_RX_ENABLED) || (LOG_HD_ENABLED) */

#if(LOG_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define LOG_INTCLOCK_CLKEN_REG     (* (reg8 *) LOG_IntClock__PM_ACT_CFG)
    #define LOG_INTCLOCK_CLKEN_PTR     (  (reg8 *) LOG_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define LOG_INTCLOCK_CLKEN_MASK    LOG_IntClock__PM_ACT_MSK
#endif /* End LOG_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(LOG_TX_ENABLED)
    #define LOG_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End LOG_TX_ENABLED */

#if(LOG_HD_ENABLED)
    #define LOG_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End LOG_HD_ENABLED */

#if( (LOG_RX_ENABLED) || (LOG_HD_ENABLED) )
    #define LOG_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (LOG_RX_ENABLED) || (LOG_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define LOG_WAIT_1_MS      LOG_BL_CHK_DELAY_MS   

#define LOG_TXBUFFERSIZE   LOG_TX_BUFFER_SIZE
#define LOG_RXBUFFERSIZE   LOG_RX_BUFFER_SIZE

#if (LOG_RXHW_ADDRESS_ENABLED)
    #define LOG_RXADDRESSMODE  LOG_RX_ADDRESS_MODE
    #define LOG_RXHWADDRESS1   LOG_RX_HW_ADDRESS1
    #define LOG_RXHWADDRESS2   LOG_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define LOG_RXAddressMode  LOG_RXADDRESSMODE
#endif /* (LOG_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define LOG_initvar                    LOG_initVar

#define LOG_RX_Enabled                 LOG_RX_ENABLED
#define LOG_TX_Enabled                 LOG_TX_ENABLED
#define LOG_HD_Enabled                 LOG_HD_ENABLED
#define LOG_RX_IntInterruptEnabled     LOG_RX_INTERRUPT_ENABLED
#define LOG_TX_IntInterruptEnabled     LOG_TX_INTERRUPT_ENABLED
#define LOG_InternalClockUsed          LOG_INTERNAL_CLOCK_USED
#define LOG_RXHW_Address_Enabled       LOG_RXHW_ADDRESS_ENABLED
#define LOG_OverSampleCount            LOG_OVER_SAMPLE_COUNT
#define LOG_ParityType                 LOG_PARITY_TYPE

#if( LOG_TX_ENABLED && (LOG_TXBUFFERSIZE > LOG_FIFO_LENGTH))
    #define LOG_TXBUFFER               LOG_txBuffer
    #define LOG_TXBUFFERREAD           LOG_txBufferRead
    #define LOG_TXBUFFERWRITE          LOG_txBufferWrite
#endif /* End LOG_TX_ENABLED */
#if( ( LOG_RX_ENABLED || LOG_HD_ENABLED ) && \
     (LOG_RXBUFFERSIZE > LOG_FIFO_LENGTH) )
    #define LOG_RXBUFFER               LOG_rxBuffer
    #define LOG_RXBUFFERREAD           LOG_rxBufferRead
    #define LOG_RXBUFFERWRITE          LOG_rxBufferWrite
    #define LOG_RXBUFFERLOOPDETECT     LOG_rxBufferLoopDetect
    #define LOG_RXBUFFER_OVERFLOW      LOG_rxBufferOverflow
#endif /* End LOG_RX_ENABLED */

#ifdef LOG_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define LOG_CONTROL                LOG_CONTROL_REG
#endif /* End LOG_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(LOG_TX_ENABLED)
    #define LOG_TXDATA                 LOG_TXDATA_REG
    #define LOG_TXSTATUS               LOG_TXSTATUS_REG
    #define LOG_TXSTATUS_MASK          LOG_TXSTATUS_MASK_REG
    #define LOG_TXSTATUS_ACTL          LOG_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(LOG_TXCLKGEN_DP)
        #define LOG_TXBITCLKGEN_CTR        LOG_TXBITCLKGEN_CTR_REG
        #define LOG_TXBITCLKTX_COMPLETE    LOG_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define LOG_TXBITCTR_PERIOD        LOG_TXBITCTR_PERIOD_REG
        #define LOG_TXBITCTR_CONTROL       LOG_TXBITCTR_CONTROL_REG
        #define LOG_TXBITCTR_COUNTER       LOG_TXBITCTR_COUNTER_REG
    #endif /* LOG_TXCLKGEN_DP */
#endif /* End LOG_TX_ENABLED */

#if(LOG_HD_ENABLED)
    #define LOG_TXDATA                 LOG_TXDATA_REG
    #define LOG_TXSTATUS               LOG_TXSTATUS_REG
    #define LOG_TXSTATUS_MASK          LOG_TXSTATUS_MASK_REG
    #define LOG_TXSTATUS_ACTL          LOG_TXSTATUS_ACTL_REG
#endif /* End LOG_HD_ENABLED */

#if( (LOG_RX_ENABLED) || (LOG_HD_ENABLED) )
    #define LOG_RXDATA                 LOG_RXDATA_REG
    #define LOG_RXADDRESS1             LOG_RXADDRESS1_REG
    #define LOG_RXADDRESS2             LOG_RXADDRESS2_REG
    #define LOG_RXBITCTR_PERIOD        LOG_RXBITCTR_PERIOD_REG
    #define LOG_RXBITCTR_CONTROL       LOG_RXBITCTR_CONTROL_REG
    #define LOG_RXBITCTR_COUNTER       LOG_RXBITCTR_COUNTER_REG
    #define LOG_RXSTATUS               LOG_RXSTATUS_REG
    #define LOG_RXSTATUS_MASK          LOG_RXSTATUS_MASK_REG
    #define LOG_RXSTATUS_ACTL          LOG_RXSTATUS_ACTL_REG
#endif /* End  (LOG_RX_ENABLED) || (LOG_HD_ENABLED) */

#if(LOG_INTERNAL_CLOCK_USED)
    #define LOG_INTCLOCK_CLKEN         LOG_INTCLOCK_CLKEN_REG
#endif /* End LOG_INTERNAL_CLOCK_USED */

#define LOG_WAIT_FOR_COMLETE_REINIT    LOG_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_LOG_H */


/* [] END OF FILE */
