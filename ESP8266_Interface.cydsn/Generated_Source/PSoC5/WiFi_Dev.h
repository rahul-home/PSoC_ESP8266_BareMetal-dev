/*******************************************************************************
* File Name: WiFi_Dev.h
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


#if !defined(CY_UART_WiFi_Dev_H)
#define CY_UART_WiFi_Dev_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define WiFi_Dev_RX_ENABLED                     (1u)
#define WiFi_Dev_TX_ENABLED                     (1u)
#define WiFi_Dev_HD_ENABLED                     (0u)
#define WiFi_Dev_RX_INTERRUPT_ENABLED           (1u)
#define WiFi_Dev_TX_INTERRUPT_ENABLED           (1u)
#define WiFi_Dev_INTERNAL_CLOCK_USED            (1u)
#define WiFi_Dev_RXHW_ADDRESS_ENABLED           (0u)
#define WiFi_Dev_OVER_SAMPLE_COUNT              (8u)
#define WiFi_Dev_PARITY_TYPE                    (0u)
#define WiFi_Dev_PARITY_TYPE_SW                 (0u)
#define WiFi_Dev_BREAK_DETECT                   (0u)
#define WiFi_Dev_BREAK_BITS_TX                  (13u)
#define WiFi_Dev_BREAK_BITS_RX                  (13u)
#define WiFi_Dev_TXCLKGEN_DP                    (1u)
#define WiFi_Dev_USE23POLLING                   (1u)
#define WiFi_Dev_FLOW_CONTROL                   (0u)
#define WiFi_Dev_CLK_FREQ                       (0u)
#define WiFi_Dev_TX_BUFFER_SIZE                 (32u)
#define WiFi_Dev_RX_BUFFER_SIZE                 (32u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(WiFi_Dev_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define WiFi_Dev_CONTROL_REG_REMOVED            (0u)
#else
    #define WiFi_Dev_CONTROL_REG_REMOVED            (1u)
#endif /* End WiFi_Dev_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct WiFi_Dev_backupStruct_
{
    uint8 enableState;

    #if(WiFi_Dev_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End WiFi_Dev_CONTROL_REG_REMOVED */

} WiFi_Dev_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void WiFi_Dev_Start(void) ;
void WiFi_Dev_Stop(void) ;
uint8 WiFi_Dev_ReadControlRegister(void) ;
void WiFi_Dev_WriteControlRegister(uint8 control) ;

void WiFi_Dev_Init(void) ;
void WiFi_Dev_Enable(void) ;
void WiFi_Dev_SaveConfig(void) ;
void WiFi_Dev_RestoreConfig(void) ;
void WiFi_Dev_Sleep(void) ;
void WiFi_Dev_Wakeup(void) ;

/* Only if RX is enabled */
#if( (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) )

    #if (WiFi_Dev_RX_INTERRUPT_ENABLED)
        #define WiFi_Dev_EnableRxInt()  CyIntEnable (WiFi_Dev_RX_VECT_NUM)
        #define WiFi_Dev_DisableRxInt() CyIntDisable(WiFi_Dev_RX_VECT_NUM)
        CY_ISR_PROTO(WiFi_Dev_RXISR);
    #endif /* WiFi_Dev_RX_INTERRUPT_ENABLED */

    void WiFi_Dev_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void WiFi_Dev_SetRxAddress1(uint8 address) ;
    void WiFi_Dev_SetRxAddress2(uint8 address) ;

    void  WiFi_Dev_SetRxInterruptMode(uint8 intSrc) ;
    uint8 WiFi_Dev_ReadRxData(void) ;
    uint8 WiFi_Dev_ReadRxStatus(void) ;
    uint8 WiFi_Dev_GetChar(void) ;
    uint16 WiFi_Dev_GetByte(void) ;
    uint8 WiFi_Dev_GetRxBufferSize(void)
                                                            ;
    void WiFi_Dev_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define WiFi_Dev_GetRxInterruptSource   WiFi_Dev_ReadRxStatus

#endif /* End (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) */

/* Only if TX is enabled */
#if(WiFi_Dev_TX_ENABLED || WiFi_Dev_HD_ENABLED)

    #if(WiFi_Dev_TX_INTERRUPT_ENABLED)
        #define WiFi_Dev_EnableTxInt()  CyIntEnable (WiFi_Dev_TX_VECT_NUM)
        #define WiFi_Dev_DisableTxInt() CyIntDisable(WiFi_Dev_TX_VECT_NUM)
        #define WiFi_Dev_SetPendingTxInt() CyIntSetPending(WiFi_Dev_TX_VECT_NUM)
        #define WiFi_Dev_ClearPendingTxInt() CyIntClearPending(WiFi_Dev_TX_VECT_NUM)
        CY_ISR_PROTO(WiFi_Dev_TXISR);
    #endif /* WiFi_Dev_TX_INTERRUPT_ENABLED */

    void WiFi_Dev_SetTxInterruptMode(uint8 intSrc) ;
    void WiFi_Dev_WriteTxData(uint8 txDataByte) ;
    uint8 WiFi_Dev_ReadTxStatus(void) ;
    void WiFi_Dev_PutChar(uint8 txDataByte) ;
    void WiFi_Dev_PutString(const char8 string[]) ;
    void WiFi_Dev_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void WiFi_Dev_PutCRLF(uint8 txDataByte) ;
    void WiFi_Dev_ClearTxBuffer(void) ;
    void WiFi_Dev_SetTxAddressMode(uint8 addressMode) ;
    void WiFi_Dev_SendBreak(uint8 retMode) ;
    uint8 WiFi_Dev_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define WiFi_Dev_PutStringConst         WiFi_Dev_PutString
    #define WiFi_Dev_PutArrayConst          WiFi_Dev_PutArray
    #define WiFi_Dev_GetTxInterruptSource   WiFi_Dev_ReadTxStatus

#endif /* End WiFi_Dev_TX_ENABLED || WiFi_Dev_HD_ENABLED */

#if(WiFi_Dev_HD_ENABLED)
    void WiFi_Dev_LoadRxConfig(void) ;
    void WiFi_Dev_LoadTxConfig(void) ;
#endif /* End WiFi_Dev_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_WiFi_Dev) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    WiFi_Dev_CyBtldrCommStart(void) CYSMALL ;
    void    WiFi_Dev_CyBtldrCommStop(void) CYSMALL ;
    void    WiFi_Dev_CyBtldrCommReset(void) CYSMALL ;
    cystatus WiFi_Dev_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus WiFi_Dev_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_WiFi_Dev)
        #define CyBtldrCommStart    WiFi_Dev_CyBtldrCommStart
        #define CyBtldrCommStop     WiFi_Dev_CyBtldrCommStop
        #define CyBtldrCommReset    WiFi_Dev_CyBtldrCommReset
        #define CyBtldrCommWrite    WiFi_Dev_CyBtldrCommWrite
        #define CyBtldrCommRead     WiFi_Dev_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_WiFi_Dev) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define WiFi_Dev_BYTE2BYTE_TIME_OUT (25u)
    #define WiFi_Dev_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define WiFi_Dev_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define WiFi_Dev_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define WiFi_Dev_SET_SPACE      (0x00u)
#define WiFi_Dev_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (WiFi_Dev_TX_ENABLED) || (WiFi_Dev_HD_ENABLED) )
    #if(WiFi_Dev_TX_INTERRUPT_ENABLED)
        #define WiFi_Dev_TX_VECT_NUM            (uint8)WiFi_Dev_TXInternalInterrupt__INTC_NUMBER
        #define WiFi_Dev_TX_PRIOR_NUM           (uint8)WiFi_Dev_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* WiFi_Dev_TX_INTERRUPT_ENABLED */

    #define WiFi_Dev_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define WiFi_Dev_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define WiFi_Dev_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(WiFi_Dev_TX_ENABLED)
        #define WiFi_Dev_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (WiFi_Dev_HD_ENABLED) */
        #define WiFi_Dev_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (WiFi_Dev_TX_ENABLED) */

    #define WiFi_Dev_TX_STS_COMPLETE            (uint8)(0x01u << WiFi_Dev_TX_STS_COMPLETE_SHIFT)
    #define WiFi_Dev_TX_STS_FIFO_EMPTY          (uint8)(0x01u << WiFi_Dev_TX_STS_FIFO_EMPTY_SHIFT)
    #define WiFi_Dev_TX_STS_FIFO_FULL           (uint8)(0x01u << WiFi_Dev_TX_STS_FIFO_FULL_SHIFT)
    #define WiFi_Dev_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << WiFi_Dev_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (WiFi_Dev_TX_ENABLED) || (WiFi_Dev_HD_ENABLED)*/

#if( (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) )
    #if(WiFi_Dev_RX_INTERRUPT_ENABLED)
        #define WiFi_Dev_RX_VECT_NUM            (uint8)WiFi_Dev_RXInternalInterrupt__INTC_NUMBER
        #define WiFi_Dev_RX_PRIOR_NUM           (uint8)WiFi_Dev_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* WiFi_Dev_RX_INTERRUPT_ENABLED */
    #define WiFi_Dev_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define WiFi_Dev_RX_STS_BREAK_SHIFT             (0x01u)
    #define WiFi_Dev_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define WiFi_Dev_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define WiFi_Dev_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define WiFi_Dev_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define WiFi_Dev_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define WiFi_Dev_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define WiFi_Dev_RX_STS_MRKSPC           (uint8)(0x01u << WiFi_Dev_RX_STS_MRKSPC_SHIFT)
    #define WiFi_Dev_RX_STS_BREAK            (uint8)(0x01u << WiFi_Dev_RX_STS_BREAK_SHIFT)
    #define WiFi_Dev_RX_STS_PAR_ERROR        (uint8)(0x01u << WiFi_Dev_RX_STS_PAR_ERROR_SHIFT)
    #define WiFi_Dev_RX_STS_STOP_ERROR       (uint8)(0x01u << WiFi_Dev_RX_STS_STOP_ERROR_SHIFT)
    #define WiFi_Dev_RX_STS_OVERRUN          (uint8)(0x01u << WiFi_Dev_RX_STS_OVERRUN_SHIFT)
    #define WiFi_Dev_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << WiFi_Dev_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define WiFi_Dev_RX_STS_ADDR_MATCH       (uint8)(0x01u << WiFi_Dev_RX_STS_ADDR_MATCH_SHIFT)
    #define WiFi_Dev_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << WiFi_Dev_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define WiFi_Dev_RX_HW_MASK                     (0x7Fu)
#endif /* End (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) */

/* Control Register definitions */
#define WiFi_Dev_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define WiFi_Dev_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define WiFi_Dev_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define WiFi_Dev_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define WiFi_Dev_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define WiFi_Dev_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define WiFi_Dev_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define WiFi_Dev_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define WiFi_Dev_CTRL_HD_SEND               (uint8)(0x01u << WiFi_Dev_CTRL_HD_SEND_SHIFT)
#define WiFi_Dev_CTRL_HD_SEND_BREAK         (uint8)(0x01u << WiFi_Dev_CTRL_HD_SEND_BREAK_SHIFT)
#define WiFi_Dev_CTRL_MARK                  (uint8)(0x01u << WiFi_Dev_CTRL_MARK_SHIFT)
#define WiFi_Dev_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << WiFi_Dev_CTRL_PARITY_TYPE0_SHIFT)
#define WiFi_Dev_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << WiFi_Dev_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define WiFi_Dev_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define WiFi_Dev_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define WiFi_Dev_SEND_BREAK                         (0x00u)
#define WiFi_Dev_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define WiFi_Dev_REINIT                             (0x02u)
#define WiFi_Dev_SEND_WAIT_REINIT                   (0x03u)

#define WiFi_Dev_OVER_SAMPLE_8                      (8u)
#define WiFi_Dev_OVER_SAMPLE_16                     (16u)

#define WiFi_Dev_BIT_CENTER                         (WiFi_Dev_OVER_SAMPLE_COUNT - 2u)

#define WiFi_Dev_FIFO_LENGTH                        (4u)
#define WiFi_Dev_NUMBER_OF_START_BIT                (1u)
#define WiFi_Dev_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define WiFi_Dev_TXBITCTR_BREAKBITS8X   ((WiFi_Dev_BREAK_BITS_TX * WiFi_Dev_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define WiFi_Dev_TXBITCTR_BREAKBITS ((WiFi_Dev_BREAK_BITS_TX * WiFi_Dev_OVER_SAMPLE_COUNT) - 1u)

#define WiFi_Dev_HALF_BIT_COUNT   \
                            (((WiFi_Dev_OVER_SAMPLE_COUNT / 2u) + (WiFi_Dev_USE23POLLING * 1u)) - 2u)
#if (WiFi_Dev_OVER_SAMPLE_COUNT == WiFi_Dev_OVER_SAMPLE_8)
    #define WiFi_Dev_HD_TXBITCTR_INIT   (((WiFi_Dev_BREAK_BITS_TX + \
                            WiFi_Dev_NUMBER_OF_START_BIT) * WiFi_Dev_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define WiFi_Dev_RXBITCTR_INIT  ((((WiFi_Dev_BREAK_BITS_RX + WiFi_Dev_NUMBER_OF_START_BIT) \
                            * WiFi_Dev_OVER_SAMPLE_COUNT) + WiFi_Dev_HALF_BIT_COUNT) - 1u)

#else /* WiFi_Dev_OVER_SAMPLE_COUNT == WiFi_Dev_OVER_SAMPLE_16 */
    #define WiFi_Dev_HD_TXBITCTR_INIT   ((8u * WiFi_Dev_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define WiFi_Dev_RXBITCTR_INIT      (((7u * WiFi_Dev_OVER_SAMPLE_COUNT) - 1u) + \
                                                      WiFi_Dev_HALF_BIT_COUNT)
#endif /* End WiFi_Dev_OVER_SAMPLE_COUNT */

#define WiFi_Dev_HD_RXBITCTR_INIT                   WiFi_Dev_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 WiFi_Dev_initVar;
#if (WiFi_Dev_TX_INTERRUPT_ENABLED && WiFi_Dev_TX_ENABLED)
    extern volatile uint8 WiFi_Dev_txBuffer[WiFi_Dev_TX_BUFFER_SIZE];
    extern volatile uint8 WiFi_Dev_txBufferRead;
    extern uint8 WiFi_Dev_txBufferWrite;
#endif /* (WiFi_Dev_TX_INTERRUPT_ENABLED && WiFi_Dev_TX_ENABLED) */
#if (WiFi_Dev_RX_INTERRUPT_ENABLED && (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED))
    extern uint8 WiFi_Dev_errorStatus;
    extern volatile uint8 WiFi_Dev_rxBuffer[WiFi_Dev_RX_BUFFER_SIZE];
    extern volatile uint8 WiFi_Dev_rxBufferRead;
    extern volatile uint8 WiFi_Dev_rxBufferWrite;
    extern volatile uint8 WiFi_Dev_rxBufferLoopDetect;
    extern volatile uint8 WiFi_Dev_rxBufferOverflow;
    #if (WiFi_Dev_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 WiFi_Dev_rxAddressMode;
        extern volatile uint8 WiFi_Dev_rxAddressDetected;
    #endif /* (WiFi_Dev_RXHW_ADDRESS_ENABLED) */
#endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED && (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define WiFi_Dev__B_UART__AM_SW_BYTE_BYTE 1
#define WiFi_Dev__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define WiFi_Dev__B_UART__AM_HW_BYTE_BY_BYTE 3
#define WiFi_Dev__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define WiFi_Dev__B_UART__AM_NONE 0

#define WiFi_Dev__B_UART__NONE_REVB 0
#define WiFi_Dev__B_UART__EVEN_REVB 1
#define WiFi_Dev__B_UART__ODD_REVB 2
#define WiFi_Dev__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define WiFi_Dev_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define WiFi_Dev_NUMBER_OF_STOP_BITS    (1u)

#if (WiFi_Dev_RXHW_ADDRESS_ENABLED)
    #define WiFi_Dev_RX_ADDRESS_MODE    (0u)
    #define WiFi_Dev_RX_HW_ADDRESS1     (0u)
    #define WiFi_Dev_RX_HW_ADDRESS2     (0u)
#endif /* (WiFi_Dev_RXHW_ADDRESS_ENABLED) */

#define WiFi_Dev_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << WiFi_Dev_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << WiFi_Dev_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << WiFi_Dev_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << WiFi_Dev_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << WiFi_Dev_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << WiFi_Dev_RX_STS_BREAK_SHIFT) \
                                        | (0 << WiFi_Dev_RX_STS_OVERRUN_SHIFT))

#define WiFi_Dev_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << WiFi_Dev_TX_STS_COMPLETE_SHIFT) \
                                        | (1 << WiFi_Dev_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << WiFi_Dev_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << WiFi_Dev_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef WiFi_Dev_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define WiFi_Dev_CONTROL_REG \
                            (* (reg8 *) WiFi_Dev_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define WiFi_Dev_CONTROL_PTR \
                            (  (reg8 *) WiFi_Dev_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End WiFi_Dev_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(WiFi_Dev_TX_ENABLED)
    #define WiFi_Dev_TXDATA_REG          (* (reg8 *) WiFi_Dev_BUART_sTX_TxShifter_u0__F0_REG)
    #define WiFi_Dev_TXDATA_PTR          (  (reg8 *) WiFi_Dev_BUART_sTX_TxShifter_u0__F0_REG)
    #define WiFi_Dev_TXDATA_AUX_CTL_REG  (* (reg8 *) WiFi_Dev_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define WiFi_Dev_TXDATA_AUX_CTL_PTR  (  (reg8 *) WiFi_Dev_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define WiFi_Dev_TXSTATUS_REG        (* (reg8 *) WiFi_Dev_BUART_sTX_TxSts__STATUS_REG)
    #define WiFi_Dev_TXSTATUS_PTR        (  (reg8 *) WiFi_Dev_BUART_sTX_TxSts__STATUS_REG)
    #define WiFi_Dev_TXSTATUS_MASK_REG   (* (reg8 *) WiFi_Dev_BUART_sTX_TxSts__MASK_REG)
    #define WiFi_Dev_TXSTATUS_MASK_PTR   (  (reg8 *) WiFi_Dev_BUART_sTX_TxSts__MASK_REG)
    #define WiFi_Dev_TXSTATUS_ACTL_REG   (* (reg8 *) WiFi_Dev_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define WiFi_Dev_TXSTATUS_ACTL_PTR   (  (reg8 *) WiFi_Dev_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(WiFi_Dev_TXCLKGEN_DP)
        #define WiFi_Dev_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) WiFi_Dev_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define WiFi_Dev_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) WiFi_Dev_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define WiFi_Dev_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) WiFi_Dev_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define WiFi_Dev_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) WiFi_Dev_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define WiFi_Dev_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) WiFi_Dev_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define WiFi_Dev_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) WiFi_Dev_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define WiFi_Dev_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) WiFi_Dev_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define WiFi_Dev_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) WiFi_Dev_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define WiFi_Dev_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) WiFi_Dev_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define WiFi_Dev_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) WiFi_Dev_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* WiFi_Dev_TXCLKGEN_DP */

#endif /* End WiFi_Dev_TX_ENABLED */

#if(WiFi_Dev_HD_ENABLED)

    #define WiFi_Dev_TXDATA_REG             (* (reg8 *) WiFi_Dev_BUART_sRX_RxShifter_u0__F1_REG )
    #define WiFi_Dev_TXDATA_PTR             (  (reg8 *) WiFi_Dev_BUART_sRX_RxShifter_u0__F1_REG )
    #define WiFi_Dev_TXDATA_AUX_CTL_REG     (* (reg8 *) WiFi_Dev_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define WiFi_Dev_TXDATA_AUX_CTL_PTR     (  (reg8 *) WiFi_Dev_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define WiFi_Dev_TXSTATUS_REG           (* (reg8 *) WiFi_Dev_BUART_sRX_RxSts__STATUS_REG )
    #define WiFi_Dev_TXSTATUS_PTR           (  (reg8 *) WiFi_Dev_BUART_sRX_RxSts__STATUS_REG )
    #define WiFi_Dev_TXSTATUS_MASK_REG      (* (reg8 *) WiFi_Dev_BUART_sRX_RxSts__MASK_REG )
    #define WiFi_Dev_TXSTATUS_MASK_PTR      (  (reg8 *) WiFi_Dev_BUART_sRX_RxSts__MASK_REG )
    #define WiFi_Dev_TXSTATUS_ACTL_REG      (* (reg8 *) WiFi_Dev_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define WiFi_Dev_TXSTATUS_ACTL_PTR      (  (reg8 *) WiFi_Dev_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End WiFi_Dev_HD_ENABLED */

#if( (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) )
    #define WiFi_Dev_RXDATA_REG             (* (reg8 *) WiFi_Dev_BUART_sRX_RxShifter_u0__F0_REG )
    #define WiFi_Dev_RXDATA_PTR             (  (reg8 *) WiFi_Dev_BUART_sRX_RxShifter_u0__F0_REG )
    #define WiFi_Dev_RXADDRESS1_REG         (* (reg8 *) WiFi_Dev_BUART_sRX_RxShifter_u0__D0_REG )
    #define WiFi_Dev_RXADDRESS1_PTR         (  (reg8 *) WiFi_Dev_BUART_sRX_RxShifter_u0__D0_REG )
    #define WiFi_Dev_RXADDRESS2_REG         (* (reg8 *) WiFi_Dev_BUART_sRX_RxShifter_u0__D1_REG )
    #define WiFi_Dev_RXADDRESS2_PTR         (  (reg8 *) WiFi_Dev_BUART_sRX_RxShifter_u0__D1_REG )
    #define WiFi_Dev_RXDATA_AUX_CTL_REG     (* (reg8 *) WiFi_Dev_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define WiFi_Dev_RXBITCTR_PERIOD_REG    (* (reg8 *) WiFi_Dev_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define WiFi_Dev_RXBITCTR_PERIOD_PTR    (  (reg8 *) WiFi_Dev_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define WiFi_Dev_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) WiFi_Dev_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define WiFi_Dev_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) WiFi_Dev_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define WiFi_Dev_RXBITCTR_COUNTER_REG   (* (reg8 *) WiFi_Dev_BUART_sRX_RxBitCounter__COUNT_REG )
    #define WiFi_Dev_RXBITCTR_COUNTER_PTR   (  (reg8 *) WiFi_Dev_BUART_sRX_RxBitCounter__COUNT_REG )

    #define WiFi_Dev_RXSTATUS_REG           (* (reg8 *) WiFi_Dev_BUART_sRX_RxSts__STATUS_REG )
    #define WiFi_Dev_RXSTATUS_PTR           (  (reg8 *) WiFi_Dev_BUART_sRX_RxSts__STATUS_REG )
    #define WiFi_Dev_RXSTATUS_MASK_REG      (* (reg8 *) WiFi_Dev_BUART_sRX_RxSts__MASK_REG )
    #define WiFi_Dev_RXSTATUS_MASK_PTR      (  (reg8 *) WiFi_Dev_BUART_sRX_RxSts__MASK_REG )
    #define WiFi_Dev_RXSTATUS_ACTL_REG      (* (reg8 *) WiFi_Dev_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define WiFi_Dev_RXSTATUS_ACTL_PTR      (  (reg8 *) WiFi_Dev_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) */

#if(WiFi_Dev_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define WiFi_Dev_INTCLOCK_CLKEN_REG     (* (reg8 *) WiFi_Dev_IntClock__PM_ACT_CFG)
    #define WiFi_Dev_INTCLOCK_CLKEN_PTR     (  (reg8 *) WiFi_Dev_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define WiFi_Dev_INTCLOCK_CLKEN_MASK    WiFi_Dev_IntClock__PM_ACT_MSK
#endif /* End WiFi_Dev_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(WiFi_Dev_TX_ENABLED)
    #define WiFi_Dev_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End WiFi_Dev_TX_ENABLED */

#if(WiFi_Dev_HD_ENABLED)
    #define WiFi_Dev_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End WiFi_Dev_HD_ENABLED */

#if( (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) )
    #define WiFi_Dev_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define WiFi_Dev_WAIT_1_MS      WiFi_Dev_BL_CHK_DELAY_MS   

#define WiFi_Dev_TXBUFFERSIZE   WiFi_Dev_TX_BUFFER_SIZE
#define WiFi_Dev_RXBUFFERSIZE   WiFi_Dev_RX_BUFFER_SIZE

#if (WiFi_Dev_RXHW_ADDRESS_ENABLED)
    #define WiFi_Dev_RXADDRESSMODE  WiFi_Dev_RX_ADDRESS_MODE
    #define WiFi_Dev_RXHWADDRESS1   WiFi_Dev_RX_HW_ADDRESS1
    #define WiFi_Dev_RXHWADDRESS2   WiFi_Dev_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define WiFi_Dev_RXAddressMode  WiFi_Dev_RXADDRESSMODE
#endif /* (WiFi_Dev_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define WiFi_Dev_initvar                    WiFi_Dev_initVar

#define WiFi_Dev_RX_Enabled                 WiFi_Dev_RX_ENABLED
#define WiFi_Dev_TX_Enabled                 WiFi_Dev_TX_ENABLED
#define WiFi_Dev_HD_Enabled                 WiFi_Dev_HD_ENABLED
#define WiFi_Dev_RX_IntInterruptEnabled     WiFi_Dev_RX_INTERRUPT_ENABLED
#define WiFi_Dev_TX_IntInterruptEnabled     WiFi_Dev_TX_INTERRUPT_ENABLED
#define WiFi_Dev_InternalClockUsed          WiFi_Dev_INTERNAL_CLOCK_USED
#define WiFi_Dev_RXHW_Address_Enabled       WiFi_Dev_RXHW_ADDRESS_ENABLED
#define WiFi_Dev_OverSampleCount            WiFi_Dev_OVER_SAMPLE_COUNT
#define WiFi_Dev_ParityType                 WiFi_Dev_PARITY_TYPE

#if( WiFi_Dev_TX_ENABLED && (WiFi_Dev_TXBUFFERSIZE > WiFi_Dev_FIFO_LENGTH))
    #define WiFi_Dev_TXBUFFER               WiFi_Dev_txBuffer
    #define WiFi_Dev_TXBUFFERREAD           WiFi_Dev_txBufferRead
    #define WiFi_Dev_TXBUFFERWRITE          WiFi_Dev_txBufferWrite
#endif /* End WiFi_Dev_TX_ENABLED */
#if( ( WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED ) && \
     (WiFi_Dev_RXBUFFERSIZE > WiFi_Dev_FIFO_LENGTH) )
    #define WiFi_Dev_RXBUFFER               WiFi_Dev_rxBuffer
    #define WiFi_Dev_RXBUFFERREAD           WiFi_Dev_rxBufferRead
    #define WiFi_Dev_RXBUFFERWRITE          WiFi_Dev_rxBufferWrite
    #define WiFi_Dev_RXBUFFERLOOPDETECT     WiFi_Dev_rxBufferLoopDetect
    #define WiFi_Dev_RXBUFFER_OVERFLOW      WiFi_Dev_rxBufferOverflow
#endif /* End WiFi_Dev_RX_ENABLED */

#ifdef WiFi_Dev_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define WiFi_Dev_CONTROL                WiFi_Dev_CONTROL_REG
#endif /* End WiFi_Dev_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(WiFi_Dev_TX_ENABLED)
    #define WiFi_Dev_TXDATA                 WiFi_Dev_TXDATA_REG
    #define WiFi_Dev_TXSTATUS               WiFi_Dev_TXSTATUS_REG
    #define WiFi_Dev_TXSTATUS_MASK          WiFi_Dev_TXSTATUS_MASK_REG
    #define WiFi_Dev_TXSTATUS_ACTL          WiFi_Dev_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(WiFi_Dev_TXCLKGEN_DP)
        #define WiFi_Dev_TXBITCLKGEN_CTR        WiFi_Dev_TXBITCLKGEN_CTR_REG
        #define WiFi_Dev_TXBITCLKTX_COMPLETE    WiFi_Dev_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define WiFi_Dev_TXBITCTR_PERIOD        WiFi_Dev_TXBITCTR_PERIOD_REG
        #define WiFi_Dev_TXBITCTR_CONTROL       WiFi_Dev_TXBITCTR_CONTROL_REG
        #define WiFi_Dev_TXBITCTR_COUNTER       WiFi_Dev_TXBITCTR_COUNTER_REG
    #endif /* WiFi_Dev_TXCLKGEN_DP */
#endif /* End WiFi_Dev_TX_ENABLED */

#if(WiFi_Dev_HD_ENABLED)
    #define WiFi_Dev_TXDATA                 WiFi_Dev_TXDATA_REG
    #define WiFi_Dev_TXSTATUS               WiFi_Dev_TXSTATUS_REG
    #define WiFi_Dev_TXSTATUS_MASK          WiFi_Dev_TXSTATUS_MASK_REG
    #define WiFi_Dev_TXSTATUS_ACTL          WiFi_Dev_TXSTATUS_ACTL_REG
#endif /* End WiFi_Dev_HD_ENABLED */

#if( (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) )
    #define WiFi_Dev_RXDATA                 WiFi_Dev_RXDATA_REG
    #define WiFi_Dev_RXADDRESS1             WiFi_Dev_RXADDRESS1_REG
    #define WiFi_Dev_RXADDRESS2             WiFi_Dev_RXADDRESS2_REG
    #define WiFi_Dev_RXBITCTR_PERIOD        WiFi_Dev_RXBITCTR_PERIOD_REG
    #define WiFi_Dev_RXBITCTR_CONTROL       WiFi_Dev_RXBITCTR_CONTROL_REG
    #define WiFi_Dev_RXBITCTR_COUNTER       WiFi_Dev_RXBITCTR_COUNTER_REG
    #define WiFi_Dev_RXSTATUS               WiFi_Dev_RXSTATUS_REG
    #define WiFi_Dev_RXSTATUS_MASK          WiFi_Dev_RXSTATUS_MASK_REG
    #define WiFi_Dev_RXSTATUS_ACTL          WiFi_Dev_RXSTATUS_ACTL_REG
#endif /* End  (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) */

#if(WiFi_Dev_INTERNAL_CLOCK_USED)
    #define WiFi_Dev_INTCLOCK_CLKEN         WiFi_Dev_INTCLOCK_CLKEN_REG
#endif /* End WiFi_Dev_INTERNAL_CLOCK_USED */

#define WiFi_Dev_WAIT_FOR_COMLETE_REINIT    WiFi_Dev_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_WiFi_Dev_H */


/* [] END OF FILE */
