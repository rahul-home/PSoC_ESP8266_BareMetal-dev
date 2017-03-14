/*******************************************************************************
* File Name: WIFIdev_ChannelINT.c
* Version 2.50
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "WIFIdev_Channel.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (WIFIdev_Channel_RX_INTERRUPT_ENABLED && (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED))
    /*******************************************************************************
    * Function Name: WIFIdev_Channel_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WIFIdev_Channel_rxBuffer - RAM buffer pointer for save received data.
    *  WIFIdev_Channel_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  WIFIdev_Channel_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  WIFIdev_Channel_rxBufferOverflow - software overflow flag. Set to one
    *     when WIFIdev_Channel_rxBufferWrite index overtakes
    *     WIFIdev_Channel_rxBufferRead index.
    *  WIFIdev_Channel_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when WIFIdev_Channel_rxBufferWrite is equal to
    *    WIFIdev_Channel_rxBufferRead
    *  WIFIdev_Channel_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  WIFIdev_Channel_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(WIFIdev_Channel_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef WIFIdev_Channel_RXISR_ENTRY_CALLBACK
        WIFIdev_Channel_RXISR_EntryCallback();
    #endif /* WIFIdev_Channel_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START WIFIdev_Channel_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = WIFIdev_Channel_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in WIFIdev_Channel_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (WIFIdev_Channel_RX_STS_BREAK | 
                            WIFIdev_Channel_RX_STS_PAR_ERROR |
                            WIFIdev_Channel_RX_STS_STOP_ERROR | 
                            WIFIdev_Channel_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                WIFIdev_Channel_errorStatus |= readStatus & ( WIFIdev_Channel_RX_STS_BREAK | 
                                                            WIFIdev_Channel_RX_STS_PAR_ERROR | 
                                                            WIFIdev_Channel_RX_STS_STOP_ERROR | 
                                                            WIFIdev_Channel_RX_STS_OVERRUN);
                /* `#START WIFIdev_Channel_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef WIFIdev_Channel_RXISR_ERROR_CALLBACK
                WIFIdev_Channel_RXISR_ERROR_Callback();
            #endif /* WIFIdev_Channel_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = WIFIdev_Channel_RXDATA_REG;
            #if (WIFIdev_Channel_RXHW_ADDRESS_ENABLED)
                if(WIFIdev_Channel_rxAddressMode == (uint8)WIFIdev_Channel__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & WIFIdev_Channel_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & WIFIdev_Channel_RX_STS_ADDR_MATCH) != 0u)
                        {
                            WIFIdev_Channel_rxAddressDetected = 1u;
                        }
                        else
                        {
                            WIFIdev_Channel_rxAddressDetected = 0u;
                        }
                    }
                    if(WIFIdev_Channel_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        WIFIdev_Channel_rxBuffer[WIFIdev_Channel_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    WIFIdev_Channel_rxBuffer[WIFIdev_Channel_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                WIFIdev_Channel_rxBuffer[WIFIdev_Channel_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (WIFIdev_Channel_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(WIFIdev_Channel_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        WIFIdev_Channel_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    WIFIdev_Channel_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(WIFIdev_Channel_rxBufferWrite >= WIFIdev_Channel_RX_BUFFER_SIZE)
                    {
                        WIFIdev_Channel_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(WIFIdev_Channel_rxBufferWrite == WIFIdev_Channel_rxBufferRead)
                    {
                        WIFIdev_Channel_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (WIFIdev_Channel_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            WIFIdev_Channel_RXSTATUS_MASK_REG  &= (uint8)~WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(WIFIdev_Channel_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (WIFIdev_Channel_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START WIFIdev_Channel_RXISR_END` */

        /* `#END` */

    #ifdef WIFIdev_Channel_RXISR_EXIT_CALLBACK
        WIFIdev_Channel_RXISR_ExitCallback();
    #endif /* WIFIdev_Channel_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED && (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED)) */


#if (WIFIdev_Channel_TX_INTERRUPT_ENABLED && WIFIdev_Channel_TX_ENABLED)
    /*******************************************************************************
    * Function Name: WIFIdev_Channel_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WIFIdev_Channel_txBuffer - RAM buffer pointer for transmit data from.
    *  WIFIdev_Channel_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  WIFIdev_Channel_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(WIFIdev_Channel_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef WIFIdev_Channel_TXISR_ENTRY_CALLBACK
        WIFIdev_Channel_TXISR_EntryCallback();
    #endif /* WIFIdev_Channel_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START WIFIdev_Channel_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((WIFIdev_Channel_txBufferRead != WIFIdev_Channel_txBufferWrite) &&
             ((WIFIdev_Channel_TXSTATUS_REG & WIFIdev_Channel_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(WIFIdev_Channel_txBufferRead >= WIFIdev_Channel_TX_BUFFER_SIZE)
            {
                WIFIdev_Channel_txBufferRead = 0u;
            }

            WIFIdev_Channel_TXDATA_REG = WIFIdev_Channel_txBuffer[WIFIdev_Channel_txBufferRead];

            /* Set next pointer */
            WIFIdev_Channel_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START WIFIdev_Channel_TXISR_END` */

        /* `#END` */

    #ifdef WIFIdev_Channel_TXISR_EXIT_CALLBACK
        WIFIdev_Channel_TXISR_ExitCallback();
    #endif /* WIFIdev_Channel_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (WIFIdev_Channel_TX_INTERRUPT_ENABLED && WIFIdev_Channel_TX_ENABLED) */


/* [] END OF FILE */
