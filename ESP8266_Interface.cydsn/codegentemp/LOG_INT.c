/*******************************************************************************
* File Name: LOGINT.c
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

#include "LOG.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (LOG_RX_INTERRUPT_ENABLED && (LOG_RX_ENABLED || LOG_HD_ENABLED))
    /*******************************************************************************
    * Function Name: LOG_RXISR
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
    *  LOG_rxBuffer - RAM buffer pointer for save received data.
    *  LOG_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  LOG_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  LOG_rxBufferOverflow - software overflow flag. Set to one
    *     when LOG_rxBufferWrite index overtakes
    *     LOG_rxBufferRead index.
    *  LOG_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when LOG_rxBufferWrite is equal to
    *    LOG_rxBufferRead
    *  LOG_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  LOG_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(LOG_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef LOG_RXISR_ENTRY_CALLBACK
        LOG_RXISR_EntryCallback();
    #endif /* LOG_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START LOG_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = LOG_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in LOG_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (LOG_RX_STS_BREAK | 
                            LOG_RX_STS_PAR_ERROR |
                            LOG_RX_STS_STOP_ERROR | 
                            LOG_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                LOG_errorStatus |= readStatus & ( LOG_RX_STS_BREAK | 
                                                            LOG_RX_STS_PAR_ERROR | 
                                                            LOG_RX_STS_STOP_ERROR | 
                                                            LOG_RX_STS_OVERRUN);
                /* `#START LOG_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef LOG_RXISR_ERROR_CALLBACK
                LOG_RXISR_ERROR_Callback();
            #endif /* LOG_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & LOG_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = LOG_RXDATA_REG;
            #if (LOG_RXHW_ADDRESS_ENABLED)
                if(LOG_rxAddressMode == (uint8)LOG__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & LOG_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & LOG_RX_STS_ADDR_MATCH) != 0u)
                        {
                            LOG_rxAddressDetected = 1u;
                        }
                        else
                        {
                            LOG_rxAddressDetected = 0u;
                        }
                    }
                    if(LOG_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        LOG_rxBuffer[LOG_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    LOG_rxBuffer[LOG_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                LOG_rxBuffer[LOG_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (LOG_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(LOG_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        LOG_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    LOG_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(LOG_rxBufferWrite >= LOG_RX_BUFFER_SIZE)
                    {
                        LOG_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(LOG_rxBufferWrite == LOG_rxBufferRead)
                    {
                        LOG_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (LOG_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            LOG_RXSTATUS_MASK_REG  &= (uint8)~LOG_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(LOG_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (LOG_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & LOG_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START LOG_RXISR_END` */

        /* `#END` */

    #ifdef LOG_RXISR_EXIT_CALLBACK
        LOG_RXISR_ExitCallback();
    #endif /* LOG_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (LOG_RX_INTERRUPT_ENABLED && (LOG_RX_ENABLED || LOG_HD_ENABLED)) */


#if (LOG_TX_INTERRUPT_ENABLED && LOG_TX_ENABLED)
    /*******************************************************************************
    * Function Name: LOG_TXISR
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
    *  LOG_txBuffer - RAM buffer pointer for transmit data from.
    *  LOG_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  LOG_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(LOG_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef LOG_TXISR_ENTRY_CALLBACK
        LOG_TXISR_EntryCallback();
    #endif /* LOG_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START LOG_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((LOG_txBufferRead != LOG_txBufferWrite) &&
             ((LOG_TXSTATUS_REG & LOG_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(LOG_txBufferRead >= LOG_TX_BUFFER_SIZE)
            {
                LOG_txBufferRead = 0u;
            }

            LOG_TXDATA_REG = LOG_txBuffer[LOG_txBufferRead];

            /* Set next pointer */
            LOG_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START LOG_TXISR_END` */

        /* `#END` */

    #ifdef LOG_TXISR_EXIT_CALLBACK
        LOG_TXISR_ExitCallback();
    #endif /* LOG_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (LOG_TX_INTERRUPT_ENABLED && LOG_TX_ENABLED) */


/* [] END OF FILE */
