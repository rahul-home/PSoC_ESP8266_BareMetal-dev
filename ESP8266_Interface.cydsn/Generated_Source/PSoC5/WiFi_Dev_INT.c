/*******************************************************************************
* File Name: WiFi_DevINT.c
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

#include "WiFi_Dev.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (WiFi_Dev_RX_INTERRUPT_ENABLED && (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED))
    /*******************************************************************************
    * Function Name: WiFi_Dev_RXISR
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
    *  WiFi_Dev_rxBuffer - RAM buffer pointer for save received data.
    *  WiFi_Dev_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  WiFi_Dev_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  WiFi_Dev_rxBufferOverflow - software overflow flag. Set to one
    *     when WiFi_Dev_rxBufferWrite index overtakes
    *     WiFi_Dev_rxBufferRead index.
    *  WiFi_Dev_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when WiFi_Dev_rxBufferWrite is equal to
    *    WiFi_Dev_rxBufferRead
    *  WiFi_Dev_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  WiFi_Dev_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(WiFi_Dev_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef WiFi_Dev_RXISR_ENTRY_CALLBACK
        WiFi_Dev_RXISR_EntryCallback();
    #endif /* WiFi_Dev_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START WiFi_Dev_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = WiFi_Dev_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in WiFi_Dev_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (WiFi_Dev_RX_STS_BREAK | 
                            WiFi_Dev_RX_STS_PAR_ERROR |
                            WiFi_Dev_RX_STS_STOP_ERROR | 
                            WiFi_Dev_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                WiFi_Dev_errorStatus |= readStatus & ( WiFi_Dev_RX_STS_BREAK | 
                                                            WiFi_Dev_RX_STS_PAR_ERROR | 
                                                            WiFi_Dev_RX_STS_STOP_ERROR | 
                                                            WiFi_Dev_RX_STS_OVERRUN);
                /* `#START WiFi_Dev_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef WiFi_Dev_RXISR_ERROR_CALLBACK
                WiFi_Dev_RXISR_ERROR_Callback();
            #endif /* WiFi_Dev_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & WiFi_Dev_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = WiFi_Dev_RXDATA_REG;
            #if (WiFi_Dev_RXHW_ADDRESS_ENABLED)
                if(WiFi_Dev_rxAddressMode == (uint8)WiFi_Dev__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & WiFi_Dev_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & WiFi_Dev_RX_STS_ADDR_MATCH) != 0u)
                        {
                            WiFi_Dev_rxAddressDetected = 1u;
                        }
                        else
                        {
                            WiFi_Dev_rxAddressDetected = 0u;
                        }
                    }
                    if(WiFi_Dev_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        WiFi_Dev_rxBuffer[WiFi_Dev_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    WiFi_Dev_rxBuffer[WiFi_Dev_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                WiFi_Dev_rxBuffer[WiFi_Dev_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (WiFi_Dev_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(WiFi_Dev_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        WiFi_Dev_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    WiFi_Dev_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(WiFi_Dev_rxBufferWrite >= WiFi_Dev_RX_BUFFER_SIZE)
                    {
                        WiFi_Dev_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(WiFi_Dev_rxBufferWrite == WiFi_Dev_rxBufferRead)
                    {
                        WiFi_Dev_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (WiFi_Dev_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            WiFi_Dev_RXSTATUS_MASK_REG  &= (uint8)~WiFi_Dev_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(WiFi_Dev_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (WiFi_Dev_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & WiFi_Dev_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START WiFi_Dev_RXISR_END` */

        /* `#END` */

    #ifdef WiFi_Dev_RXISR_EXIT_CALLBACK
        WiFi_Dev_RXISR_ExitCallback();
    #endif /* WiFi_Dev_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED && (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED)) */


#if (WiFi_Dev_TX_INTERRUPT_ENABLED && WiFi_Dev_TX_ENABLED)
    /*******************************************************************************
    * Function Name: WiFi_Dev_TXISR
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
    *  WiFi_Dev_txBuffer - RAM buffer pointer for transmit data from.
    *  WiFi_Dev_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  WiFi_Dev_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(WiFi_Dev_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef WiFi_Dev_TXISR_ENTRY_CALLBACK
        WiFi_Dev_TXISR_EntryCallback();
    #endif /* WiFi_Dev_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START WiFi_Dev_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((WiFi_Dev_txBufferRead != WiFi_Dev_txBufferWrite) &&
             ((WiFi_Dev_TXSTATUS_REG & WiFi_Dev_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(WiFi_Dev_txBufferRead >= WiFi_Dev_TX_BUFFER_SIZE)
            {
                WiFi_Dev_txBufferRead = 0u;
            }

            WiFi_Dev_TXDATA_REG = WiFi_Dev_txBuffer[WiFi_Dev_txBufferRead];

            /* Set next pointer */
            WiFi_Dev_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START WiFi_Dev_TXISR_END` */

        /* `#END` */

    #ifdef WiFi_Dev_TXISR_EXIT_CALLBACK
        WiFi_Dev_TXISR_ExitCallback();
    #endif /* WiFi_Dev_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (WiFi_Dev_TX_INTERRUPT_ENABLED && WiFi_Dev_TX_ENABLED) */


/* [] END OF FILE */
