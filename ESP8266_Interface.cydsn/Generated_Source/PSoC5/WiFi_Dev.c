/*******************************************************************************
* File Name: WiFi_Dev.c
* Version 2.50
*
* Description:
*  This file provides all API functionality of the UART component
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "WiFi_Dev.h"
#if (WiFi_Dev_INTERNAL_CLOCK_USED)
    #include "WiFi_Dev_IntClock.h"
#endif /* End WiFi_Dev_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 WiFi_Dev_initVar = 0u;

#if (WiFi_Dev_TX_INTERRUPT_ENABLED && WiFi_Dev_TX_ENABLED)
    volatile uint8 WiFi_Dev_txBuffer[WiFi_Dev_TX_BUFFER_SIZE];
    volatile uint8 WiFi_Dev_txBufferRead = 0u;
    uint8 WiFi_Dev_txBufferWrite = 0u;
#endif /* (WiFi_Dev_TX_INTERRUPT_ENABLED && WiFi_Dev_TX_ENABLED) */

#if (WiFi_Dev_RX_INTERRUPT_ENABLED && (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED))
    uint8 WiFi_Dev_errorStatus = 0u;
    volatile uint8 WiFi_Dev_rxBuffer[WiFi_Dev_RX_BUFFER_SIZE];
    volatile uint8 WiFi_Dev_rxBufferRead  = 0u;
    volatile uint8 WiFi_Dev_rxBufferWrite = 0u;
    volatile uint8 WiFi_Dev_rxBufferLoopDetect = 0u;
    volatile uint8 WiFi_Dev_rxBufferOverflow   = 0u;
    #if (WiFi_Dev_RXHW_ADDRESS_ENABLED)
        volatile uint8 WiFi_Dev_rxAddressMode = WiFi_Dev_RX_ADDRESS_MODE;
        volatile uint8 WiFi_Dev_rxAddressDetected = 0u;
    #endif /* (WiFi_Dev_RXHW_ADDRESS_ENABLED) */
#endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED && (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED)) */


/*******************************************************************************
* Function Name: WiFi_Dev_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  WiFi_Dev_Start() sets the initVar variable, calls the
*  WiFi_Dev_Init() function, and then calls the
*  WiFi_Dev_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The WiFi_Dev_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time WiFi_Dev_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the WiFi_Dev_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void WiFi_Dev_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(WiFi_Dev_initVar == 0u)
    {
        WiFi_Dev_Init();
        WiFi_Dev_initVar = 1u;
    }

    WiFi_Dev_Enable();
}


/*******************************************************************************
* Function Name: WiFi_Dev_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call WiFi_Dev_Init() because
*  the WiFi_Dev_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void WiFi_Dev_Init(void) 
{
    #if(WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED)

        #if (WiFi_Dev_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(WiFi_Dev_RX_VECT_NUM, &WiFi_Dev_RXISR);
            CyIntSetPriority(WiFi_Dev_RX_VECT_NUM, WiFi_Dev_RX_PRIOR_NUM);
            WiFi_Dev_errorStatus = 0u;
        #endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED) */

        #if (WiFi_Dev_RXHW_ADDRESS_ENABLED)
            WiFi_Dev_SetRxAddressMode(WiFi_Dev_RX_ADDRESS_MODE);
            WiFi_Dev_SetRxAddress1(WiFi_Dev_RX_HW_ADDRESS1);
            WiFi_Dev_SetRxAddress2(WiFi_Dev_RX_HW_ADDRESS2);
        #endif /* End WiFi_Dev_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        WiFi_Dev_RXBITCTR_PERIOD_REG = WiFi_Dev_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        WiFi_Dev_RXSTATUS_MASK_REG  = WiFi_Dev_INIT_RX_INTERRUPTS_MASK;
    #endif /* End WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED*/

    #if(WiFi_Dev_TX_ENABLED)
        #if (WiFi_Dev_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(WiFi_Dev_TX_VECT_NUM, &WiFi_Dev_TXISR);
            CyIntSetPriority(WiFi_Dev_TX_VECT_NUM, WiFi_Dev_TX_PRIOR_NUM);
        #endif /* (WiFi_Dev_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (WiFi_Dev_TXCLKGEN_DP)
            WiFi_Dev_TXBITCLKGEN_CTR_REG = WiFi_Dev_BIT_CENTER;
            WiFi_Dev_TXBITCLKTX_COMPLETE_REG = ((WiFi_Dev_NUMBER_OF_DATA_BITS +
                        WiFi_Dev_NUMBER_OF_START_BIT) * WiFi_Dev_OVER_SAMPLE_COUNT) - 1u;
        #else
            WiFi_Dev_TXBITCTR_PERIOD_REG = ((WiFi_Dev_NUMBER_OF_DATA_BITS +
                        WiFi_Dev_NUMBER_OF_START_BIT) * WiFi_Dev_OVER_SAMPLE_8) - 1u;
        #endif /* End WiFi_Dev_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (WiFi_Dev_TX_INTERRUPT_ENABLED)
            WiFi_Dev_TXSTATUS_MASK_REG = WiFi_Dev_TX_STS_FIFO_EMPTY;
        #else
            WiFi_Dev_TXSTATUS_MASK_REG = WiFi_Dev_INIT_TX_INTERRUPTS_MASK;
        #endif /*End WiFi_Dev_TX_INTERRUPT_ENABLED*/

    #endif /* End WiFi_Dev_TX_ENABLED */

    #if(WiFi_Dev_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        WiFi_Dev_WriteControlRegister( \
            (WiFi_Dev_ReadControlRegister() & (uint8)~WiFi_Dev_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(WiFi_Dev_PARITY_TYPE << WiFi_Dev_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End WiFi_Dev_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: WiFi_Dev_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call WiFi_Dev_Enable() because the WiFi_Dev_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  WiFi_Dev_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void WiFi_Dev_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        WiFi_Dev_RXBITCTR_CONTROL_REG |= WiFi_Dev_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        WiFi_Dev_RXSTATUS_ACTL_REG  |= WiFi_Dev_INT_ENABLE;

        #if (WiFi_Dev_RX_INTERRUPT_ENABLED)
            WiFi_Dev_EnableRxInt();

            #if (WiFi_Dev_RXHW_ADDRESS_ENABLED)
                WiFi_Dev_rxAddressDetected = 0u;
            #endif /* (WiFi_Dev_RXHW_ADDRESS_ENABLED) */
        #endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED) */
    #endif /* (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED) */

    #if(WiFi_Dev_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!WiFi_Dev_TXCLKGEN_DP)
            WiFi_Dev_TXBITCTR_CONTROL_REG |= WiFi_Dev_CNTR_ENABLE;
        #endif /* End WiFi_Dev_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        WiFi_Dev_TXSTATUS_ACTL_REG |= WiFi_Dev_INT_ENABLE;
        #if (WiFi_Dev_TX_INTERRUPT_ENABLED)
            WiFi_Dev_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            WiFi_Dev_EnableTxInt();
        #endif /* (WiFi_Dev_TX_INTERRUPT_ENABLED) */
     #endif /* (WiFi_Dev_TX_INTERRUPT_ENABLED) */

    #if (WiFi_Dev_INTERNAL_CLOCK_USED)
        WiFi_Dev_IntClock_Start();  /* Enable the clock */
    #endif /* (WiFi_Dev_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: WiFi_Dev_Stop
********************************************************************************
*
* Summary:
*  Disables the UART operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void WiFi_Dev_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED)
        WiFi_Dev_RXBITCTR_CONTROL_REG &= (uint8) ~WiFi_Dev_CNTR_ENABLE;
    #endif /* (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED) */

    #if (WiFi_Dev_TX_ENABLED)
        #if(!WiFi_Dev_TXCLKGEN_DP)
            WiFi_Dev_TXBITCTR_CONTROL_REG &= (uint8) ~WiFi_Dev_CNTR_ENABLE;
        #endif /* (!WiFi_Dev_TXCLKGEN_DP) */
    #endif /* (WiFi_Dev_TX_ENABLED) */

    #if (WiFi_Dev_INTERNAL_CLOCK_USED)
        WiFi_Dev_IntClock_Stop();   /* Disable the clock */
    #endif /* (WiFi_Dev_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED)
        WiFi_Dev_RXSTATUS_ACTL_REG  &= (uint8) ~WiFi_Dev_INT_ENABLE;

        #if (WiFi_Dev_RX_INTERRUPT_ENABLED)
            WiFi_Dev_DisableRxInt();
        #endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED) */
    #endif /* (WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED) */

    #if (WiFi_Dev_TX_ENABLED)
        WiFi_Dev_TXSTATUS_ACTL_REG &= (uint8) ~WiFi_Dev_INT_ENABLE;

        #if (WiFi_Dev_TX_INTERRUPT_ENABLED)
            WiFi_Dev_DisableTxInt();
        #endif /* (WiFi_Dev_TX_INTERRUPT_ENABLED) */
    #endif /* (WiFi_Dev_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: WiFi_Dev_ReadControlRegister
********************************************************************************
*
* Summary:
*  Returns the current value of the control register.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the control register.
*
*******************************************************************************/
uint8 WiFi_Dev_ReadControlRegister(void) 
{
    #if (WiFi_Dev_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(WiFi_Dev_CONTROL_REG);
    #endif /* (WiFi_Dev_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: WiFi_Dev_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  WiFi_Dev_WriteControlRegister(uint8 control) 
{
    #if (WiFi_Dev_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       WiFi_Dev_CONTROL_REG = control;
    #endif /* (WiFi_Dev_CONTROL_REG_REMOVED) */
}


#if(WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED)
    /*******************************************************************************
    * Function Name: WiFi_Dev_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      WiFi_Dev_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      WiFi_Dev_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      WiFi_Dev_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      WiFi_Dev_RX_STS_BREAK            Interrupt on break.
    *      WiFi_Dev_RX_STS_OVERRUN          Interrupt on overrun error.
    *      WiFi_Dev_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      WiFi_Dev_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void WiFi_Dev_SetRxInterruptMode(uint8 intSrc) 
    {
        WiFi_Dev_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns the next byte of received data. This function returns data without
    *  checking the status. You must check the status separately.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  WiFi_Dev_rxBuffer - RAM buffer pointer for save received data.
    *  WiFi_Dev_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  WiFi_Dev_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  WiFi_Dev_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 WiFi_Dev_ReadRxData(void) 
    {
        uint8 rxData;

    #if (WiFi_Dev_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        WiFi_Dev_DisableRxInt();

        locRxBufferRead  = WiFi_Dev_rxBufferRead;
        locRxBufferWrite = WiFi_Dev_rxBufferWrite;

        if( (WiFi_Dev_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = WiFi_Dev_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= WiFi_Dev_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            WiFi_Dev_rxBufferRead = locRxBufferRead;

            if(WiFi_Dev_rxBufferLoopDetect != 0u)
            {
                WiFi_Dev_rxBufferLoopDetect = 0u;
                #if ((WiFi_Dev_RX_INTERRUPT_ENABLED) && (WiFi_Dev_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( WiFi_Dev_HD_ENABLED )
                        if((WiFi_Dev_CONTROL_REG & WiFi_Dev_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            WiFi_Dev_RXSTATUS_MASK_REG  |= WiFi_Dev_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        WiFi_Dev_RXSTATUS_MASK_REG  |= WiFi_Dev_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end WiFi_Dev_HD_ENABLED */
                #endif /* ((WiFi_Dev_RX_INTERRUPT_ENABLED) && (WiFi_Dev_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = WiFi_Dev_RXDATA_REG;
        }

        WiFi_Dev_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = WiFi_Dev_RXDATA_REG;

    #endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Returns the current state of the receiver status register and the software
    *  buffer overflow status.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Side Effect:
    *  All status register bits are clear-on-read except
    *  WiFi_Dev_RX_STS_FIFO_NOTEMPTY.
    *  WiFi_Dev_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  WiFi_Dev_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   WiFi_Dev_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   WiFi_Dev_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 WiFi_Dev_ReadRxStatus(void) 
    {
        uint8 status;

        status = WiFi_Dev_RXSTATUS_REG & WiFi_Dev_RX_HW_MASK;

    #if (WiFi_Dev_RX_INTERRUPT_ENABLED)
        if(WiFi_Dev_rxBufferOverflow != 0u)
        {
            status |= WiFi_Dev_RX_STS_SOFT_BUFF_OVER;
            WiFi_Dev_rxBufferOverflow = 0u;
        }
    #endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. WiFi_Dev_GetChar() is
    *  designed for ASCII characters and returns a uint8 where 1 to 255 are values
    *  for valid characters and 0 indicates an error occurred or no data is present.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  WiFi_Dev_rxBuffer - RAM buffer pointer for save received data.
    *  WiFi_Dev_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  WiFi_Dev_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  WiFi_Dev_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 WiFi_Dev_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (WiFi_Dev_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        WiFi_Dev_DisableRxInt();

        locRxBufferRead  = WiFi_Dev_rxBufferRead;
        locRxBufferWrite = WiFi_Dev_rxBufferWrite;

        if( (WiFi_Dev_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = WiFi_Dev_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= WiFi_Dev_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            WiFi_Dev_rxBufferRead = locRxBufferRead;

            if(WiFi_Dev_rxBufferLoopDetect != 0u)
            {
                WiFi_Dev_rxBufferLoopDetect = 0u;
                #if( (WiFi_Dev_RX_INTERRUPT_ENABLED) && (WiFi_Dev_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( WiFi_Dev_HD_ENABLED )
                        if((WiFi_Dev_CONTROL_REG & WiFi_Dev_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            WiFi_Dev_RXSTATUS_MASK_REG |= WiFi_Dev_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        WiFi_Dev_RXSTATUS_MASK_REG |= WiFi_Dev_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end WiFi_Dev_HD_ENABLED */
                #endif /* WiFi_Dev_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = WiFi_Dev_RXSTATUS_REG;
            if((rxStatus & WiFi_Dev_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = WiFi_Dev_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (WiFi_Dev_RX_STS_BREAK | WiFi_Dev_RX_STS_PAR_ERROR |
                                WiFi_Dev_RX_STS_STOP_ERROR | WiFi_Dev_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        WiFi_Dev_EnableRxInt();

    #else

        rxStatus =WiFi_Dev_RXSTATUS_REG;
        if((rxStatus & WiFi_Dev_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = WiFi_Dev_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (WiFi_Dev_RX_STS_BREAK | WiFi_Dev_RX_STS_PAR_ERROR |
                            WiFi_Dev_RX_STS_STOP_ERROR | WiFi_Dev_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, returns received character and error
    *  condition.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains status and LSB contains UART RX data. If the MSB is nonzero,
    *  an error has occurred.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 WiFi_Dev_GetByte(void) 
    {
        
    #if (WiFi_Dev_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        WiFi_Dev_DisableRxInt();
        locErrorStatus = (uint16)WiFi_Dev_errorStatus;
        WiFi_Dev_errorStatus = 0u;
        WiFi_Dev_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | WiFi_Dev_ReadRxData() );
    #else
        return ( ((uint16)WiFi_Dev_ReadRxStatus() << 8u) | WiFi_Dev_ReadRxData() );
    #endif /* WiFi_Dev_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of received bytes available in the RX buffer.
    *  * RX software buffer is disabled (RX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty RX FIFO or 1 for not empty RX FIFO.
    *  * RX software buffer is enabled: returns the number of bytes available in 
    *    the RX software buffer. Bytes available in the RX FIFO do not take to 
    *    account.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Number of bytes in the RX buffer. 
    *    Return value type depends on RX Buffer Size parameter.
    *
    * Global Variables:
    *  WiFi_Dev_rxBufferWrite - used to calculate left bytes.
    *  WiFi_Dev_rxBufferRead - used to calculate left bytes.
    *  WiFi_Dev_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 WiFi_Dev_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (WiFi_Dev_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        WiFi_Dev_DisableRxInt();

        if(WiFi_Dev_rxBufferRead == WiFi_Dev_rxBufferWrite)
        {
            if(WiFi_Dev_rxBufferLoopDetect != 0u)
            {
                size = WiFi_Dev_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(WiFi_Dev_rxBufferRead < WiFi_Dev_rxBufferWrite)
        {
            size = (WiFi_Dev_rxBufferWrite - WiFi_Dev_rxBufferRead);
        }
        else
        {
            size = (WiFi_Dev_RX_BUFFER_SIZE - WiFi_Dev_rxBufferRead) + WiFi_Dev_rxBufferWrite;
        }

        WiFi_Dev_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((WiFi_Dev_RXSTATUS_REG & WiFi_Dev_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the receiver memory buffer and hardware RX FIFO of all received data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WiFi_Dev_rxBufferWrite - cleared to zero.
    *  WiFi_Dev_rxBufferRead - cleared to zero.
    *  WiFi_Dev_rxBufferLoopDetect - cleared to zero.
    *  WiFi_Dev_rxBufferOverflow - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *
    *******************************************************************************/
    void WiFi_Dev_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        WiFi_Dev_RXDATA_AUX_CTL_REG |= (uint8)  WiFi_Dev_RX_FIFO_CLR;
        WiFi_Dev_RXDATA_AUX_CTL_REG &= (uint8) ~WiFi_Dev_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (WiFi_Dev_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        WiFi_Dev_DisableRxInt();

        WiFi_Dev_rxBufferRead = 0u;
        WiFi_Dev_rxBufferWrite = 0u;
        WiFi_Dev_rxBufferLoopDetect = 0u;
        WiFi_Dev_rxBufferOverflow = 0u;

        WiFi_Dev_EnableRxInt();

    #endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  WiFi_Dev__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  WiFi_Dev__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  WiFi_Dev__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  WiFi_Dev__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  WiFi_Dev__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WiFi_Dev_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  WiFi_Dev_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void WiFi_Dev_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(WiFi_Dev_RXHW_ADDRESS_ENABLED)
            #if(WiFi_Dev_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* WiFi_Dev_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = WiFi_Dev_CONTROL_REG & (uint8)~WiFi_Dev_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << WiFi_Dev_CTRL_RXADDR_MODE0_SHIFT);
                WiFi_Dev_CONTROL_REG = tmpCtrl;

                #if(WiFi_Dev_RX_INTERRUPT_ENABLED && \
                   (WiFi_Dev_RXBUFFERSIZE > WiFi_Dev_FIFO_LENGTH) )
                    WiFi_Dev_rxAddressMode = addressMode;
                    WiFi_Dev_rxAddressDetected = 0u;
                #endif /* End WiFi_Dev_RXBUFFERSIZE > WiFi_Dev_FIFO_LENGTH*/
            #endif /* End WiFi_Dev_CONTROL_REG_REMOVED */
        #else /* WiFi_Dev_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End WiFi_Dev_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Sets the first of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #1 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void WiFi_Dev_SetRxAddress1(uint8 address) 
    {
        WiFi_Dev_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Sets the second of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #2 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void WiFi_Dev_SetRxAddress2(uint8 address) 
    {
        WiFi_Dev_RXADDRESS2_REG = address;
    }

#endif  /* WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED*/


#if( (WiFi_Dev_TX_ENABLED) || (WiFi_Dev_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: WiFi_Dev_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   WiFi_Dev_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   WiFi_Dev_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   WiFi_Dev_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   WiFi_Dev_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void WiFi_Dev_SetTxInterruptMode(uint8 intSrc) 
    {
        WiFi_Dev_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Places a byte of data into the transmit buffer to be sent when the bus is
    *  available without checking the TX status register. You must check status
    *  separately.
    *
    * Parameters:
    *  txDataByte: data byte
    *
    * Return:
    * None.
    *
    * Global Variables:
    *  WiFi_Dev_txBuffer - RAM buffer pointer for save data for transmission
    *  WiFi_Dev_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  WiFi_Dev_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  WiFi_Dev_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void WiFi_Dev_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(WiFi_Dev_initVar != 0u)
        {
        #if (WiFi_Dev_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            WiFi_Dev_DisableTxInt();

            if( (WiFi_Dev_txBufferRead == WiFi_Dev_txBufferWrite) &&
                ((WiFi_Dev_TXSTATUS_REG & WiFi_Dev_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                WiFi_Dev_TXDATA_REG = txDataByte;
            }
            else
            {
                if(WiFi_Dev_txBufferWrite >= WiFi_Dev_TX_BUFFER_SIZE)
                {
                    WiFi_Dev_txBufferWrite = 0u;
                }

                WiFi_Dev_txBuffer[WiFi_Dev_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                WiFi_Dev_txBufferWrite++;
            }

            WiFi_Dev_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            WiFi_Dev_TXDATA_REG = txDataByte;

        #endif /*(WiFi_Dev_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Reads the status register for the TX portion of the UART.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the TX status register, which is cleared on read.
    *  It is up to the user to handle all bits in this return value accordingly,
    *  even if the bit was not enabled as an interrupt source the event happened
    *  and must be handled accordingly.
    *
    *******************************************************************************/
    uint8 WiFi_Dev_ReadTxStatus(void) 
    {
        return(WiFi_Dev_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Puts a byte of data into the transmit buffer to be sent when the bus is
    *  available. This is a blocking API that waits until the TX buffer has room to
    *  hold the data.
    *
    * Parameters:
    *  txDataByte: Byte containing the data to transmit
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WiFi_Dev_txBuffer - RAM buffer pointer for save data for transmission
    *  WiFi_Dev_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  WiFi_Dev_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  WiFi_Dev_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void WiFi_Dev_PutChar(uint8 txDataByte) 
    {
    #if (WiFi_Dev_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((WiFi_Dev_TX_BUFFER_SIZE > WiFi_Dev_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            WiFi_Dev_DisableTxInt();
        #endif /* (WiFi_Dev_TX_BUFFER_SIZE > WiFi_Dev_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = WiFi_Dev_txBufferWrite;
            locTxBufferRead  = WiFi_Dev_txBufferRead;

        #if ((WiFi_Dev_TX_BUFFER_SIZE > WiFi_Dev_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            WiFi_Dev_EnableTxInt();
        #endif /* (WiFi_Dev_TX_BUFFER_SIZE > WiFi_Dev_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(WiFi_Dev_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((WiFi_Dev_TXSTATUS_REG & WiFi_Dev_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            WiFi_Dev_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= WiFi_Dev_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            WiFi_Dev_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((WiFi_Dev_TX_BUFFER_SIZE > WiFi_Dev_MAX_BYTE_VALUE) && (CY_PSOC3))
            WiFi_Dev_DisableTxInt();
        #endif /* (WiFi_Dev_TX_BUFFER_SIZE > WiFi_Dev_MAX_BYTE_VALUE) && (CY_PSOC3) */

            WiFi_Dev_txBufferWrite = locTxBufferWrite;

        #if ((WiFi_Dev_TX_BUFFER_SIZE > WiFi_Dev_MAX_BYTE_VALUE) && (CY_PSOC3))
            WiFi_Dev_EnableTxInt();
        #endif /* (WiFi_Dev_TX_BUFFER_SIZE > WiFi_Dev_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (WiFi_Dev_TXSTATUS_REG & WiFi_Dev_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                WiFi_Dev_SetPendingTxInt();
            }
        }

    #else

        while((WiFi_Dev_TXSTATUS_REG & WiFi_Dev_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        WiFi_Dev_TXDATA_REG = txDataByte;

    #endif /* WiFi_Dev_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_PutString
    ********************************************************************************
    *
    * Summary:
    *  Sends a NULL terminated string to the TX buffer for transmission.
    *
    * Parameters:
    *  string[]: Pointer to the null terminated string array residing in RAM or ROM
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WiFi_Dev_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void WiFi_Dev_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(WiFi_Dev_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                WiFi_Dev_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Places N bytes of data from a memory array into the TX buffer for
    *  transmission.
    *
    * Parameters:
    *  string[]: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of bytes to be transmitted. The type depends on TX Buffer
    *             Size parameter.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WiFi_Dev_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void WiFi_Dev_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(WiFi_Dev_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                WiFi_Dev_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Writes a byte of data followed by a carriage return (0x0D) and line feed
    *  (0x0A) to the transmit buffer.
    *
    * Parameters:
    *  txDataByte: Data byte to transmit before the carriage return and line feed.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WiFi_Dev_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void WiFi_Dev_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(WiFi_Dev_initVar != 0u)
        {
            WiFi_Dev_PutChar(txDataByte);
            WiFi_Dev_PutChar(0x0Du);
            WiFi_Dev_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of bytes in the TX buffer which are waiting to be 
    *  transmitted.
    *  * TX software buffer is disabled (TX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty TX FIFO, 1 for not full TX FIFO or 4 for full TX FIFO.
    *  * TX software buffer is enabled: returns the number of bytes in the TX 
    *    software buffer which are waiting to be transmitted. Bytes available in the
    *    TX FIFO do not count.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Number of bytes used in the TX buffer. Return value type depends on the TX 
    *  Buffer Size parameter.
    *
    * Global Variables:
    *  WiFi_Dev_txBufferWrite - used to calculate left space.
    *  WiFi_Dev_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 WiFi_Dev_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (WiFi_Dev_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        WiFi_Dev_DisableTxInt();

        if(WiFi_Dev_txBufferRead == WiFi_Dev_txBufferWrite)
        {
            size = 0u;
        }
        else if(WiFi_Dev_txBufferRead < WiFi_Dev_txBufferWrite)
        {
            size = (WiFi_Dev_txBufferWrite - WiFi_Dev_txBufferRead);
        }
        else
        {
            size = (WiFi_Dev_TX_BUFFER_SIZE - WiFi_Dev_txBufferRead) +
                    WiFi_Dev_txBufferWrite;
        }

        WiFi_Dev_EnableTxInt();

    #else

        size = WiFi_Dev_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & WiFi_Dev_TX_STS_FIFO_FULL) != 0u)
        {
            size = WiFi_Dev_FIFO_LENGTH;
        }
        else if((size & WiFi_Dev_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (WiFi_Dev_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears all data from the TX buffer and hardware TX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WiFi_Dev_txBufferWrite - cleared to zero.
    *  WiFi_Dev_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Data waiting in the transmit buffer is not sent; a byte that is currently
    *  transmitting finishes transmitting.
    *
    *******************************************************************************/
    void WiFi_Dev_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        WiFi_Dev_TXDATA_AUX_CTL_REG |= (uint8)  WiFi_Dev_TX_FIFO_CLR;
        WiFi_Dev_TXDATA_AUX_CTL_REG &= (uint8) ~WiFi_Dev_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (WiFi_Dev_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        WiFi_Dev_DisableTxInt();

        WiFi_Dev_txBufferRead = 0u;
        WiFi_Dev_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        WiFi_Dev_EnableTxInt();

    #endif /* (WiFi_Dev_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   WiFi_Dev_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   WiFi_Dev_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   WiFi_Dev_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   WiFi_Dev_SEND_WAIT_REINIT - Performs both options: 
    *      WiFi_Dev_SEND_BREAK and WiFi_Dev_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WiFi_Dev_initVar - checked to identify that the component has been
    *     initialized.
    *  txPeriod - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  There are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Function will block CPU until transmission
    *     complete.
    *  2) User may want to use blocking time if UART configured to the low speed
    *     operation
    *     Example for this case:
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to initialize and use the interrupt to
    *     complete break operation.
    *     Example for this case:
    *     Initialize TX interrupt with "TX - On TX Complete" parameter
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     When interrupt appear with WiFi_Dev_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The WiFi_Dev_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void WiFi_Dev_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(WiFi_Dev_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(WiFi_Dev_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == WiFi_Dev_SEND_BREAK) ||
                (retMode == WiFi_Dev_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                WiFi_Dev_WriteControlRegister(WiFi_Dev_ReadControlRegister() |
                                                      WiFi_Dev_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                WiFi_Dev_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = WiFi_Dev_TXSTATUS_REG;
                }
                while((tmpStat & WiFi_Dev_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == WiFi_Dev_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == WiFi_Dev_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = WiFi_Dev_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & WiFi_Dev_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == WiFi_Dev_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == WiFi_Dev_REINIT) ||
                (retMode == WiFi_Dev_SEND_WAIT_REINIT) )
            {
                WiFi_Dev_WriteControlRegister(WiFi_Dev_ReadControlRegister() &
                                              (uint8)~WiFi_Dev_CTRL_HD_SEND_BREAK);
            }

        #else /* WiFi_Dev_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == WiFi_Dev_SEND_BREAK) ||
                (retMode == WiFi_Dev_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (WiFi_Dev_PARITY_TYPE != WiFi_Dev__B_UART__NONE_REVB) || \
                                    (WiFi_Dev_PARITY_TYPE_SW != 0u) )
                    WiFi_Dev_WriteControlRegister(WiFi_Dev_ReadControlRegister() |
                                                          WiFi_Dev_CTRL_HD_SEND_BREAK);
                #endif /* End WiFi_Dev_PARITY_TYPE != WiFi_Dev__B_UART__NONE_REVB  */

                #if(WiFi_Dev_TXCLKGEN_DP)
                    txPeriod = WiFi_Dev_TXBITCLKTX_COMPLETE_REG;
                    WiFi_Dev_TXBITCLKTX_COMPLETE_REG = WiFi_Dev_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = WiFi_Dev_TXBITCTR_PERIOD_REG;
                    WiFi_Dev_TXBITCTR_PERIOD_REG = WiFi_Dev_TXBITCTR_BREAKBITS8X;
                #endif /* End WiFi_Dev_TXCLKGEN_DP */

                /* Send zeros */
                WiFi_Dev_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = WiFi_Dev_TXSTATUS_REG;
                }
                while((tmpStat & WiFi_Dev_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == WiFi_Dev_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == WiFi_Dev_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = WiFi_Dev_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & WiFi_Dev_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == WiFi_Dev_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == WiFi_Dev_REINIT) ||
                (retMode == WiFi_Dev_SEND_WAIT_REINIT) )
            {

            #if(WiFi_Dev_TXCLKGEN_DP)
                WiFi_Dev_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                WiFi_Dev_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End WiFi_Dev_TXCLKGEN_DP */

            #if( (WiFi_Dev_PARITY_TYPE != WiFi_Dev__B_UART__NONE_REVB) || \
                 (WiFi_Dev_PARITY_TYPE_SW != 0u) )
                WiFi_Dev_WriteControlRegister(WiFi_Dev_ReadControlRegister() &
                                                      (uint8) ~WiFi_Dev_CTRL_HD_SEND_BREAK);
            #endif /* End WiFi_Dev_PARITY_TYPE != NONE */
            }
        #endif    /* End WiFi_Dev_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       WiFi_Dev_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       WiFi_Dev_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears WiFi_Dev_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void WiFi_Dev_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( WiFi_Dev_CONTROL_REG_REMOVED == 0u )
            WiFi_Dev_WriteControlRegister(WiFi_Dev_ReadControlRegister() |
                                                  WiFi_Dev_CTRL_MARK);
        #endif /* End WiFi_Dev_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( WiFi_Dev_CONTROL_REG_REMOVED == 0u )
            WiFi_Dev_WriteControlRegister(WiFi_Dev_ReadControlRegister() &
                                                  (uint8) ~WiFi_Dev_CTRL_MARK);
        #endif /* End WiFi_Dev_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndWiFi_Dev_TX_ENABLED */

#if(WiFi_Dev_HD_ENABLED)


    /*******************************************************************************
    * Function Name: WiFi_Dev_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the receiver configuration in half duplex mode. After calling this
    *  function, the UART is ready to receive data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the transmitter
    *  configuration.
    *
    *******************************************************************************/
    void WiFi_Dev_LoadRxConfig(void) 
    {
        WiFi_Dev_WriteControlRegister(WiFi_Dev_ReadControlRegister() &
                                                (uint8)~WiFi_Dev_CTRL_HD_SEND);
        WiFi_Dev_RXBITCTR_PERIOD_REG = WiFi_Dev_HD_RXBITCTR_INIT;

    #if (WiFi_Dev_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        WiFi_Dev_SetRxInterruptMode(WiFi_Dev_INIT_RX_INTERRUPTS_MASK);
    #endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: WiFi_Dev_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the transmitter configuration in half duplex mode. After calling this
    *  function, the UART is ready to transmit data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the receiver configuration.
    *
    *******************************************************************************/
    void WiFi_Dev_LoadTxConfig(void) 
    {
    #if (WiFi_Dev_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        WiFi_Dev_SetRxInterruptMode(0u);
    #endif /* (WiFi_Dev_RX_INTERRUPT_ENABLED) */

        WiFi_Dev_WriteControlRegister(WiFi_Dev_ReadControlRegister() | WiFi_Dev_CTRL_HD_SEND);
        WiFi_Dev_RXBITCTR_PERIOD_REG = WiFi_Dev_HD_TXBITCTR_INIT;
    }

#endif  /* WiFi_Dev_HD_ENABLED */


/* [] END OF FILE */
