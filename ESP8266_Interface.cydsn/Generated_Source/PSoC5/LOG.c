/*******************************************************************************
* File Name: LOG.c
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

#include "LOG.h"
#if (LOG_INTERNAL_CLOCK_USED)
    #include "LOG_IntClock.h"
#endif /* End LOG_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 LOG_initVar = 0u;

#if (LOG_TX_INTERRUPT_ENABLED && LOG_TX_ENABLED)
    volatile uint8 LOG_txBuffer[LOG_TX_BUFFER_SIZE];
    volatile uint8 LOG_txBufferRead = 0u;
    uint8 LOG_txBufferWrite = 0u;
#endif /* (LOG_TX_INTERRUPT_ENABLED && LOG_TX_ENABLED) */

#if (LOG_RX_INTERRUPT_ENABLED && (LOG_RX_ENABLED || LOG_HD_ENABLED))
    uint8 LOG_errorStatus = 0u;
    volatile uint8 LOG_rxBuffer[LOG_RX_BUFFER_SIZE];
    volatile uint8 LOG_rxBufferRead  = 0u;
    volatile uint8 LOG_rxBufferWrite = 0u;
    volatile uint8 LOG_rxBufferLoopDetect = 0u;
    volatile uint8 LOG_rxBufferOverflow   = 0u;
    #if (LOG_RXHW_ADDRESS_ENABLED)
        volatile uint8 LOG_rxAddressMode = LOG_RX_ADDRESS_MODE;
        volatile uint8 LOG_rxAddressDetected = 0u;
    #endif /* (LOG_RXHW_ADDRESS_ENABLED) */
#endif /* (LOG_RX_INTERRUPT_ENABLED && (LOG_RX_ENABLED || LOG_HD_ENABLED)) */


/*******************************************************************************
* Function Name: LOG_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  LOG_Start() sets the initVar variable, calls the
*  LOG_Init() function, and then calls the
*  LOG_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The LOG_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time LOG_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the LOG_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LOG_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(LOG_initVar == 0u)
    {
        LOG_Init();
        LOG_initVar = 1u;
    }

    LOG_Enable();
}


/*******************************************************************************
* Function Name: LOG_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call LOG_Init() because
*  the LOG_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void LOG_Init(void) 
{
    #if(LOG_RX_ENABLED || LOG_HD_ENABLED)

        #if (LOG_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(LOG_RX_VECT_NUM, &LOG_RXISR);
            CyIntSetPriority(LOG_RX_VECT_NUM, LOG_RX_PRIOR_NUM);
            LOG_errorStatus = 0u;
        #endif /* (LOG_RX_INTERRUPT_ENABLED) */

        #if (LOG_RXHW_ADDRESS_ENABLED)
            LOG_SetRxAddressMode(LOG_RX_ADDRESS_MODE);
            LOG_SetRxAddress1(LOG_RX_HW_ADDRESS1);
            LOG_SetRxAddress2(LOG_RX_HW_ADDRESS2);
        #endif /* End LOG_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        LOG_RXBITCTR_PERIOD_REG = LOG_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        LOG_RXSTATUS_MASK_REG  = LOG_INIT_RX_INTERRUPTS_MASK;
    #endif /* End LOG_RX_ENABLED || LOG_HD_ENABLED*/

    #if(LOG_TX_ENABLED)
        #if (LOG_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(LOG_TX_VECT_NUM, &LOG_TXISR);
            CyIntSetPriority(LOG_TX_VECT_NUM, LOG_TX_PRIOR_NUM);
        #endif /* (LOG_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (LOG_TXCLKGEN_DP)
            LOG_TXBITCLKGEN_CTR_REG = LOG_BIT_CENTER;
            LOG_TXBITCLKTX_COMPLETE_REG = ((LOG_NUMBER_OF_DATA_BITS +
                        LOG_NUMBER_OF_START_BIT) * LOG_OVER_SAMPLE_COUNT) - 1u;
        #else
            LOG_TXBITCTR_PERIOD_REG = ((LOG_NUMBER_OF_DATA_BITS +
                        LOG_NUMBER_OF_START_BIT) * LOG_OVER_SAMPLE_8) - 1u;
        #endif /* End LOG_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (LOG_TX_INTERRUPT_ENABLED)
            LOG_TXSTATUS_MASK_REG = LOG_TX_STS_FIFO_EMPTY;
        #else
            LOG_TXSTATUS_MASK_REG = LOG_INIT_TX_INTERRUPTS_MASK;
        #endif /*End LOG_TX_INTERRUPT_ENABLED*/

    #endif /* End LOG_TX_ENABLED */

    #if(LOG_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        LOG_WriteControlRegister( \
            (LOG_ReadControlRegister() & (uint8)~LOG_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(LOG_PARITY_TYPE << LOG_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End LOG_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: LOG_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call LOG_Enable() because the LOG_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  LOG_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void LOG_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (LOG_RX_ENABLED || LOG_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        LOG_RXBITCTR_CONTROL_REG |= LOG_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        LOG_RXSTATUS_ACTL_REG  |= LOG_INT_ENABLE;

        #if (LOG_RX_INTERRUPT_ENABLED)
            LOG_EnableRxInt();

            #if (LOG_RXHW_ADDRESS_ENABLED)
                LOG_rxAddressDetected = 0u;
            #endif /* (LOG_RXHW_ADDRESS_ENABLED) */
        #endif /* (LOG_RX_INTERRUPT_ENABLED) */
    #endif /* (LOG_RX_ENABLED || LOG_HD_ENABLED) */

    #if(LOG_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!LOG_TXCLKGEN_DP)
            LOG_TXBITCTR_CONTROL_REG |= LOG_CNTR_ENABLE;
        #endif /* End LOG_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        LOG_TXSTATUS_ACTL_REG |= LOG_INT_ENABLE;
        #if (LOG_TX_INTERRUPT_ENABLED)
            LOG_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            LOG_EnableTxInt();
        #endif /* (LOG_TX_INTERRUPT_ENABLED) */
     #endif /* (LOG_TX_INTERRUPT_ENABLED) */

    #if (LOG_INTERNAL_CLOCK_USED)
        LOG_IntClock_Start();  /* Enable the clock */
    #endif /* (LOG_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: LOG_Stop
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
void LOG_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (LOG_RX_ENABLED || LOG_HD_ENABLED)
        LOG_RXBITCTR_CONTROL_REG &= (uint8) ~LOG_CNTR_ENABLE;
    #endif /* (LOG_RX_ENABLED || LOG_HD_ENABLED) */

    #if (LOG_TX_ENABLED)
        #if(!LOG_TXCLKGEN_DP)
            LOG_TXBITCTR_CONTROL_REG &= (uint8) ~LOG_CNTR_ENABLE;
        #endif /* (!LOG_TXCLKGEN_DP) */
    #endif /* (LOG_TX_ENABLED) */

    #if (LOG_INTERNAL_CLOCK_USED)
        LOG_IntClock_Stop();   /* Disable the clock */
    #endif /* (LOG_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (LOG_RX_ENABLED || LOG_HD_ENABLED)
        LOG_RXSTATUS_ACTL_REG  &= (uint8) ~LOG_INT_ENABLE;

        #if (LOG_RX_INTERRUPT_ENABLED)
            LOG_DisableRxInt();
        #endif /* (LOG_RX_INTERRUPT_ENABLED) */
    #endif /* (LOG_RX_ENABLED || LOG_HD_ENABLED) */

    #if (LOG_TX_ENABLED)
        LOG_TXSTATUS_ACTL_REG &= (uint8) ~LOG_INT_ENABLE;

        #if (LOG_TX_INTERRUPT_ENABLED)
            LOG_DisableTxInt();
        #endif /* (LOG_TX_INTERRUPT_ENABLED) */
    #endif /* (LOG_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: LOG_ReadControlRegister
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
uint8 LOG_ReadControlRegister(void) 
{
    #if (LOG_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(LOG_CONTROL_REG);
    #endif /* (LOG_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: LOG_WriteControlRegister
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
void  LOG_WriteControlRegister(uint8 control) 
{
    #if (LOG_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       LOG_CONTROL_REG = control;
    #endif /* (LOG_CONTROL_REG_REMOVED) */
}


#if(LOG_RX_ENABLED || LOG_HD_ENABLED)
    /*******************************************************************************
    * Function Name: LOG_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      LOG_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      LOG_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      LOG_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      LOG_RX_STS_BREAK            Interrupt on break.
    *      LOG_RX_STS_OVERRUN          Interrupt on overrun error.
    *      LOG_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      LOG_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void LOG_SetRxInterruptMode(uint8 intSrc) 
    {
        LOG_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: LOG_ReadRxData
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
    *  LOG_rxBuffer - RAM buffer pointer for save received data.
    *  LOG_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  LOG_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  LOG_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 LOG_ReadRxData(void) 
    {
        uint8 rxData;

    #if (LOG_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        LOG_DisableRxInt();

        locRxBufferRead  = LOG_rxBufferRead;
        locRxBufferWrite = LOG_rxBufferWrite;

        if( (LOG_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = LOG_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= LOG_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            LOG_rxBufferRead = locRxBufferRead;

            if(LOG_rxBufferLoopDetect != 0u)
            {
                LOG_rxBufferLoopDetect = 0u;
                #if ((LOG_RX_INTERRUPT_ENABLED) && (LOG_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( LOG_HD_ENABLED )
                        if((LOG_CONTROL_REG & LOG_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            LOG_RXSTATUS_MASK_REG  |= LOG_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        LOG_RXSTATUS_MASK_REG  |= LOG_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end LOG_HD_ENABLED */
                #endif /* ((LOG_RX_INTERRUPT_ENABLED) && (LOG_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = LOG_RXDATA_REG;
        }

        LOG_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = LOG_RXDATA_REG;

    #endif /* (LOG_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: LOG_ReadRxStatus
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
    *  LOG_RX_STS_FIFO_NOTEMPTY.
    *  LOG_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  LOG_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   LOG_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   LOG_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 LOG_ReadRxStatus(void) 
    {
        uint8 status;

        status = LOG_RXSTATUS_REG & LOG_RX_HW_MASK;

    #if (LOG_RX_INTERRUPT_ENABLED)
        if(LOG_rxBufferOverflow != 0u)
        {
            status |= LOG_RX_STS_SOFT_BUFF_OVER;
            LOG_rxBufferOverflow = 0u;
        }
    #endif /* (LOG_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: LOG_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. LOG_GetChar() is
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
    *  LOG_rxBuffer - RAM buffer pointer for save received data.
    *  LOG_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  LOG_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  LOG_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 LOG_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (LOG_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        LOG_DisableRxInt();

        locRxBufferRead  = LOG_rxBufferRead;
        locRxBufferWrite = LOG_rxBufferWrite;

        if( (LOG_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = LOG_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= LOG_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            LOG_rxBufferRead = locRxBufferRead;

            if(LOG_rxBufferLoopDetect != 0u)
            {
                LOG_rxBufferLoopDetect = 0u;
                #if( (LOG_RX_INTERRUPT_ENABLED) && (LOG_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( LOG_HD_ENABLED )
                        if((LOG_CONTROL_REG & LOG_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            LOG_RXSTATUS_MASK_REG |= LOG_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        LOG_RXSTATUS_MASK_REG |= LOG_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end LOG_HD_ENABLED */
                #endif /* LOG_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = LOG_RXSTATUS_REG;
            if((rxStatus & LOG_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = LOG_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (LOG_RX_STS_BREAK | LOG_RX_STS_PAR_ERROR |
                                LOG_RX_STS_STOP_ERROR | LOG_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        LOG_EnableRxInt();

    #else

        rxStatus =LOG_RXSTATUS_REG;
        if((rxStatus & LOG_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = LOG_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (LOG_RX_STS_BREAK | LOG_RX_STS_PAR_ERROR |
                            LOG_RX_STS_STOP_ERROR | LOG_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (LOG_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: LOG_GetByte
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
    uint16 LOG_GetByte(void) 
    {
        
    #if (LOG_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        LOG_DisableRxInt();
        locErrorStatus = (uint16)LOG_errorStatus;
        LOG_errorStatus = 0u;
        LOG_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | LOG_ReadRxData() );
    #else
        return ( ((uint16)LOG_ReadRxStatus() << 8u) | LOG_ReadRxData() );
    #endif /* LOG_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: LOG_GetRxBufferSize
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
    *  LOG_rxBufferWrite - used to calculate left bytes.
    *  LOG_rxBufferRead - used to calculate left bytes.
    *  LOG_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 LOG_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (LOG_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        LOG_DisableRxInt();

        if(LOG_rxBufferRead == LOG_rxBufferWrite)
        {
            if(LOG_rxBufferLoopDetect != 0u)
            {
                size = LOG_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(LOG_rxBufferRead < LOG_rxBufferWrite)
        {
            size = (LOG_rxBufferWrite - LOG_rxBufferRead);
        }
        else
        {
            size = (LOG_RX_BUFFER_SIZE - LOG_rxBufferRead) + LOG_rxBufferWrite;
        }

        LOG_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((LOG_RXSTATUS_REG & LOG_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (LOG_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: LOG_ClearRxBuffer
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
    *  LOG_rxBufferWrite - cleared to zero.
    *  LOG_rxBufferRead - cleared to zero.
    *  LOG_rxBufferLoopDetect - cleared to zero.
    *  LOG_rxBufferOverflow - cleared to zero.
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
    void LOG_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        LOG_RXDATA_AUX_CTL_REG |= (uint8)  LOG_RX_FIFO_CLR;
        LOG_RXDATA_AUX_CTL_REG &= (uint8) ~LOG_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (LOG_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        LOG_DisableRxInt();

        LOG_rxBufferRead = 0u;
        LOG_rxBufferWrite = 0u;
        LOG_rxBufferLoopDetect = 0u;
        LOG_rxBufferOverflow = 0u;

        LOG_EnableRxInt();

    #endif /* (LOG_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: LOG_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  LOG__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  LOG__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  LOG__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  LOG__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  LOG__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  LOG_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  LOG_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void LOG_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(LOG_RXHW_ADDRESS_ENABLED)
            #if(LOG_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* LOG_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = LOG_CONTROL_REG & (uint8)~LOG_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << LOG_CTRL_RXADDR_MODE0_SHIFT);
                LOG_CONTROL_REG = tmpCtrl;

                #if(LOG_RX_INTERRUPT_ENABLED && \
                   (LOG_RXBUFFERSIZE > LOG_FIFO_LENGTH) )
                    LOG_rxAddressMode = addressMode;
                    LOG_rxAddressDetected = 0u;
                #endif /* End LOG_RXBUFFERSIZE > LOG_FIFO_LENGTH*/
            #endif /* End LOG_CONTROL_REG_REMOVED */
        #else /* LOG_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End LOG_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: LOG_SetRxAddress1
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
    void LOG_SetRxAddress1(uint8 address) 
    {
        LOG_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: LOG_SetRxAddress2
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
    void LOG_SetRxAddress2(uint8 address) 
    {
        LOG_RXADDRESS2_REG = address;
    }

#endif  /* LOG_RX_ENABLED || LOG_HD_ENABLED*/


#if( (LOG_TX_ENABLED) || (LOG_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: LOG_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   LOG_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   LOG_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   LOG_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   LOG_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void LOG_SetTxInterruptMode(uint8 intSrc) 
    {
        LOG_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: LOG_WriteTxData
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
    *  LOG_txBuffer - RAM buffer pointer for save data for transmission
    *  LOG_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  LOG_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  LOG_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void LOG_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(LOG_initVar != 0u)
        {
        #if (LOG_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            LOG_DisableTxInt();

            if( (LOG_txBufferRead == LOG_txBufferWrite) &&
                ((LOG_TXSTATUS_REG & LOG_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                LOG_TXDATA_REG = txDataByte;
            }
            else
            {
                if(LOG_txBufferWrite >= LOG_TX_BUFFER_SIZE)
                {
                    LOG_txBufferWrite = 0u;
                }

                LOG_txBuffer[LOG_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                LOG_txBufferWrite++;
            }

            LOG_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            LOG_TXDATA_REG = txDataByte;

        #endif /*(LOG_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: LOG_ReadTxStatus
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
    uint8 LOG_ReadTxStatus(void) 
    {
        return(LOG_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: LOG_PutChar
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
    *  LOG_txBuffer - RAM buffer pointer for save data for transmission
    *  LOG_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  LOG_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  LOG_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void LOG_PutChar(uint8 txDataByte) 
    {
    #if (LOG_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((LOG_TX_BUFFER_SIZE > LOG_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            LOG_DisableTxInt();
        #endif /* (LOG_TX_BUFFER_SIZE > LOG_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = LOG_txBufferWrite;
            locTxBufferRead  = LOG_txBufferRead;

        #if ((LOG_TX_BUFFER_SIZE > LOG_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            LOG_EnableTxInt();
        #endif /* (LOG_TX_BUFFER_SIZE > LOG_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(LOG_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((LOG_TXSTATUS_REG & LOG_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            LOG_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= LOG_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            LOG_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((LOG_TX_BUFFER_SIZE > LOG_MAX_BYTE_VALUE) && (CY_PSOC3))
            LOG_DisableTxInt();
        #endif /* (LOG_TX_BUFFER_SIZE > LOG_MAX_BYTE_VALUE) && (CY_PSOC3) */

            LOG_txBufferWrite = locTxBufferWrite;

        #if ((LOG_TX_BUFFER_SIZE > LOG_MAX_BYTE_VALUE) && (CY_PSOC3))
            LOG_EnableTxInt();
        #endif /* (LOG_TX_BUFFER_SIZE > LOG_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (LOG_TXSTATUS_REG & LOG_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                LOG_SetPendingTxInt();
            }
        }

    #else

        while((LOG_TXSTATUS_REG & LOG_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        LOG_TXDATA_REG = txDataByte;

    #endif /* LOG_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: LOG_PutString
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
    *  LOG_initVar - checked to identify that the component has been
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
    void LOG_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(LOG_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                LOG_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: LOG_PutArray
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
    *  LOG_initVar - checked to identify that the component has been
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
    void LOG_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(LOG_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                LOG_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: LOG_PutCRLF
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
    *  LOG_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void LOG_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(LOG_initVar != 0u)
        {
            LOG_PutChar(txDataByte);
            LOG_PutChar(0x0Du);
            LOG_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: LOG_GetTxBufferSize
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
    *  LOG_txBufferWrite - used to calculate left space.
    *  LOG_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 LOG_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (LOG_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        LOG_DisableTxInt();

        if(LOG_txBufferRead == LOG_txBufferWrite)
        {
            size = 0u;
        }
        else if(LOG_txBufferRead < LOG_txBufferWrite)
        {
            size = (LOG_txBufferWrite - LOG_txBufferRead);
        }
        else
        {
            size = (LOG_TX_BUFFER_SIZE - LOG_txBufferRead) +
                    LOG_txBufferWrite;
        }

        LOG_EnableTxInt();

    #else

        size = LOG_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & LOG_TX_STS_FIFO_FULL) != 0u)
        {
            size = LOG_FIFO_LENGTH;
        }
        else if((size & LOG_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (LOG_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: LOG_ClearTxBuffer
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
    *  LOG_txBufferWrite - cleared to zero.
    *  LOG_txBufferRead - cleared to zero.
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
    void LOG_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        LOG_TXDATA_AUX_CTL_REG |= (uint8)  LOG_TX_FIFO_CLR;
        LOG_TXDATA_AUX_CTL_REG &= (uint8) ~LOG_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (LOG_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        LOG_DisableTxInt();

        LOG_txBufferRead = 0u;
        LOG_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        LOG_EnableTxInt();

    #endif /* (LOG_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: LOG_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   LOG_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   LOG_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   LOG_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   LOG_SEND_WAIT_REINIT - Performs both options: 
    *      LOG_SEND_BREAK and LOG_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  LOG_initVar - checked to identify that the component has been
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
    *     When interrupt appear with LOG_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The LOG_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void LOG_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(LOG_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(LOG_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == LOG_SEND_BREAK) ||
                (retMode == LOG_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                LOG_WriteControlRegister(LOG_ReadControlRegister() |
                                                      LOG_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                LOG_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = LOG_TXSTATUS_REG;
                }
                while((tmpStat & LOG_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == LOG_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == LOG_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = LOG_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & LOG_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == LOG_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == LOG_REINIT) ||
                (retMode == LOG_SEND_WAIT_REINIT) )
            {
                LOG_WriteControlRegister(LOG_ReadControlRegister() &
                                              (uint8)~LOG_CTRL_HD_SEND_BREAK);
            }

        #else /* LOG_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == LOG_SEND_BREAK) ||
                (retMode == LOG_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (LOG_PARITY_TYPE != LOG__B_UART__NONE_REVB) || \
                                    (LOG_PARITY_TYPE_SW != 0u) )
                    LOG_WriteControlRegister(LOG_ReadControlRegister() |
                                                          LOG_CTRL_HD_SEND_BREAK);
                #endif /* End LOG_PARITY_TYPE != LOG__B_UART__NONE_REVB  */

                #if(LOG_TXCLKGEN_DP)
                    txPeriod = LOG_TXBITCLKTX_COMPLETE_REG;
                    LOG_TXBITCLKTX_COMPLETE_REG = LOG_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = LOG_TXBITCTR_PERIOD_REG;
                    LOG_TXBITCTR_PERIOD_REG = LOG_TXBITCTR_BREAKBITS8X;
                #endif /* End LOG_TXCLKGEN_DP */

                /* Send zeros */
                LOG_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = LOG_TXSTATUS_REG;
                }
                while((tmpStat & LOG_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == LOG_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == LOG_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = LOG_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & LOG_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == LOG_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == LOG_REINIT) ||
                (retMode == LOG_SEND_WAIT_REINIT) )
            {

            #if(LOG_TXCLKGEN_DP)
                LOG_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                LOG_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End LOG_TXCLKGEN_DP */

            #if( (LOG_PARITY_TYPE != LOG__B_UART__NONE_REVB) || \
                 (LOG_PARITY_TYPE_SW != 0u) )
                LOG_WriteControlRegister(LOG_ReadControlRegister() &
                                                      (uint8) ~LOG_CTRL_HD_SEND_BREAK);
            #endif /* End LOG_PARITY_TYPE != NONE */
            }
        #endif    /* End LOG_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: LOG_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       LOG_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       LOG_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears LOG_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void LOG_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( LOG_CONTROL_REG_REMOVED == 0u )
            LOG_WriteControlRegister(LOG_ReadControlRegister() |
                                                  LOG_CTRL_MARK);
        #endif /* End LOG_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( LOG_CONTROL_REG_REMOVED == 0u )
            LOG_WriteControlRegister(LOG_ReadControlRegister() &
                                                  (uint8) ~LOG_CTRL_MARK);
        #endif /* End LOG_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndLOG_TX_ENABLED */

#if(LOG_HD_ENABLED)


    /*******************************************************************************
    * Function Name: LOG_LoadRxConfig
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
    void LOG_LoadRxConfig(void) 
    {
        LOG_WriteControlRegister(LOG_ReadControlRegister() &
                                                (uint8)~LOG_CTRL_HD_SEND);
        LOG_RXBITCTR_PERIOD_REG = LOG_HD_RXBITCTR_INIT;

    #if (LOG_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        LOG_SetRxInterruptMode(LOG_INIT_RX_INTERRUPTS_MASK);
    #endif /* (LOG_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: LOG_LoadTxConfig
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
    void LOG_LoadTxConfig(void) 
    {
    #if (LOG_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        LOG_SetRxInterruptMode(0u);
    #endif /* (LOG_RX_INTERRUPT_ENABLED) */

        LOG_WriteControlRegister(LOG_ReadControlRegister() | LOG_CTRL_HD_SEND);
        LOG_RXBITCTR_PERIOD_REG = LOG_HD_TXBITCTR_INIT;
    }

#endif  /* LOG_HD_ENABLED */


/* [] END OF FILE */
