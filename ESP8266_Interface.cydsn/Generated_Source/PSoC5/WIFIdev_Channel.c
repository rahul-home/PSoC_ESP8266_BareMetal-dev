/*******************************************************************************
* File Name: WIFIdev_Channel.c
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

#include "WIFIdev_Channel.h"
#if (WIFIdev_Channel_INTERNAL_CLOCK_USED)
    #include "WIFIdev_Channel_IntClock.h"
#endif /* End WIFIdev_Channel_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 WIFIdev_Channel_initVar = 0u;

#if (WIFIdev_Channel_TX_INTERRUPT_ENABLED && WIFIdev_Channel_TX_ENABLED)
    volatile uint8 WIFIdev_Channel_txBuffer[WIFIdev_Channel_TX_BUFFER_SIZE];
    volatile uint8 WIFIdev_Channel_txBufferRead = 0u;
    uint8 WIFIdev_Channel_txBufferWrite = 0u;
#endif /* (WIFIdev_Channel_TX_INTERRUPT_ENABLED && WIFIdev_Channel_TX_ENABLED) */

#if (WIFIdev_Channel_RX_INTERRUPT_ENABLED && (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED))
    uint8 WIFIdev_Channel_errorStatus = 0u;
    volatile uint8 WIFIdev_Channel_rxBuffer[WIFIdev_Channel_RX_BUFFER_SIZE];
    volatile uint8 WIFIdev_Channel_rxBufferRead  = 0u;
    volatile uint8 WIFIdev_Channel_rxBufferWrite = 0u;
    volatile uint8 WIFIdev_Channel_rxBufferLoopDetect = 0u;
    volatile uint8 WIFIdev_Channel_rxBufferOverflow   = 0u;
    #if (WIFIdev_Channel_RXHW_ADDRESS_ENABLED)
        volatile uint8 WIFIdev_Channel_rxAddressMode = WIFIdev_Channel_RX_ADDRESS_MODE;
        volatile uint8 WIFIdev_Channel_rxAddressDetected = 0u;
    #endif /* (WIFIdev_Channel_RXHW_ADDRESS_ENABLED) */
#endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED && (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED)) */


/*******************************************************************************
* Function Name: WIFIdev_Channel_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  WIFIdev_Channel_Start() sets the initVar variable, calls the
*  WIFIdev_Channel_Init() function, and then calls the
*  WIFIdev_Channel_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The WIFIdev_Channel_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time WIFIdev_Channel_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the WIFIdev_Channel_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void WIFIdev_Channel_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(WIFIdev_Channel_initVar == 0u)
    {
        WIFIdev_Channel_Init();
        WIFIdev_Channel_initVar = 1u;
    }

    WIFIdev_Channel_Enable();
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call WIFIdev_Channel_Init() because
*  the WIFIdev_Channel_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void WIFIdev_Channel_Init(void) 
{
    #if(WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED)

        #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(WIFIdev_Channel_RX_VECT_NUM, &WIFIdev_Channel_RXISR);
            CyIntSetPriority(WIFIdev_Channel_RX_VECT_NUM, WIFIdev_Channel_RX_PRIOR_NUM);
            WIFIdev_Channel_errorStatus = 0u;
        #endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED) */

        #if (WIFIdev_Channel_RXHW_ADDRESS_ENABLED)
            WIFIdev_Channel_SetRxAddressMode(WIFIdev_Channel_RX_ADDRESS_MODE);
            WIFIdev_Channel_SetRxAddress1(WIFIdev_Channel_RX_HW_ADDRESS1);
            WIFIdev_Channel_SetRxAddress2(WIFIdev_Channel_RX_HW_ADDRESS2);
        #endif /* End WIFIdev_Channel_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        WIFIdev_Channel_RXBITCTR_PERIOD_REG = WIFIdev_Channel_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        WIFIdev_Channel_RXSTATUS_MASK_REG  = WIFIdev_Channel_INIT_RX_INTERRUPTS_MASK;
    #endif /* End WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED*/

    #if(WIFIdev_Channel_TX_ENABLED)
        #if (WIFIdev_Channel_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(WIFIdev_Channel_TX_VECT_NUM, &WIFIdev_Channel_TXISR);
            CyIntSetPriority(WIFIdev_Channel_TX_VECT_NUM, WIFIdev_Channel_TX_PRIOR_NUM);
        #endif /* (WIFIdev_Channel_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (WIFIdev_Channel_TXCLKGEN_DP)
            WIFIdev_Channel_TXBITCLKGEN_CTR_REG = WIFIdev_Channel_BIT_CENTER;
            WIFIdev_Channel_TXBITCLKTX_COMPLETE_REG = ((WIFIdev_Channel_NUMBER_OF_DATA_BITS +
                        WIFIdev_Channel_NUMBER_OF_START_BIT) * WIFIdev_Channel_OVER_SAMPLE_COUNT) - 1u;
        #else
            WIFIdev_Channel_TXBITCTR_PERIOD_REG = ((WIFIdev_Channel_NUMBER_OF_DATA_BITS +
                        WIFIdev_Channel_NUMBER_OF_START_BIT) * WIFIdev_Channel_OVER_SAMPLE_8) - 1u;
        #endif /* End WIFIdev_Channel_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (WIFIdev_Channel_TX_INTERRUPT_ENABLED)
            WIFIdev_Channel_TXSTATUS_MASK_REG = WIFIdev_Channel_TX_STS_FIFO_EMPTY;
        #else
            WIFIdev_Channel_TXSTATUS_MASK_REG = WIFIdev_Channel_INIT_TX_INTERRUPTS_MASK;
        #endif /*End WIFIdev_Channel_TX_INTERRUPT_ENABLED*/

    #endif /* End WIFIdev_Channel_TX_ENABLED */

    #if(WIFIdev_Channel_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        WIFIdev_Channel_WriteControlRegister( \
            (WIFIdev_Channel_ReadControlRegister() & (uint8)~WIFIdev_Channel_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(WIFIdev_Channel_PARITY_TYPE << WIFIdev_Channel_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End WIFIdev_Channel_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call WIFIdev_Channel_Enable() because the WIFIdev_Channel_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  WIFIdev_Channel_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void WIFIdev_Channel_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        WIFIdev_Channel_RXBITCTR_CONTROL_REG |= WIFIdev_Channel_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        WIFIdev_Channel_RXSTATUS_ACTL_REG  |= WIFIdev_Channel_INT_ENABLE;

        #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)
            WIFIdev_Channel_EnableRxInt();

            #if (WIFIdev_Channel_RXHW_ADDRESS_ENABLED)
                WIFIdev_Channel_rxAddressDetected = 0u;
            #endif /* (WIFIdev_Channel_RXHW_ADDRESS_ENABLED) */
        #endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED) */
    #endif /* (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED) */

    #if(WIFIdev_Channel_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!WIFIdev_Channel_TXCLKGEN_DP)
            WIFIdev_Channel_TXBITCTR_CONTROL_REG |= WIFIdev_Channel_CNTR_ENABLE;
        #endif /* End WIFIdev_Channel_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        WIFIdev_Channel_TXSTATUS_ACTL_REG |= WIFIdev_Channel_INT_ENABLE;
        #if (WIFIdev_Channel_TX_INTERRUPT_ENABLED)
            WIFIdev_Channel_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            WIFIdev_Channel_EnableTxInt();
        #endif /* (WIFIdev_Channel_TX_INTERRUPT_ENABLED) */
     #endif /* (WIFIdev_Channel_TX_INTERRUPT_ENABLED) */

    #if (WIFIdev_Channel_INTERNAL_CLOCK_USED)
        WIFIdev_Channel_IntClock_Start();  /* Enable the clock */
    #endif /* (WIFIdev_Channel_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_Stop
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
void WIFIdev_Channel_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED)
        WIFIdev_Channel_RXBITCTR_CONTROL_REG &= (uint8) ~WIFIdev_Channel_CNTR_ENABLE;
    #endif /* (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED) */

    #if (WIFIdev_Channel_TX_ENABLED)
        #if(!WIFIdev_Channel_TXCLKGEN_DP)
            WIFIdev_Channel_TXBITCTR_CONTROL_REG &= (uint8) ~WIFIdev_Channel_CNTR_ENABLE;
        #endif /* (!WIFIdev_Channel_TXCLKGEN_DP) */
    #endif /* (WIFIdev_Channel_TX_ENABLED) */

    #if (WIFIdev_Channel_INTERNAL_CLOCK_USED)
        WIFIdev_Channel_IntClock_Stop();   /* Disable the clock */
    #endif /* (WIFIdev_Channel_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED)
        WIFIdev_Channel_RXSTATUS_ACTL_REG  &= (uint8) ~WIFIdev_Channel_INT_ENABLE;

        #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)
            WIFIdev_Channel_DisableRxInt();
        #endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED) */
    #endif /* (WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED) */

    #if (WIFIdev_Channel_TX_ENABLED)
        WIFIdev_Channel_TXSTATUS_ACTL_REG &= (uint8) ~WIFIdev_Channel_INT_ENABLE;

        #if (WIFIdev_Channel_TX_INTERRUPT_ENABLED)
            WIFIdev_Channel_DisableTxInt();
        #endif /* (WIFIdev_Channel_TX_INTERRUPT_ENABLED) */
    #endif /* (WIFIdev_Channel_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_ReadControlRegister
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
uint8 WIFIdev_Channel_ReadControlRegister(void) 
{
    #if (WIFIdev_Channel_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(WIFIdev_Channel_CONTROL_REG);
    #endif /* (WIFIdev_Channel_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_WriteControlRegister
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
void  WIFIdev_Channel_WriteControlRegister(uint8 control) 
{
    #if (WIFIdev_Channel_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       WIFIdev_Channel_CONTROL_REG = control;
    #endif /* (WIFIdev_Channel_CONTROL_REG_REMOVED) */
}


#if(WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED)
    /*******************************************************************************
    * Function Name: WIFIdev_Channel_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      WIFIdev_Channel_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      WIFIdev_Channel_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      WIFIdev_Channel_RX_STS_BREAK            Interrupt on break.
    *      WIFIdev_Channel_RX_STS_OVERRUN          Interrupt on overrun error.
    *      WIFIdev_Channel_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      WIFIdev_Channel_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void WIFIdev_Channel_SetRxInterruptMode(uint8 intSrc) 
    {
        WIFIdev_Channel_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_ReadRxData
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
    *  WIFIdev_Channel_rxBuffer - RAM buffer pointer for save received data.
    *  WIFIdev_Channel_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  WIFIdev_Channel_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  WIFIdev_Channel_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 WIFIdev_Channel_ReadRxData(void) 
    {
        uint8 rxData;

    #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        WIFIdev_Channel_DisableRxInt();

        locRxBufferRead  = WIFIdev_Channel_rxBufferRead;
        locRxBufferWrite = WIFIdev_Channel_rxBufferWrite;

        if( (WIFIdev_Channel_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = WIFIdev_Channel_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= WIFIdev_Channel_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            WIFIdev_Channel_rxBufferRead = locRxBufferRead;

            if(WIFIdev_Channel_rxBufferLoopDetect != 0u)
            {
                WIFIdev_Channel_rxBufferLoopDetect = 0u;
                #if ((WIFIdev_Channel_RX_INTERRUPT_ENABLED) && (WIFIdev_Channel_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( WIFIdev_Channel_HD_ENABLED )
                        if((WIFIdev_Channel_CONTROL_REG & WIFIdev_Channel_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            WIFIdev_Channel_RXSTATUS_MASK_REG  |= WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        WIFIdev_Channel_RXSTATUS_MASK_REG  |= WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end WIFIdev_Channel_HD_ENABLED */
                #endif /* ((WIFIdev_Channel_RX_INTERRUPT_ENABLED) && (WIFIdev_Channel_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = WIFIdev_Channel_RXDATA_REG;
        }

        WIFIdev_Channel_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = WIFIdev_Channel_RXDATA_REG;

    #endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_ReadRxStatus
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
    *  WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY.
    *  WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  WIFIdev_Channel_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   WIFIdev_Channel_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   WIFIdev_Channel_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 WIFIdev_Channel_ReadRxStatus(void) 
    {
        uint8 status;

        status = WIFIdev_Channel_RXSTATUS_REG & WIFIdev_Channel_RX_HW_MASK;

    #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)
        if(WIFIdev_Channel_rxBufferOverflow != 0u)
        {
            status |= WIFIdev_Channel_RX_STS_SOFT_BUFF_OVER;
            WIFIdev_Channel_rxBufferOverflow = 0u;
        }
    #endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. WIFIdev_Channel_GetChar() is
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
    *  WIFIdev_Channel_rxBuffer - RAM buffer pointer for save received data.
    *  WIFIdev_Channel_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  WIFIdev_Channel_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  WIFIdev_Channel_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 WIFIdev_Channel_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        WIFIdev_Channel_DisableRxInt();

        locRxBufferRead  = WIFIdev_Channel_rxBufferRead;
        locRxBufferWrite = WIFIdev_Channel_rxBufferWrite;

        if( (WIFIdev_Channel_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = WIFIdev_Channel_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= WIFIdev_Channel_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            WIFIdev_Channel_rxBufferRead = locRxBufferRead;

            if(WIFIdev_Channel_rxBufferLoopDetect != 0u)
            {
                WIFIdev_Channel_rxBufferLoopDetect = 0u;
                #if( (WIFIdev_Channel_RX_INTERRUPT_ENABLED) && (WIFIdev_Channel_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( WIFIdev_Channel_HD_ENABLED )
                        if((WIFIdev_Channel_CONTROL_REG & WIFIdev_Channel_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            WIFIdev_Channel_RXSTATUS_MASK_REG |= WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        WIFIdev_Channel_RXSTATUS_MASK_REG |= WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end WIFIdev_Channel_HD_ENABLED */
                #endif /* WIFIdev_Channel_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = WIFIdev_Channel_RXSTATUS_REG;
            if((rxStatus & WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = WIFIdev_Channel_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (WIFIdev_Channel_RX_STS_BREAK | WIFIdev_Channel_RX_STS_PAR_ERROR |
                                WIFIdev_Channel_RX_STS_STOP_ERROR | WIFIdev_Channel_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        WIFIdev_Channel_EnableRxInt();

    #else

        rxStatus =WIFIdev_Channel_RXSTATUS_REG;
        if((rxStatus & WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = WIFIdev_Channel_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (WIFIdev_Channel_RX_STS_BREAK | WIFIdev_Channel_RX_STS_PAR_ERROR |
                            WIFIdev_Channel_RX_STS_STOP_ERROR | WIFIdev_Channel_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_GetByte
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
    uint16 WIFIdev_Channel_GetByte(void) 
    {
        
    #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        WIFIdev_Channel_DisableRxInt();
        locErrorStatus = (uint16)WIFIdev_Channel_errorStatus;
        WIFIdev_Channel_errorStatus = 0u;
        WIFIdev_Channel_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | WIFIdev_Channel_ReadRxData() );
    #else
        return ( ((uint16)WIFIdev_Channel_ReadRxStatus() << 8u) | WIFIdev_Channel_ReadRxData() );
    #endif /* WIFIdev_Channel_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_GetRxBufferSize
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
    *  WIFIdev_Channel_rxBufferWrite - used to calculate left bytes.
    *  WIFIdev_Channel_rxBufferRead - used to calculate left bytes.
    *  WIFIdev_Channel_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 WIFIdev_Channel_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        WIFIdev_Channel_DisableRxInt();

        if(WIFIdev_Channel_rxBufferRead == WIFIdev_Channel_rxBufferWrite)
        {
            if(WIFIdev_Channel_rxBufferLoopDetect != 0u)
            {
                size = WIFIdev_Channel_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(WIFIdev_Channel_rxBufferRead < WIFIdev_Channel_rxBufferWrite)
        {
            size = (WIFIdev_Channel_rxBufferWrite - WIFIdev_Channel_rxBufferRead);
        }
        else
        {
            size = (WIFIdev_Channel_RX_BUFFER_SIZE - WIFIdev_Channel_rxBufferRead) + WIFIdev_Channel_rxBufferWrite;
        }

        WIFIdev_Channel_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((WIFIdev_Channel_RXSTATUS_REG & WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_ClearRxBuffer
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
    *  WIFIdev_Channel_rxBufferWrite - cleared to zero.
    *  WIFIdev_Channel_rxBufferRead - cleared to zero.
    *  WIFIdev_Channel_rxBufferLoopDetect - cleared to zero.
    *  WIFIdev_Channel_rxBufferOverflow - cleared to zero.
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
    void WIFIdev_Channel_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        WIFIdev_Channel_RXDATA_AUX_CTL_REG |= (uint8)  WIFIdev_Channel_RX_FIFO_CLR;
        WIFIdev_Channel_RXDATA_AUX_CTL_REG &= (uint8) ~WIFIdev_Channel_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        WIFIdev_Channel_DisableRxInt();

        WIFIdev_Channel_rxBufferRead = 0u;
        WIFIdev_Channel_rxBufferWrite = 0u;
        WIFIdev_Channel_rxBufferLoopDetect = 0u;
        WIFIdev_Channel_rxBufferOverflow = 0u;

        WIFIdev_Channel_EnableRxInt();

    #endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  WIFIdev_Channel__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  WIFIdev_Channel__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  WIFIdev_Channel__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  WIFIdev_Channel__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  WIFIdev_Channel__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WIFIdev_Channel_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  WIFIdev_Channel_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void WIFIdev_Channel_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(WIFIdev_Channel_RXHW_ADDRESS_ENABLED)
            #if(WIFIdev_Channel_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* WIFIdev_Channel_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = WIFIdev_Channel_CONTROL_REG & (uint8)~WIFIdev_Channel_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << WIFIdev_Channel_CTRL_RXADDR_MODE0_SHIFT);
                WIFIdev_Channel_CONTROL_REG = tmpCtrl;

                #if(WIFIdev_Channel_RX_INTERRUPT_ENABLED && \
                   (WIFIdev_Channel_RXBUFFERSIZE > WIFIdev_Channel_FIFO_LENGTH) )
                    WIFIdev_Channel_rxAddressMode = addressMode;
                    WIFIdev_Channel_rxAddressDetected = 0u;
                #endif /* End WIFIdev_Channel_RXBUFFERSIZE > WIFIdev_Channel_FIFO_LENGTH*/
            #endif /* End WIFIdev_Channel_CONTROL_REG_REMOVED */
        #else /* WIFIdev_Channel_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End WIFIdev_Channel_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_SetRxAddress1
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
    void WIFIdev_Channel_SetRxAddress1(uint8 address) 
    {
        WIFIdev_Channel_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_SetRxAddress2
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
    void WIFIdev_Channel_SetRxAddress2(uint8 address) 
    {
        WIFIdev_Channel_RXADDRESS2_REG = address;
    }

#endif  /* WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED*/


#if( (WIFIdev_Channel_TX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: WIFIdev_Channel_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   WIFIdev_Channel_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   WIFIdev_Channel_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   WIFIdev_Channel_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   WIFIdev_Channel_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void WIFIdev_Channel_SetTxInterruptMode(uint8 intSrc) 
    {
        WIFIdev_Channel_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_WriteTxData
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
    *  WIFIdev_Channel_txBuffer - RAM buffer pointer for save data for transmission
    *  WIFIdev_Channel_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  WIFIdev_Channel_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  WIFIdev_Channel_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void WIFIdev_Channel_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(WIFIdev_Channel_initVar != 0u)
        {
        #if (WIFIdev_Channel_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            WIFIdev_Channel_DisableTxInt();

            if( (WIFIdev_Channel_txBufferRead == WIFIdev_Channel_txBufferWrite) &&
                ((WIFIdev_Channel_TXSTATUS_REG & WIFIdev_Channel_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                WIFIdev_Channel_TXDATA_REG = txDataByte;
            }
            else
            {
                if(WIFIdev_Channel_txBufferWrite >= WIFIdev_Channel_TX_BUFFER_SIZE)
                {
                    WIFIdev_Channel_txBufferWrite = 0u;
                }

                WIFIdev_Channel_txBuffer[WIFIdev_Channel_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                WIFIdev_Channel_txBufferWrite++;
            }

            WIFIdev_Channel_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            WIFIdev_Channel_TXDATA_REG = txDataByte;

        #endif /*(WIFIdev_Channel_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_ReadTxStatus
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
    uint8 WIFIdev_Channel_ReadTxStatus(void) 
    {
        return(WIFIdev_Channel_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_PutChar
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
    *  WIFIdev_Channel_txBuffer - RAM buffer pointer for save data for transmission
    *  WIFIdev_Channel_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  WIFIdev_Channel_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  WIFIdev_Channel_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void WIFIdev_Channel_PutChar(uint8 txDataByte) 
    {
    #if (WIFIdev_Channel_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((WIFIdev_Channel_TX_BUFFER_SIZE > WIFIdev_Channel_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            WIFIdev_Channel_DisableTxInt();
        #endif /* (WIFIdev_Channel_TX_BUFFER_SIZE > WIFIdev_Channel_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = WIFIdev_Channel_txBufferWrite;
            locTxBufferRead  = WIFIdev_Channel_txBufferRead;

        #if ((WIFIdev_Channel_TX_BUFFER_SIZE > WIFIdev_Channel_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            WIFIdev_Channel_EnableTxInt();
        #endif /* (WIFIdev_Channel_TX_BUFFER_SIZE > WIFIdev_Channel_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(WIFIdev_Channel_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((WIFIdev_Channel_TXSTATUS_REG & WIFIdev_Channel_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            WIFIdev_Channel_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= WIFIdev_Channel_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            WIFIdev_Channel_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((WIFIdev_Channel_TX_BUFFER_SIZE > WIFIdev_Channel_MAX_BYTE_VALUE) && (CY_PSOC3))
            WIFIdev_Channel_DisableTxInt();
        #endif /* (WIFIdev_Channel_TX_BUFFER_SIZE > WIFIdev_Channel_MAX_BYTE_VALUE) && (CY_PSOC3) */

            WIFIdev_Channel_txBufferWrite = locTxBufferWrite;

        #if ((WIFIdev_Channel_TX_BUFFER_SIZE > WIFIdev_Channel_MAX_BYTE_VALUE) && (CY_PSOC3))
            WIFIdev_Channel_EnableTxInt();
        #endif /* (WIFIdev_Channel_TX_BUFFER_SIZE > WIFIdev_Channel_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (WIFIdev_Channel_TXSTATUS_REG & WIFIdev_Channel_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                WIFIdev_Channel_SetPendingTxInt();
            }
        }

    #else

        while((WIFIdev_Channel_TXSTATUS_REG & WIFIdev_Channel_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        WIFIdev_Channel_TXDATA_REG = txDataByte;

    #endif /* WIFIdev_Channel_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_PutString
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
    *  WIFIdev_Channel_initVar - checked to identify that the component has been
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
    void WIFIdev_Channel_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(WIFIdev_Channel_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                WIFIdev_Channel_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_PutArray
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
    *  WIFIdev_Channel_initVar - checked to identify that the component has been
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
    void WIFIdev_Channel_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(WIFIdev_Channel_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                WIFIdev_Channel_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_PutCRLF
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
    *  WIFIdev_Channel_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void WIFIdev_Channel_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(WIFIdev_Channel_initVar != 0u)
        {
            WIFIdev_Channel_PutChar(txDataByte);
            WIFIdev_Channel_PutChar(0x0Du);
            WIFIdev_Channel_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_GetTxBufferSize
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
    *  WIFIdev_Channel_txBufferWrite - used to calculate left space.
    *  WIFIdev_Channel_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 WIFIdev_Channel_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (WIFIdev_Channel_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        WIFIdev_Channel_DisableTxInt();

        if(WIFIdev_Channel_txBufferRead == WIFIdev_Channel_txBufferWrite)
        {
            size = 0u;
        }
        else if(WIFIdev_Channel_txBufferRead < WIFIdev_Channel_txBufferWrite)
        {
            size = (WIFIdev_Channel_txBufferWrite - WIFIdev_Channel_txBufferRead);
        }
        else
        {
            size = (WIFIdev_Channel_TX_BUFFER_SIZE - WIFIdev_Channel_txBufferRead) +
                    WIFIdev_Channel_txBufferWrite;
        }

        WIFIdev_Channel_EnableTxInt();

    #else

        size = WIFIdev_Channel_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & WIFIdev_Channel_TX_STS_FIFO_FULL) != 0u)
        {
            size = WIFIdev_Channel_FIFO_LENGTH;
        }
        else if((size & WIFIdev_Channel_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (WIFIdev_Channel_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_ClearTxBuffer
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
    *  WIFIdev_Channel_txBufferWrite - cleared to zero.
    *  WIFIdev_Channel_txBufferRead - cleared to zero.
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
    void WIFIdev_Channel_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        WIFIdev_Channel_TXDATA_AUX_CTL_REG |= (uint8)  WIFIdev_Channel_TX_FIFO_CLR;
        WIFIdev_Channel_TXDATA_AUX_CTL_REG &= (uint8) ~WIFIdev_Channel_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (WIFIdev_Channel_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        WIFIdev_Channel_DisableTxInt();

        WIFIdev_Channel_txBufferRead = 0u;
        WIFIdev_Channel_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        WIFIdev_Channel_EnableTxInt();

    #endif /* (WIFIdev_Channel_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   WIFIdev_Channel_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   WIFIdev_Channel_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   WIFIdev_Channel_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   WIFIdev_Channel_SEND_WAIT_REINIT - Performs both options: 
    *      WIFIdev_Channel_SEND_BREAK and WIFIdev_Channel_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  WIFIdev_Channel_initVar - checked to identify that the component has been
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
    *     When interrupt appear with WIFIdev_Channel_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The WIFIdev_Channel_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void WIFIdev_Channel_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(WIFIdev_Channel_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(WIFIdev_Channel_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == WIFIdev_Channel_SEND_BREAK) ||
                (retMode == WIFIdev_Channel_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                WIFIdev_Channel_WriteControlRegister(WIFIdev_Channel_ReadControlRegister() |
                                                      WIFIdev_Channel_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                WIFIdev_Channel_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = WIFIdev_Channel_TXSTATUS_REG;
                }
                while((tmpStat & WIFIdev_Channel_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == WIFIdev_Channel_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == WIFIdev_Channel_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = WIFIdev_Channel_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & WIFIdev_Channel_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == WIFIdev_Channel_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == WIFIdev_Channel_REINIT) ||
                (retMode == WIFIdev_Channel_SEND_WAIT_REINIT) )
            {
                WIFIdev_Channel_WriteControlRegister(WIFIdev_Channel_ReadControlRegister() &
                                              (uint8)~WIFIdev_Channel_CTRL_HD_SEND_BREAK);
            }

        #else /* WIFIdev_Channel_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == WIFIdev_Channel_SEND_BREAK) ||
                (retMode == WIFIdev_Channel_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (WIFIdev_Channel_PARITY_TYPE != WIFIdev_Channel__B_UART__NONE_REVB) || \
                                    (WIFIdev_Channel_PARITY_TYPE_SW != 0u) )
                    WIFIdev_Channel_WriteControlRegister(WIFIdev_Channel_ReadControlRegister() |
                                                          WIFIdev_Channel_CTRL_HD_SEND_BREAK);
                #endif /* End WIFIdev_Channel_PARITY_TYPE != WIFIdev_Channel__B_UART__NONE_REVB  */

                #if(WIFIdev_Channel_TXCLKGEN_DP)
                    txPeriod = WIFIdev_Channel_TXBITCLKTX_COMPLETE_REG;
                    WIFIdev_Channel_TXBITCLKTX_COMPLETE_REG = WIFIdev_Channel_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = WIFIdev_Channel_TXBITCTR_PERIOD_REG;
                    WIFIdev_Channel_TXBITCTR_PERIOD_REG = WIFIdev_Channel_TXBITCTR_BREAKBITS8X;
                #endif /* End WIFIdev_Channel_TXCLKGEN_DP */

                /* Send zeros */
                WIFIdev_Channel_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = WIFIdev_Channel_TXSTATUS_REG;
                }
                while((tmpStat & WIFIdev_Channel_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == WIFIdev_Channel_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == WIFIdev_Channel_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = WIFIdev_Channel_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & WIFIdev_Channel_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == WIFIdev_Channel_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == WIFIdev_Channel_REINIT) ||
                (retMode == WIFIdev_Channel_SEND_WAIT_REINIT) )
            {

            #if(WIFIdev_Channel_TXCLKGEN_DP)
                WIFIdev_Channel_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                WIFIdev_Channel_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End WIFIdev_Channel_TXCLKGEN_DP */

            #if( (WIFIdev_Channel_PARITY_TYPE != WIFIdev_Channel__B_UART__NONE_REVB) || \
                 (WIFIdev_Channel_PARITY_TYPE_SW != 0u) )
                WIFIdev_Channel_WriteControlRegister(WIFIdev_Channel_ReadControlRegister() &
                                                      (uint8) ~WIFIdev_Channel_CTRL_HD_SEND_BREAK);
            #endif /* End WIFIdev_Channel_PARITY_TYPE != NONE */
            }
        #endif    /* End WIFIdev_Channel_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       WIFIdev_Channel_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       WIFIdev_Channel_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears WIFIdev_Channel_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void WIFIdev_Channel_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( WIFIdev_Channel_CONTROL_REG_REMOVED == 0u )
            WIFIdev_Channel_WriteControlRegister(WIFIdev_Channel_ReadControlRegister() |
                                                  WIFIdev_Channel_CTRL_MARK);
        #endif /* End WIFIdev_Channel_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( WIFIdev_Channel_CONTROL_REG_REMOVED == 0u )
            WIFIdev_Channel_WriteControlRegister(WIFIdev_Channel_ReadControlRegister() &
                                                  (uint8) ~WIFIdev_Channel_CTRL_MARK);
        #endif /* End WIFIdev_Channel_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndWIFIdev_Channel_TX_ENABLED */

#if(WIFIdev_Channel_HD_ENABLED)


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_LoadRxConfig
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
    void WIFIdev_Channel_LoadRxConfig(void) 
    {
        WIFIdev_Channel_WriteControlRegister(WIFIdev_Channel_ReadControlRegister() &
                                                (uint8)~WIFIdev_Channel_CTRL_HD_SEND);
        WIFIdev_Channel_RXBITCTR_PERIOD_REG = WIFIdev_Channel_HD_RXBITCTR_INIT;

    #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        WIFIdev_Channel_SetRxInterruptMode(WIFIdev_Channel_INIT_RX_INTERRUPTS_MASK);
    #endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: WIFIdev_Channel_LoadTxConfig
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
    void WIFIdev_Channel_LoadTxConfig(void) 
    {
    #if (WIFIdev_Channel_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        WIFIdev_Channel_SetRxInterruptMode(0u);
    #endif /* (WIFIdev_Channel_RX_INTERRUPT_ENABLED) */

        WIFIdev_Channel_WriteControlRegister(WIFIdev_Channel_ReadControlRegister() | WIFIdev_Channel_CTRL_HD_SEND);
        WIFIdev_Channel_RXBITCTR_PERIOD_REG = WIFIdev_Channel_HD_TXBITCTR_INIT;
    }

#endif  /* WIFIdev_Channel_HD_ENABLED */


/* [] END OF FILE */
