/*******************************************************************************
* File Name: WIFIdev_Channel_BOOT.c
* Version 2.50
*
* Description:
*  This file provides the source code of bootloader communication APIs for the
*  UART component.
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

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (0u != ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_WIFIdev_Channel) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)))


/*******************************************************************************
* Function Name: WIFIdev_Channel_CyBtldrCommStart
********************************************************************************
*
* Summary:
*  Starts the UART communication component.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  This component automatically enables global interrupt.
*
*******************************************************************************/
void WIFIdev_Channel_CyBtldrCommStart(void) CYSMALL 
{
    /* Start UART component and clear the Tx,Rx buffers */
    WIFIdev_Channel_Start();
    WIFIdev_Channel_ClearRxBuffer();
    WIFIdev_Channel_ClearTxBuffer();
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_CyBtldrCommStop
********************************************************************************
*
* Summary:
*  Disables the communication component and disables the interrupt.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void WIFIdev_Channel_CyBtldrCommStop(void) CYSMALL 
{
    /* Stop UART component */
    WIFIdev_Channel_Stop();
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_CyBtldrCommReset
********************************************************************************
*
* Summary:
*  Resets the receive and transmit communication Buffers.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void WIFIdev_Channel_CyBtldrCommReset(void) CYSMALL 
{
    /* Clear RX and TX buffers */
    WIFIdev_Channel_ClearRxBuffer();
    WIFIdev_Channel_ClearTxBuffer();
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_CyBtldrCommWrite
********************************************************************************
*
* Summary:
*  Allows the caller to write data to the boot loader host. This function uses
* a blocking write function for writing data using UART communication component.
*
* Parameters:
*  pData:    A pointer to the block of data to send to the device
*  size:     The number of bytes to write.
*  count:    Pointer to an unsigned short variable to write the number of
*             bytes actually written.
*  timeOut:  Number of units to wait before returning because of a timeout.
*
* Return:
*   cystatus: This function will return CYRET_SUCCESS if data is sent
*             successfully.
*
* Side Effects:
*  This function should be called after command was received .
*
*******************************************************************************/
cystatus WIFIdev_Channel_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
         
{
    uint16 bufIndex = 0u;

    if(0u != timeOut)
    {
        /* Suppress compiler warning */
    }

    /* Clear receive buffers */
    WIFIdev_Channel_ClearRxBuffer();

    /* Write TX data using blocking function */
    while(bufIndex < size)
    {
        WIFIdev_Channel_PutChar(pData[bufIndex]);
        bufIndex++;
    }

    /* Return success code */
    *count = size;

    return (CYRET_SUCCESS);
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_CyBtldrCommRead
********************************************************************************
*
* Summary:
*  Receives the command.
*
* Parameters:
*  pData:    A pointer to the area to store the block of data received
*             from the device.
*  size:     Maximum size of the read buffer
*  count:    Pointer to an unsigned short variable to write the number
*             of bytes actually read.
*  timeOut:  Number of units to wait before returning because of a timeOut.
*            Time out is measured in 10s of ms.
*
* Return:
*  cystatus: This function will return CYRET_SUCCESS if at least one byte is
*            received successfully within the time out interval. If no data is
*            received  this function will return CYRET_EMPTY.
*
*  BYTE2BYTE_TIME_OUT is used for detecting time out marking end of block data
*  from host. This has to be set to a value which is greater than the expected
*  maximum delay between two bytes during a block/packet transmission from the
*  host. You have to account for the delay in hardware converters while
*  calculating this value, if you are using any USB-UART bridges.
*******************************************************************************/
cystatus WIFIdev_Channel_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
         
{
    uint16 iCntr;
    uint16 dataIndexCntr;
    uint16 tempCount;
    uint16 oldDataCount;

    cystatus status = CYRET_EMPTY;

    /* Check whether data is received within the time out period.
    *  Time out period is in units of 10ms.
    *  If at least one byte is received within the time out interval, wait for more data */
    for (iCntr = 0u; iCntr < ((uint16)10u * timeOut); iCntr++)
    {
        /* If at least one byte is received within the timeout interval
        *  enter the next loop waiting for more data reception
        */
        if(0u != WIFIdev_Channel_GetRxBufferSize())
        {
            /* Wait for more data until 25ms byte to byte time out interval.
            * If no data is received during the last 25 ms(BYTE2BYTE_TIME_OUT)
            * then it is considered as end of transmitted data block(packet)
            * from the host and the program execution will break from the
            * data awaiting loop with status=CYRET_SUCCESS
            */
            do
            {
                oldDataCount = WIFIdev_Channel_GetRxBufferSize();
                CyDelay(WIFIdev_Channel_BYTE2BYTE_TIME_OUT);
            }
            while(WIFIdev_Channel_GetRxBufferSize() > oldDataCount);

            status = CYRET_SUCCESS;
            break;
        }
        /* If the data is not received, give a delay of 
        *  WIFIdev_Channel_BL_CHK_DELAY_MS and check again until the timeOut specified.
        */
        else
        {
            CyDelay(WIFIdev_Channel_BL_CHK_DELAY_MS);
        }
    }

    /* Initialize the data read indexes and Count value */
    *count = 0u;
    dataIndexCntr = 0u;

    /* If GetRxBufferSize()>0 , move the received data to the pData buffer */
    while(WIFIdev_Channel_GetRxBufferSize() > 0u)
    {
        tempCount = WIFIdev_Channel_GetRxBufferSize();
        *count  =(*count) + tempCount;

        /* Check if buffer overflow will occur before moving the data */
        if(*count < size)
        {
            for (iCntr = 0u; iCntr < tempCount; iCntr++)
            {
                /* Read the data and move it to the pData buffer */
                pData[dataIndexCntr] = WIFIdev_Channel_ReadRxData();
                dataIndexCntr++;
            }

            /* Check if the last received byte is end of packet defined by bootloader
            *  If not wait for additional WIFIdev_Channel_WAIT_EOP_DELAY ms.
            */
            if(pData[dataIndexCntr - 1u] != WIFIdev_Channel_PACKET_EOP)
            {
                CyDelay(WIFIdev_Channel_WAIT_EOP_DELAY);
            }
        }
        /* If there is no space to move data, break from the loop */
        else
        {
            *count = (*count) - tempCount;
            break;
        }
    }

    return (status);
}

#endif /* end CYDEV_BOOTLOADER_IO_COMP */


/* [] END OF FILE */
