/*******************************************************************************
* File Name: WIFIdev_Channel_PM.c
* Version 2.50
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
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


/***************************************
* Local data allocation
***************************************/

static WIFIdev_Channel_BACKUP_STRUCT  WIFIdev_Channel_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: WIFIdev_Channel_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the WIFIdev_Channel_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  WIFIdev_Channel_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void WIFIdev_Channel_SaveConfig(void)
{
    #if(WIFIdev_Channel_CONTROL_REG_REMOVED == 0u)
        WIFIdev_Channel_backup.cr = WIFIdev_Channel_CONTROL_REG;
    #endif /* End WIFIdev_Channel_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the nonretention control register except FIFO.
*  Does not restore the FIFO which is a set of nonretention registers.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  WIFIdev_Channel_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling WIFIdev_Channel_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void WIFIdev_Channel_RestoreConfig(void)
{
    #if(WIFIdev_Channel_CONTROL_REG_REMOVED == 0u)
        WIFIdev_Channel_CONTROL_REG = WIFIdev_Channel_backup.cr;
    #endif /* End WIFIdev_Channel_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The WIFIdev_Channel_Sleep() API saves the current component state. Then it
*  calls the WIFIdev_Channel_Stop() function and calls 
*  WIFIdev_Channel_SaveConfig() to save the hardware configuration.
*  Call the WIFIdev_Channel_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  WIFIdev_Channel_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void WIFIdev_Channel_Sleep(void)
{
    #if(WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED)
        if((WIFIdev_Channel_RXSTATUS_ACTL_REG  & WIFIdev_Channel_INT_ENABLE) != 0u)
        {
            WIFIdev_Channel_backup.enableState = 1u;
        }
        else
        {
            WIFIdev_Channel_backup.enableState = 0u;
        }
    #else
        if((WIFIdev_Channel_TXSTATUS_ACTL_REG  & WIFIdev_Channel_INT_ENABLE) !=0u)
        {
            WIFIdev_Channel_backup.enableState = 1u;
        }
        else
        {
            WIFIdev_Channel_backup.enableState = 0u;
        }
    #endif /* End WIFIdev_Channel_RX_ENABLED || WIFIdev_Channel_HD_ENABLED*/

    WIFIdev_Channel_Stop();
    WIFIdev_Channel_SaveConfig();
}


/*******************************************************************************
* Function Name: WIFIdev_Channel_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  WIFIdev_Channel_Sleep() was called. The WIFIdev_Channel_Wakeup() function
*  calls the WIFIdev_Channel_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  WIFIdev_Channel_Sleep() function was called, the WIFIdev_Channel_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  WIFIdev_Channel_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void WIFIdev_Channel_Wakeup(void)
{
    WIFIdev_Channel_RestoreConfig();
    #if( (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) )
        WIFIdev_Channel_ClearRxBuffer();
    #endif /* End (WIFIdev_Channel_RX_ENABLED) || (WIFIdev_Channel_HD_ENABLED) */
    #if(WIFIdev_Channel_TX_ENABLED || WIFIdev_Channel_HD_ENABLED)
        WIFIdev_Channel_ClearTxBuffer();
    #endif /* End WIFIdev_Channel_TX_ENABLED || WIFIdev_Channel_HD_ENABLED */

    if(WIFIdev_Channel_backup.enableState != 0u)
    {
        WIFIdev_Channel_Enable();
    }
}


/* [] END OF FILE */
