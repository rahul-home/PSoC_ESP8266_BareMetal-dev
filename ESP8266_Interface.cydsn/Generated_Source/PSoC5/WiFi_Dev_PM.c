/*******************************************************************************
* File Name: WiFi_Dev_PM.c
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

#include "WiFi_Dev.h"


/***************************************
* Local data allocation
***************************************/

static WiFi_Dev_BACKUP_STRUCT  WiFi_Dev_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: WiFi_Dev_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the WiFi_Dev_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  WiFi_Dev_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void WiFi_Dev_SaveConfig(void)
{
    #if(WiFi_Dev_CONTROL_REG_REMOVED == 0u)
        WiFi_Dev_backup.cr = WiFi_Dev_CONTROL_REG;
    #endif /* End WiFi_Dev_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: WiFi_Dev_RestoreConfig
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
*  WiFi_Dev_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling WiFi_Dev_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void WiFi_Dev_RestoreConfig(void)
{
    #if(WiFi_Dev_CONTROL_REG_REMOVED == 0u)
        WiFi_Dev_CONTROL_REG = WiFi_Dev_backup.cr;
    #endif /* End WiFi_Dev_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: WiFi_Dev_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The WiFi_Dev_Sleep() API saves the current component state. Then it
*  calls the WiFi_Dev_Stop() function and calls 
*  WiFi_Dev_SaveConfig() to save the hardware configuration.
*  Call the WiFi_Dev_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  WiFi_Dev_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void WiFi_Dev_Sleep(void)
{
    #if(WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED)
        if((WiFi_Dev_RXSTATUS_ACTL_REG  & WiFi_Dev_INT_ENABLE) != 0u)
        {
            WiFi_Dev_backup.enableState = 1u;
        }
        else
        {
            WiFi_Dev_backup.enableState = 0u;
        }
    #else
        if((WiFi_Dev_TXSTATUS_ACTL_REG  & WiFi_Dev_INT_ENABLE) !=0u)
        {
            WiFi_Dev_backup.enableState = 1u;
        }
        else
        {
            WiFi_Dev_backup.enableState = 0u;
        }
    #endif /* End WiFi_Dev_RX_ENABLED || WiFi_Dev_HD_ENABLED*/

    WiFi_Dev_Stop();
    WiFi_Dev_SaveConfig();
}


/*******************************************************************************
* Function Name: WiFi_Dev_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  WiFi_Dev_Sleep() was called. The WiFi_Dev_Wakeup() function
*  calls the WiFi_Dev_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  WiFi_Dev_Sleep() function was called, the WiFi_Dev_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  WiFi_Dev_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void WiFi_Dev_Wakeup(void)
{
    WiFi_Dev_RestoreConfig();
    #if( (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) )
        WiFi_Dev_ClearRxBuffer();
    #endif /* End (WiFi_Dev_RX_ENABLED) || (WiFi_Dev_HD_ENABLED) */
    #if(WiFi_Dev_TX_ENABLED || WiFi_Dev_HD_ENABLED)
        WiFi_Dev_ClearTxBuffer();
    #endif /* End WiFi_Dev_TX_ENABLED || WiFi_Dev_HD_ENABLED */

    if(WiFi_Dev_backup.enableState != 0u)
    {
        WiFi_Dev_Enable();
    }
}


/* [] END OF FILE */
