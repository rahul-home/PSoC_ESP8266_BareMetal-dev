/*******************************************************************************
* File Name: LOG_PM.c
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

#include "LOG.h"


/***************************************
* Local data allocation
***************************************/

static LOG_BACKUP_STRUCT  LOG_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: LOG_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the LOG_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  LOG_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LOG_SaveConfig(void)
{
    #if(LOG_CONTROL_REG_REMOVED == 0u)
        LOG_backup.cr = LOG_CONTROL_REG;
    #endif /* End LOG_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: LOG_RestoreConfig
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
*  LOG_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling LOG_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void LOG_RestoreConfig(void)
{
    #if(LOG_CONTROL_REG_REMOVED == 0u)
        LOG_CONTROL_REG = LOG_backup.cr;
    #endif /* End LOG_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: LOG_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The LOG_Sleep() API saves the current component state. Then it
*  calls the LOG_Stop() function and calls 
*  LOG_SaveConfig() to save the hardware configuration.
*  Call the LOG_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  LOG_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LOG_Sleep(void)
{
    #if(LOG_RX_ENABLED || LOG_HD_ENABLED)
        if((LOG_RXSTATUS_ACTL_REG  & LOG_INT_ENABLE) != 0u)
        {
            LOG_backup.enableState = 1u;
        }
        else
        {
            LOG_backup.enableState = 0u;
        }
    #else
        if((LOG_TXSTATUS_ACTL_REG  & LOG_INT_ENABLE) !=0u)
        {
            LOG_backup.enableState = 1u;
        }
        else
        {
            LOG_backup.enableState = 0u;
        }
    #endif /* End LOG_RX_ENABLED || LOG_HD_ENABLED*/

    LOG_Stop();
    LOG_SaveConfig();
}


/*******************************************************************************
* Function Name: LOG_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  LOG_Sleep() was called. The LOG_Wakeup() function
*  calls the LOG_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  LOG_Sleep() function was called, the LOG_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  LOG_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LOG_Wakeup(void)
{
    LOG_RestoreConfig();
    #if( (LOG_RX_ENABLED) || (LOG_HD_ENABLED) )
        LOG_ClearRxBuffer();
    #endif /* End (LOG_RX_ENABLED) || (LOG_HD_ENABLED) */
    #if(LOG_TX_ENABLED || LOG_HD_ENABLED)
        LOG_ClearTxBuffer();
    #endif /* End LOG_TX_ENABLED || LOG_HD_ENABLED */

    if(LOG_backup.enableState != 0u)
    {
        LOG_Enable();
    }
}


/* [] END OF FILE */
