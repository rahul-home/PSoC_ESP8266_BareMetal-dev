/*******************************************************************************
* File Name: WIFIdev_Channel_RxISR.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_ISR_WIFIdev_Channel_RxISR_H)
#define CY_ISR_WIFIdev_Channel_RxISR_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void WIFIdev_Channel_RxISR_Start(void);
void WIFIdev_Channel_RxISR_StartEx(cyisraddress address);
void WIFIdev_Channel_RxISR_Stop(void);

CY_ISR_PROTO(WIFIdev_Channel_RxISR_Interrupt);

void WIFIdev_Channel_RxISR_SetVector(cyisraddress address);
cyisraddress WIFIdev_Channel_RxISR_GetVector(void);

void WIFIdev_Channel_RxISR_SetPriority(uint8 priority);
uint8 WIFIdev_Channel_RxISR_GetPriority(void);

void WIFIdev_Channel_RxISR_Enable(void);
uint8 WIFIdev_Channel_RxISR_GetState(void);
void WIFIdev_Channel_RxISR_Disable(void);

void WIFIdev_Channel_RxISR_SetPending(void);
void WIFIdev_Channel_RxISR_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the WIFIdev_Channel_RxISR ISR. */
#define WIFIdev_Channel_RxISR_INTC_VECTOR            ((reg32 *) WIFIdev_Channel_RxISR__INTC_VECT)

/* Address of the WIFIdev_Channel_RxISR ISR priority. */
#define WIFIdev_Channel_RxISR_INTC_PRIOR             ((reg8 *) WIFIdev_Channel_RxISR__INTC_PRIOR_REG)

/* Priority of the WIFIdev_Channel_RxISR interrupt. */
#define WIFIdev_Channel_RxISR_INTC_PRIOR_NUMBER      WIFIdev_Channel_RxISR__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable WIFIdev_Channel_RxISR interrupt. */
#define WIFIdev_Channel_RxISR_INTC_SET_EN            ((reg32 *) WIFIdev_Channel_RxISR__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the WIFIdev_Channel_RxISR interrupt. */
#define WIFIdev_Channel_RxISR_INTC_CLR_EN            ((reg32 *) WIFIdev_Channel_RxISR__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the WIFIdev_Channel_RxISR interrupt state to pending. */
#define WIFIdev_Channel_RxISR_INTC_SET_PD            ((reg32 *) WIFIdev_Channel_RxISR__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the WIFIdev_Channel_RxISR interrupt. */
#define WIFIdev_Channel_RxISR_INTC_CLR_PD            ((reg32 *) WIFIdev_Channel_RxISR__INTC_CLR_PD_REG)


#endif /* CY_ISR_WIFIdev_Channel_RxISR_H */


/* [] END OF FILE */
