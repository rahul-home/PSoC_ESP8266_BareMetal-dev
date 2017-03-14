/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "ESP8266.h"

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    uint16_t inc_counter = 0x0000,
             dec_counter = 0xFFFF;
    
    uint8_t buffer[10] = {0};
    
    
    LOG_Start();
    
    CyDelay(3000);
    
     LOG_ClearTxBuffer();
    
    LOG_PutString("\nInitializing ESP8266...");
    
    if (0 > ESP8266_Init())
    {
        LOG_PutString("\nESP8266 Initialization Failed!");
        return -1;
    }
    
    LOG_PutString("\nESP8266 Initialized!");
    
    for (;;)
    {
        /* Place your application code here. */
            CyDelay(1000);
            
            inc_counter++;
            dec_counter--;
        
            //Network order - Big Endian
            LOG_PutString("\nSending...!");
            
            buffer[0] = 'i';
            buffer[1] = ((inc_counter >> 8) & 0xFF);
            buffer[2] = (inc_counter & 0xFF);
            
            buffer[3] = 'd';
            buffer[4] = ((dec_counter >> 8) & 0xFF);
            buffer[5] = (dec_counter & 0xFF);

            ESP8266_Send(buffer, 6);
            
            LOG_PutString("\nSent!");
    
    }
}

/* [] END OF FILE */
