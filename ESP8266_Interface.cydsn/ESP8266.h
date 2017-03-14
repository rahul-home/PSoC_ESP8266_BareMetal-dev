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

#ifndef _ESP8266_H_
#define _ESP8266_H_
    
    #include <stdint.h>
   
    #define CIRCULAR_BUF_LEN (1024)
    
    typedef struct 
    {
        volatile uint8_t     mpData[CIRCULAR_BUF_LEN];
        volatile uint16_t    mnWR;
        volatile uint16_t    mnRD;
        
        volatile uint16_t    mnCount;
    }circular_buffer_t;
    
    int32_t ESP8266_Init();
    
    int32_t ESP8266_Send(uint8_t *buffer, uint16_t length);
    
#endif

/* [] END OF FILE */
