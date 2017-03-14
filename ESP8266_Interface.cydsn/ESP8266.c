/***
	Copyright 2017,  Rahul Raj Rajan (rahulraj.rajan@hotmail.com)
	See no evil, Hear no evil, Speak no evil!!
***/ 
    
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ESP8266_Config.h"
#include "ESP8266.h"
#include "project.h"

#define RESPONSE_BUF_LEN (1024)
#define REQUEST_BUF_LEN (1024)

/******************************************************************************************************/
/**                                         GLOBAL DATA                                          **/
/******************************************************************************************************/

/** 
Rx Circular Buffer Handle.
Used by RxISR to fill data. 
**/

circular_buffer_t rx_buffer;

/**
Memory for Requests!
**/

static char cRequest[REQUEST_BUF_LEN]    = {0};

/******************************************************************************************************/
/**                                         LOCAL FUNCTIONS                                          **/
/******************************************************************************************************/

static int32_t ESP8266_Request(char *request);

static int32_t ESP8266_SendData(uint8_t *buffer, uint16_t length);

static int32_t ESP8266_Await_Response(char *positive_response, char *negative_response);

static int32_t ESP8266_Await_Response_Not(uint8_t *buffer, uint16_t *length, char *negative_response);

static int32_t ESP8266_Read_Response(uint8_t *buffer, uint16_t *length);

static int32_t ESP8266_Check_Response(uint8_t *buffer, uint16_t length, char *expected_resp);



int32_t ESP8266_Init()
{
    rx_buffer.mnWR      = 0;
    rx_buffer.mnRD      = 0;
    rx_buffer.mnCount   = 0;
    
    WIFIdev_Channel_Start();
        
    WIFIdev_Channel_ClearRxBuffer();
    WIFIdev_Channel_ClearTxBuffer();
    
    WIFIdev_Channel_EnableRxInt();
    
    WIFIdev_Channel_RxISR_Start();
    WIFIdev_Channel_RxISR_Enable();
       
    /** Sending Attention Command **/
    LOG_PutString("\nCMD - AT...");
    
    strcpy(cRequest, REQUEST_AT);
    strcat(cRequest, ESP8266_EOP);
    
    ESP8266_Request(cRequest);
    if (0 > ESP8266_Await_Response(RESPONSE_OK, RESPONSE_ERROR))
    {
        LOG_PutString("\nCMD - AT - FAILED");
        return -1;
    }

    /** Setting Mode - STA **/ 
    LOG_PutString("\nCMD - MODE...");
    
    strcpy(cRequest, REQUEST_CWMODE_SET);
    strcat(cRequest, REQUEST_MODE_STA);
    strcat(cRequest, ESP8266_EOP);
    
    ESP8266_Request(cRequest);
    if (0 > ESP8266_Await_Response(RESPONSE_OK, RESPONSE_ERROR))
    {
        LOG_PutString("\nCMD - MODE - FAILED");
        return -1;
    }
       
    /** Setting DHCP - disable **/ 
    LOG_PutString("\nCMD - DHCP...");
    
    strcpy(cRequest, REQUEST_CWDHCP_SET);
    strcat(cRequest, REQUEST_MODE_STA);
    strcat(cRequest, REQUEST_FIELD_SEPARATOR);
    strcat(cRequest, REQUEST_DHCP_DISABLE);
    strcat(cRequest, ESP8266_EOP);
    
    ESP8266_Request(cRequest);
    if (0 > ESP8266_Await_Response(RESPONSE_OK, RESPONSE_ERROR))
    {
        LOG_PutString("\nCMD - DHCP : FAILED");
        return -1;
    }
    
    /** Joining AP **/ 
    LOG_PutString("\nCMD - JAP...");
    
    strcpy(cRequest, REQUEST_CWJAP_SET);
    strcat(cRequest, AP_SSID);
    strcat(cRequest, REQUEST_FIELD_SEPARATOR);
    strcat(cRequest, AP_PASSWD);
    strcat(cRequest, ESP8266_EOP);
    
    ESP8266_Request(cRequest);
        
    if (0 > ESP8266_Await_Response(RESPONSE_AP_CONNECTED, RESPONSE_ERROR))
    {
        LOG_PutString("\nCMD - JAP : FAILED");
        return -1;
    }
    
    LOG_PutString("\nCMD - JAP : Connected to AP");
    
    if (0 > ESP8266_Await_Response(RESPONSE_IP_ACQUIRED, RESPONSE_AP_DISCONNECTED))
    {
        LOG_PutString("\nCMD - JAP : FAILED");
        return -1;
    }
    
    LOG_PutString("\nCMD - JAP : Acquired IP");
    
    if (0 > ESP8266_Await_Response(RESPONSE_OK, RESPONSE_AP_DISCONNECTED))
    {
        LOG_PutString("\nCMD - JAP : FAILED");
        return -1;
    }
    
    LOG_PutString("\nCMD - JAP : Connection Stable");
 
    /** Creating Connection - SINGLE CHANNEL(0) mode
        If MULTI_CHANNEL required, need to set CIPMUX to MULTI_CHANNEL(1) mode.
    **/ 
    LOG_PutString("\nCMD - IPSTART...");
    
    strcpy(cRequest, REQUEST_CIPSTART_SET);
    strcat(cRequest, TRANSPORT_TYPE);
    strcat(cRequest, REQUEST_FIELD_SEPARATOR);
    strcat(cRequest, SERVER_ADDR);
    strcat(cRequest, REQUEST_FIELD_SEPARATOR);
    strcat(cRequest, SERVER_PORT);
    strcat(cRequest, ESP8266_EOP);
    
    ESP8266_Request(cRequest);
    if (0 > ESP8266_Await_Response(RESPONSE_TCP_CONNECTED, RESPONSE_ERROR))
    {
        /**
         If it fails here,
         check if there is a $(TRANSPORT_TYPE) server running at $(SERVER_ADDR):$(SERVER_PORT)
        **/
        LOG_PutString("\nCMD - IPSTART : FAILED");
        return -1;
    }
    
    if (0 > ESP8266_Await_Response(RESPONSE_OK, RESPONSE_TCP_CLOSED))
    {
        LOG_PutString("\nCMD - IPSTART : FAILED");
        return -1;
    }
    
    return 0;
}

int32_t ESP8266_Send(uint8_t *buffer, uint16_t length)
{
    sprintf(cRequest, "%s%d%s", REQUEST_CIPSEND, length, ESP8266_EOP);
    
    ESP8266_Request(cRequest);
    if (0 > ESP8266_Await_Response(RESPONSE_OK, RESPONSE_ERROR))
    {
        LOG_PutString("\nCMD - SEND : FAILED");
        return -1;
    }
    
    ESP8266_SendData(buffer, length);
    if (0 > ESP8266_Await_Response(RESPONSE_SEND_OK, RESPONSE_SEND_ERROR))
    {
        LOG_PutString("\nCMD - SEND : FAILED");
        return -1;
    }
   
    return length;
}

static int32_t ESP8266_Request(char *request)
{
    WIFIdev_Channel_PutString(request);
    
    return 0;
}

static int32_t ESP8266_SendData(uint8_t *buffer, uint16_t length)
{
    WIFIdev_Channel_PutArray(buffer, length);
    
    return 0;
}

static int32_t  ESP8266_Await_Response(char *positive_response, char *negative_response)
{
    uint16_t len = 0;
    uint8_t response_buffer[RESPONSE_BUF_LEN]    = {0};
    
    do
    {
        len = RESPONSE_BUF_LEN;
        
        if (0 != ESP8266_Read_Response(response_buffer, &len))
        {
            LOG_PutString("\n[WIFIdev] Read Error");
            break;
        }
        else
        {          
            if (1 == ESP8266_Check_Response(response_buffer, len, positive_response))
            {
                //LOG_PutString("\n[WIFIdev] Recvd : Positive Response");
                return 0;
            }
            else if (1 == ESP8266_Check_Response(response_buffer, len, negative_response))
            {
                LOG_PutString("\n[WIFIdev] Recvd :  Negative Response");
                return -1;
            }
        }
    } while (1 == 1);
    
    return 0;
}

static int32_t ESP8266_Await_Response_Not(uint8_t *buffer, uint16_t *length, char *negative_response)
{
     uint16_t len = 0, i = 0;
    
    do
    {
        len = *length;
        
        if (0 != ESP8266_Read_Response(buffer, &len))
        {
            LOG_PutString("\n[WIFIdev] Read Error");
            break;
        }
        else
        {
            if (0 == ESP8266_Check_Response(buffer, len, negative_response))
            {
                break;
            }
        }
    } while (1 == 1);

    *length = len;
    return 0;
}

static int32_t ESP8266_Read_Response(uint8_t *buffer, uint16_t *length)
{
    const uint8_t EOP_found = 0x01; //no start of protocol
    
    uint16_t bytes_read = 0;
    uint8_t status  = 0x00;
    
    do
    {
        /** if length of the provided buffer is less,
            - Handling of this error needs to be done at the upper layer!
        **/ 
        if (*length == bytes_read)
            return -3;

#if 0
        /** Sequential Read! No ISR - possible with huge Rx Buffers **/
        while (WIFIdev_Channel_RX_STS_FIFO_NOTEMPTY != WIFIdev_Channel_GetRxInterruptSource());
        buffer[bytes_read++] = WIFIdev_Channel_GetByte();
#else
        /** Read from Circular buffer filled by ISR **/
        while (rx_buffer.mnCount <= 0);
        buffer[bytes_read++] = rx_buffer.mpData[rx_buffer.mnRD];
        rx_buffer.mnRD = ((rx_buffer.mnRD + 1) > CIRCULAR_BUF_LEN) ? 0 : (rx_buffer.mnRD + 1);
        rx_buffer.mnCount--;
#endif
    
        /** Checking for End of Protocol! **/
        if ((bytes_read >= ESP8266_EOP_LEN) && ((status & EOP_found) == 0x00))
        {
            if (0 == strncmp((const char *)&buffer[bytes_read - ESP8266_EOP_LEN], ESP8266_EOP, ESP8266_EOP_LEN))
            {
               status |= EOP_found ;
            }
        }
        
    } while (EOP_found != (status & EOP_found));
    
    *length = bytes_read;
    return 0;
}

static int32_t ESP8266_Check_Response(uint8_t *buffer, uint16_t length, char *expected_resp)
{
    uint16_t  i = 0;
    uint32_t  check_len = 0;
    
    if (NULL == expected_resp)
        return -1;
    
    check_len = strlen(expected_resp);

    if (length < check_len)
        return 0;
    
    /** Check if the response is expected response or not **/
    for (i = 0; i <= (length - check_len); i++)
    {
        if (0 == strncmp((const char*)buffer + i, expected_resp, check_len))
            return 1;
    }

    return 0;
}

/* [] END OF FILE */
