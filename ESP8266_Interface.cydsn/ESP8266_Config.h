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

#ifndef _ESP8266_CONFIG_H_
#define _ESP8266_CONFIG_H_

    /*********************** CONNECTION PARAMS *****************/
    
    #define AP_SSID             "\"AP_SSID\""
    #define AP_PASSWD           "\"AP_PASSWORD\""
    
    #define SERVER_ADDR         "\"192.168.100.5\""
    #define SERVER_PORT         "1024"
    
    #define TRANSPORT_TYPE      "\"TCP\""
    
    /***********************************************************/
    
    #define ESP8266_EOP                 "\r\n"
    #define ESP8266_LOOPBACK_EOP        "\r\r\n"
    #define ESP8266_EOP_LEN             (2)
    #define ESP8266_LOOPBACK_EOP_LEN    (3)
    
    #define REQUEST_FIELD_SEPARATOR     ","
    
    #define REQUEST_SEND_DATA_MARKER    ">"
    
    /*********************** CONFIG REQUESTS *****************/
      
    #define REQUEST_AT              "AT"
    #define REQUEST_RST             "AT+RST"
    #define REQUEST_CWMODE_SET      "AT+CWMODE="
    #define REQUEST_CWMODE_GET      "AT+CWMODE?"
    #define REQUEST_CWDHCP_SET      "AT+CWDHCP="
    #define REQUEST_CWJAP_SET       "AT+CWJAP="
    #define REQUEST_IPADDDR_GET     "AT+CIFSR"
    #define REQUEST_CIPMUX_SET      "AT+CIPMUX="
    #define REQUEST_CIPMUX_GET      "AT+CIPMUX?"
    #define REQUEST_CIPSTART_SET    "AT+CIPSTART="
    #define REQUEST_CIPSEND         "AT+CIPSEND="
    
    /*********************************************************/
    
    /*********************** RESPONSES ***********************/
    #define RESPONSE_OK                 "OK"
    #define RESPONSE_ERROR              "ERROR"
    #define RESPONSE_READY              "ready"
    #define RESPONSE_BUSY               "busy p..."
    #define RESPONSE_AP_CONNECTED       "WIFI CONNECTED"
    #define RESPONSE_AP_DISCONNECTED    "WIFI DISCONNECT"
    #define RESPONSE_IP_ACQUIRED        "WIFI GOT IP"
    #define RESPONSE_TCP_CONNECTED      "CONNECT"
    #define RESPONSE_TCP_CLOSED         "CLOSED"
    #define RESPONSE_SEND_OK            "SEND OK"
    #define RESPONSE_SEND_ERROR         "SEND FAIL"
    
    /*********************************************************/    
    
    /************** REQUEST FIELDS ***************/
    
    #define REQUEST_MODE_STA    "1"
    #define REQUEST_MODE_AP     "2"
    
    #define REQUEST_DHCP_ENABLE     "0"
    #define REQUEST_DHCP_DISABLE    "1"
    
    #define REQUEST_SINGLE_CHANNEL  "0"
    #define REQUEST_MULTI_CHANNEL   "1"    
   
#endif

/* [] END OF FILE */
