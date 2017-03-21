/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2017 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP32 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_at.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_system.h"
#include "at_upgrade.h"

/*-------------------------------------------------------------------------*/
#include "bt.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_defs.h"

#if 0
    #ifndef CONFIG_AT_BASE_ON_UART
    #define CONFIG_AT_BASE_ON_UART
    #endif
#endif

#define BUFFER_DEPTH 50
#define ADV_DATA_LEN 31
#define HCI_H4_CMD_PREAMBLE_SIZE           (4)
#define HCI_GRP_BLE_CMDS                   (0x08 << 10)

#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}

#define HCI_BLE_WRITE_ADV_DATA             (0x0008 | HCI_GRP_BLE_CMDS)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA      (31)
#define ARRAY_TO_STREAM(p, a, len) {int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (uint8_t) a[ijk];}

static char ble_device_name[BUFFER_DEPTH] = "";
static uint8_t ble_adv_data[ADV_DATA_LEN] = {};
static uint8_t hci_cmd_buf[128];

static esp_ble_adv_params_t at_adv_params = {
	.adv_int_min       =  0x20 ,
	.adv_int_max       =  0x40 ,
	.adv_type          =  ADV_TYPE_IND ,
	.own_addr_type     =  BLE_ADDR_TYPE_PUBLIC ,
	.channel_map       =  ADV_CHNL_ALL ,
	.adv_filter_policy =  ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY ,
};

static esp_ble_scan_params_t at_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = ESP_PUBLIC_ADDR,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

enum {
    H4_TYPE_COMMAND = 1,
    H4_TYPE_ACL     = 2,
    H4_TYPE_SCO     = 3,
    H4_TYPE_EVENT   = 4
};
/*-------------------------------------------------------------------------*/

#ifdef    CONFIG_AT_BASE_ON_UART
#include "driver/uart.h"
typedef struct {
    int32_t baudrate;
    int8_t data_bits;
    int8_t stop_bits;
    int8_t parity;
    int8_t flow_control;
} at_nvm_uart_config_struct; 

QueueHandle_t esp_at_uart_queue = NULL;
static bool at_save_para_into_flash = false;


static bool at_nvm_uart_config_set (at_nvm_uart_config_struct *uart_config);
static bool at_nvm_uart_config_get (at_nvm_uart_config_struct *uart_config);


int32_t at_port_write_data(uint8_t*data,int32_t len)
{
    uint32_t length = 0;

    length = uart_write_bytes(CONFIG_AT_UART_PORT,(char*)data,len);
    return length;
}

int32_t at_port_read_data(uint8_t*buf,int32_t len)
{
    TickType_t ticks_to_wait = portTICK_RATE_MS;
    uint8_t *data = NULL;
    size_t size = 0;

    if (len == 0) {
        return 0;
    }

    if (buf == NULL) {
        if (len == -1) {
            if (ESP_OK != uart_get_buffered_data_len(CONFIG_AT_UART_PORT,&size)) {
                return -1;
            }
            len = size;
        }

        if (len == 0) {
            return 0;
        }

        data = (uint8_t *)malloc(len);
        if (data) {
            len = uart_read_bytes(CONFIG_AT_UART_PORT,data,len,ticks_to_wait);
            free(data);
            return len;
        } else {
            return -1;
        }
    } else {
        return uart_read_bytes(CONFIG_AT_UART_PORT,buf,len,ticks_to_wait);
    }
}

int32_t at_port_get_data_length (void)
{
    size_t size = 0;
    if (ESP_OK == uart_get_buffered_data_len(CONFIG_AT_UART_PORT,&size)) {
        return size;
    } else {
        return 0;
    }
}

bool at_port_wait_write_complete (int32_t timeout_msec)
{
    if (ESP_OK == uart_wait_tx_done(CONFIG_AT_UART_PORT, timeout_msec*portTICK_PERIOD_MS)) {
        return true;
    }

    return false;
}

void uart_task(void *pvParameters)
{
    uart_event_t event;
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(esp_at_uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch (event.type) {
            //Event of UART receving data
            case UART_DATA:
                if (event.size) {
                    esp_at_port_recv_data_notify (event.size, portMAX_DELAY);
                }
                break;
            case UART_PATTERN_DET:
                esp_at_transmit_terminal();
                break;
            //Others
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}


static void at_uart_init(void)
{
    at_nvm_uart_config_struct uart_nvm_config;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    memset(&uart_nvm_config,0x0,sizeof(uart_nvm_config));
    
    if (at_nvm_uart_config_get(&uart_nvm_config)) {
        if ((uart_nvm_config.baudrate >= 80) && (uart_nvm_config.baudrate <= 5000000)) {
            uart_config.baud_rate = uart_nvm_config.baudrate;
        }

        if ((uart_nvm_config.data_bits >= UART_DATA_5_BITS) && (uart_nvm_config.data_bits <= UART_DATA_8_BITS)) {
            uart_config.data_bits = uart_nvm_config.data_bits;
        }

        if ((uart_nvm_config.stop_bits >= UART_STOP_BITS_1) && (uart_nvm_config.stop_bits <= UART_STOP_BITS_2)) {
            uart_config.stop_bits = uart_nvm_config.stop_bits;
        }

        if ((uart_nvm_config.parity == UART_PARITY_DISABLE) 
            || (uart_nvm_config.parity == UART_PARITY_ODD)
            || (uart_nvm_config.parity == UART_PARITY_EVEN)) {
            uart_config.parity = uart_nvm_config.parity;
        }

        if ((uart_nvm_config.flow_control >= UART_HW_FLOWCTRL_DISABLE) && (uart_nvm_config.flow_control <= UART_HW_FLOWCTRL_CTS_RTS)) {
            uart_config.flow_ctrl = uart_nvm_config.flow_control;
        }
    }
    uart_wait_tx_done(CONFIG_AT_UART_PORT,1000*portTICK_PERIOD_MS);
    //Set UART parameters
    uart_param_config(CONFIG_AT_UART_PORT, &uart_config);
    //Set UART pins,(-1: default pin, no change.)
    uart_set_pin(CONFIG_AT_UART_PORT, CONFIG_AT_UART_PORT_TX_PIN, CONFIG_AT_UART_PORT_RX_PIN, CONFIG_AT_UART_PORT_RTS_PIN, CONFIG_AT_UART_PORT_CTS_PIN);
    //Install UART driver, and get the queue.
    uart_driver_install(CONFIG_AT_UART_PORT, 2048, 8192, 10,&esp_at_uart_queue,0);
    xTaskCreate(uart_task, "uTask", 2048, (void*)CONFIG_AT_UART_PORT, 8, NULL);
}

static bool at_nvm_uart_config_set (at_nvm_uart_config_struct *uart_config)
{
    nvs_handle handle;
    if (uart_config == NULL) {
        return false;
    }

    if (nvs_open("UART", NVS_READWRITE, &handle) == ESP_OK) {
        if (nvs_set_i32(handle, "rate", uart_config->baudrate) != ESP_OK) {
            nvs_close(handle);
            return false;
        }
        if (nvs_set_i8(handle, "databits", uart_config->data_bits) != ESP_OK) {
            nvs_close(handle);
            return false;
        }
        if (nvs_set_i8(handle, "stopbits", uart_config->stop_bits) != ESP_OK) {
            nvs_close(handle);
            return false;
        }
        if (nvs_set_i8(handle, "parity", uart_config->parity) != ESP_OK) {
            nvs_close(handle);
            return false;
        }
        if (nvs_set_i8(handle, "flow_ctrl", uart_config->flow_control) != ESP_OK) {
            nvs_close(handle);
            return false;
        }
    } else {
        return false;
    }
    nvs_close(handle);

    return true;
}

static bool at_nvm_uart_config_get (at_nvm_uart_config_struct *uart_config)
{
    nvs_handle handle;
    if (uart_config == NULL) {
        return false;
    }

    if (nvs_open("UART", NVS_READONLY, &handle) == ESP_OK) {
        if (nvs_get_i32(handle, "rate", &uart_config->baudrate) != ESP_OK) {
            nvs_close(handle);
            return false;
        }
        if (nvs_get_i8(handle, "databits", &uart_config->data_bits) != ESP_OK) {
            nvs_close(handle);
            return false;
        }
        if (nvs_get_i8(handle, "stopbits", &uart_config->stop_bits) != ESP_OK) {
            nvs_close(handle);
            return false;
        }
        if (nvs_get_i8(handle, "parity", &uart_config->parity) != ESP_OK) {
            nvs_close(handle);
            return false;
        }
        if (nvs_get_i8(handle, "flow_ctrl", &uart_config->flow_control) != ESP_OK) {
            nvs_close(handle);
            return false;
        }
    } else {
        return false;
    }
    nvs_close(handle);

    return true;
}


static uint8_t at_setupCmdUart(uint8_t para_num)
{
    int32_t value = 0;
    int32_t cnt = 0;

    at_nvm_uart_config_struct uart_config;

    memset(&uart_config,0x0,sizeof(uart_config));
    if (para_num != 5) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    if (esp_at_get_para_as_digit (cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if ((value < 80) || (value > 5000000)) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    uart_config.baudrate = value;

    if (esp_at_get_para_as_digit (cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if ((value < 5) || (value > 8)) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    uart_config.data_bits = value - 5;

    if (esp_at_get_para_as_digit (cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if ((value < 1) || (value > 3)) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    uart_config.stop_bits = value;

    if (esp_at_get_para_as_digit (cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if (value == 0) {
        uart_config.parity = UART_PARITY_DISABLE;
    } else if (value == 1) {
        uart_config.parity = UART_PARITY_ODD;
    } else if (value == 2) {
        uart_config.parity = UART_PARITY_EVEN;
    } else {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    uart_config.parity = value;

    if (esp_at_get_para_as_digit (cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if ((value < 0) || (value > 3)) {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    uart_config.flow_control = value;

    if (at_save_para_into_flash) {
        if (at_nvm_uart_config_set(&uart_config) == false) {
            return ESP_AT_RESULT_CODE_ERROR;
        }
    }
    esp_at_response_result(ESP_AT_RESULT_CODE_OK);

    uart_wait_tx_done(CONFIG_AT_UART_PORT,portMAX_DELAY);
    uart_set_baudrate(CONFIG_AT_UART_PORT,uart_config.baudrate);
    uart_set_word_length(CONFIG_AT_UART_PORT,uart_config.data_bits);
    uart_set_stop_bits(CONFIG_AT_UART_PORT,uart_config.stop_bits);
    uart_set_parity(CONFIG_AT_UART_PORT,uart_config.parity);
    uart_set_hw_flow_ctrl(CONFIG_AT_UART_PORT,uart_config.flow_control,120);

    return ESP_AT_RESULT_CODE_PROCESS_DONE;
}

static uint8_t at_setupCmdUartDef(uint8_t para_num)
{
    uint8_t ret = ESP_AT_RESULT_CODE_ERROR;
    at_save_para_into_flash = true;
    ret = at_setupCmdUart(para_num);
    at_save_para_into_flash = false;
    
    return ret;
}

static uint8_t at_exeCmdCipupdate(uint8_t *cmd_name)//add get station ip and ap ip
{

    if (esp_at_upgrade_process()) {
        esp_at_response_result(ESP_AT_RESULT_CODE_OK);
        esp_at_port_wait_write_complete(portMAX_DELAY);
        esp_restart();
        for(;;){
        }
    }

    return ESP_AT_RESULT_CODE_ERROR;
}
/*--------------------------------------------------------------------------------------------------------------------------------*/
/*
static uint16_t make_cmd_ble_set_adv_enable (uint8_t *buf, uint8_t adv_enable)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM (buf, adv_enable);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_WRITE_ADV_ENABLE;
}

static uint16_t make_cmd_ble_set_adv_param (uint8_t *buf, uint16_t adv_int_min, uint16_t adv_int_max,
        uint8_t adv_type, uint8_t addr_type_own,
        uint8_t addr_type_dir, bd_addr_t direct_bda,
        uint8_t channel_map, uint8_t adv_filter_policy)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_PARAMS);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS );

    UINT16_TO_STREAM (buf, adv_int_min);
    UINT16_TO_STREAM (buf, adv_int_max);
    UINT8_TO_STREAM (buf, adv_type);
    UINT8_TO_STREAM (buf, addr_type_own);
    UINT8_TO_STREAM (buf, addr_type_dir);
    BDADDR_TO_STREAM (buf, direct_bda);
    UINT8_TO_STREAM (buf, channel_map);
    UINT8_TO_STREAM (buf, adv_filter_policy);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS;
}
*/
static uint16_t make_cmd_ble_set_adv_data(uint8_t *buf, uint8_t data_len, uint8_t *p_data)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_DATA);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1);

    memset(buf, 0, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA);

    if (p_data != NULL && data_len > 0) {
        if (data_len > HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA) {
            data_len = HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA;
        }

        UINT8_TO_STREAM (buf, data_len);

        ARRAY_TO_STREAM (buf, p_data, data_len);
    }
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1;
}

static uint8_t at_queryCmdBleAddr(uint8_t * cmd_name)
{
	char s[200];
	const uint8_t * pAddr;
	pAddr = esp_bt_dev_get_address();
	if(!pAddr){
		return ESP_AT_RESULT_CODE_FAIL;
	}
    sprintf(s,"BLE public address : %02x:%02x:%02x:%02x:%02x:%02x\r\n",ESP_BD_ADDR_HEX(pAddr));
    esp_at_port_write_data((uint8_t *)s , strlen(s));

    return ESP_AT_RESULT_CODE_OK;
}
static uint8_t at_setupCmdBleAddr(uint8_t  para_num)
{
	int32_t cnt = 0 , value = 0;
	esp_bd_addr_t rand_addr;
	
	if(para_num != 6){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	for(cnt=0;cnt<6;cnt++){
		if(esp_at_get_para_as_digit(cnt,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
			return ESP_AT_RESULT_CODE_ERROR;
		}
		if((value < 0)||(value > 0xff)){
			return ESP_AT_RESULT_CODE_ERROR;
		}
		rand_addr[cnt] = (uint8_t)value;
	}
	if(esp_ble_gap_set_rand_addr(rand_addr) != ESP_OK)
		return ESP_AT_RESULT_CODE_ERROR;
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_queryCmdBleName(uint8_t * cmd_name)
{
	if(strlen(ble_device_name)== 0){
		esp_at_port_write_data((uint8_t *)"Device Name is NULL\r\n",strlen("Device Name is NULL\r\n"));
		return ESP_AT_RESULT_CODE_OK;
	}
	esp_at_port_write_data((uint8_t *)ble_device_name,strlen(ble_device_name));
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_setupCmdBleName(uint8_t para_num)
{
	int32_t cnt = 0;

	if(para_num != 1){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	if(esp_at_get_para_as_str(cnt,(uint8_t **)(&ble_device_name)) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	if(esp_ble_gap_set_device_name(ble_device_name) != ESP_OK )
		return ESP_AT_RESULT_CODE_ERROR;
	
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_exeCmdBleInit(uint8_t *cmd_name)
{
	esp_bt_controller_init();
	if(esp_bt_controller_enable(ESP_BT_MODE_BTDM) == ESP_OK)
		return ESP_AT_RESULT_CODE_FAIL;
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_queryCmdBleAdvParam(uint8_t *cmd_name)
{
	char s[BUFFER_DEPTH];
	
	memset(s,'\0',BUFFER_DEPTH);
    sprintf(s,"adv int min : %02x\r\n",at_adv_params.adv_int_min);
    esp_at_port_write_data((uint8_t *)s , strlen(s));
		
	memset(s,'\0',BUFFER_DEPTH);
    sprintf(s,"adv int max : %02x\r\n",at_adv_params.adv_int_max);
    esp_at_port_write_data((uint8_t *)s , strlen(s));	
	
	memset(s,'\0',BUFFER_DEPTH);
    sprintf(s,"adv type : %02x\r\n",at_adv_params.adv_type);
    esp_at_port_write_data((uint8_t *)s , strlen(s));	
	
	memset(s,'\0',BUFFER_DEPTH);
    sprintf(s,"own addr type : %02x\r\n",at_adv_params.own_addr_type);
    esp_at_port_write_data((uint8_t *)s , strlen(s));	
	
	memset(s,'\0',BUFFER_DEPTH);
    sprintf(s,"channel map : %02x\r\n",at_adv_params.channel_map);
    esp_at_port_write_data((uint8_t *)s , strlen(s));	
	
	memset(s,'\0',BUFFER_DEPTH);
    sprintf(s,"adv filter policy : %02x\r\n",at_adv_params.adv_filter_policy);
    esp_at_port_write_data((uint8_t *)s , strlen(s));	
	
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_setupCmdBleAdvParam(uint8_t para_num)
{
	int32_t cnt = 0 , value = 0;
	
	if(para_num != 6){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	
	if(esp_at_get_para_as_digit(cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	at_adv_params.adv_int_min = value;
	if(esp_at_get_para_as_digit(cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	at_adv_params.adv_int_max = value;
	if(esp_at_get_para_as_digit(cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	at_adv_params.adv_type = value;
	if(esp_at_get_para_as_digit(cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	at_adv_params.own_addr_type = value;
	if(esp_at_get_para_as_digit(cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	at_adv_params.channel_map = value;
	if(esp_at_get_para_as_digit(cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	at_adv_params.adv_filter_policy = value;
	
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_setupCmdBleAdvData(uint8_t para_num)
{
	int32_t cnt = 0 ,i = 0 ,j = 0;
	char s[ADV_DATA_LEN*2];
	
	if(para_num != 1){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	
	if(esp_at_get_para_as_str(cnt,(uint8_t **)(&s)) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	while((s[i]!='\0')&&(s[i+1]!='\0'))
	{
		ble_adv_data[j] = ((s[i]-'0')<<4) + (s[i+1]-'0');
		i += 2; 
		j++;
	}
	
	uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf,j,ble_adv_data);
	esp_vhci_host_send_packet(hci_cmd_buf,sz);
	
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_exeCmdBleAdvStart(uint8_t *cmd_name)
{
	if(esp_ble_gap_start_advertising(&at_adv_params) != ESP_OK)
		return ESP_AT_RESULT_CODE_FAIL;
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_exeCmdBleAdvStop(uint8_t *cmd_name)
{
	if(esp_ble_gap_stop_advertising() != ESP_OK)
		return ESP_AT_RESULT_CODE_FAIL;
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_setupCmdBLeSacnParam(uint8_t para_num)
{
	int32_t cnt = 0 , value = 0;
	
	if(para_num != 5){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	
	if(esp_at_get_para_as_digit(cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	at_scan_params.scan_type = value;
	if(esp_at_get_para_as_digit(cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	at_scan_params.own_addr_type = value;
	if(esp_at_get_para_as_digit(cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	at_scan_params.scan_filter_policy = value;
	if(esp_at_get_para_as_digit(cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	at_scan_params.scan_interval = value;
	if(esp_at_get_para_as_digit(cnt++,&value) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	at_scan_params.scan_window = value;
	
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_setupCmdBleScan(uint8_t para_num)
{
	int32_t cnt = 0 ;
	char s[ADV_DATA_LEN] = "";
	
	if(para_num != 1){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	if(esp_at_get_para_as_str(cnt,(uint8_t **)(&s)) != ESP_AT_PARA_PARSE_RESULT_OK){
		return ESP_AT_RESULT_CODE_ERROR;
	}
	if(strcmp(s,"true") == 0){
		esp_ble_gap_start_scanning(0xffff);
	}
	else if(strcmp(s,"false") == 0){
		esp_ble_gap_stop_scanning();
	}
	else
		return ESP_AT_RESULT_CODE_FAIL;
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_queryCmdBleConnParam(uint8_t *cmd_name)
{
	//TODO:	
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_setupCmdBleConnParam(uint8_t param)
{
	//TODO:
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_setupCmdBleConn(uint8_t param)
{
	//TODO:
	return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_exeCmdBleDisConn(uint8_t *cmd_name)
{
	esp_gatt_if_t gattc_if = 0x0;
	uint16_t conn_id = 0x0;
	
	//TODO:gattc_if & conn_id
	if(esp_ble_gattc_close(gattc_if,conn_id) != ESP_OK)
		return ESP_AT_RESULT_CODE_FAIL;
	
	return ESP_AT_RESULT_CODE_OK;
}
/*--------------------------------------------------------------------------------------------------------------------------------*/
static esp_at_cmd_struct at_custom_cmd[] = {
    {"+UART", NULL, NULL, at_setupCmdUart, NULL},
    {"+UART_CUR", NULL, NULL, at_setupCmdUart, NULL},
    {"+UART_DEF", NULL, NULL, at_setupCmdUartDef, NULL},
    {"+CIUPDATE", NULL, NULL, NULL, at_exeCmdCipupdate},

    /*add for ble at*/
    {"+BLEADDR",NULL,at_queryCmdBleAddr,at_setupCmdBleAddr,NULL},
    {"+BLEANAME",NULL,at_queryCmdBleName,at_setupCmdBleName,NULL},
    {"+BLEINIT",NULL,NULL,NULL,at_exeCmdBleInit},
	
	{"+BLEADVPARAM",NULL,at_queryCmdBleAdvParam,at_setupCmdBleAdvParam,NULL},
    {"+BLEADVDATA",NULL,NULL,at_setupCmdBleAdvData,NULL},
	{"+BLEADVSTART",NULL,NULL,NULL,at_exeCmdBleAdvStart},
	{"+BLEADVSTOP",NULL,NULL,NULL,at_exeCmdBleAdvStop},
	
	{"+BLESCANOARAM",NULL,NULL,at_setupCmdBLeSacnParam,NULL},
	{"+BLESCAN",NULL,NULL,at_setupCmdBleScan,NULL},
	{"+BLECONNPARAM",NULL,at_queryCmdBleConnParam,at_setupCmdBleConnParam,NULL},
	{"+BLECONN",NULL,NULL,at_setupCmdBleConn,NULL},
	{"+BLEDISCONN",NULL,NULL,NULL,at_exeCmdBleDisConn},
};

void at_status_callback (esp_at_status_type status)
{
    switch (status) {
    case ESP_AT_STATUS_NORMAL:
        uart_disable_pattern_det_intr(CONFIG_AT_UART_PORT);
        break;
    case ESP_AT_STATUS_TRANSMIT:
        uart_enable_pattern_det_intr(CONFIG_AT_UART_PORT, '+', 3, ((APB_CLK_FREQ*20)/1000),((APB_CLK_FREQ*20)/1000), ((APB_CLK_FREQ*20)/1000));
        break;
    }
}

void at_task_init(void)
{
    uint8_t *version = (uint8_t *)malloc(64);
    esp_at_device_ops_struct esp_at_device_ops = {
        .read_data = at_port_read_data,
        .write_data = at_port_write_data,
        .get_data_length = at_port_get_data_length,
        .wait_write_complete = at_port_wait_write_complete,
    };
    
    esp_at_custom_ops_struct esp_at_custom_ops = {
        .status_callback = at_status_callback,
    };

    at_uart_init();

    sprintf((char*)version, "compile time:%s %s", __DATE__, __TIME__);
    esp_at_device_ops_regist (&esp_at_device_ops);
    esp_at_custom_ops_regist(&esp_at_custom_ops);
    esp_at_module_init (0, version);
    free(version);

    esp_at_custom_cmd_array_regist (at_custom_cmd, sizeof(at_custom_cmd)/sizeof(at_custom_cmd[0]));
    esp_at_port_write_data((uint8_t *)"\r\nready\r\n",strlen("\r\nready\r\n"));

}

#endif


