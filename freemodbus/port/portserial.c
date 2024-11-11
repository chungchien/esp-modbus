/*
 * SPDX-FileCopyrightText: 2010 Christian Walter
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SPDX-FileContributor: 2016-2021 Espressif Systems (Shanghai) CO LTD
 */
/*
 * FreeModbus Libary: ESP32 Port
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: portother.c,v 1.1 2010/06/06 13:07:20 wolti Exp $
 */

#include "driver/uart.h"
#include "port.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/queue.h" // for queue support
#include "driver/gpio.h"
#include "esp_log.h"        // for esp_log
#include "esp_err.h"        // for ESP_ERROR_CHECK macro

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "sdkconfig.h"              // for KConfig options
#include "port_serial_slave.h"
#include "freertos/ringbuf.h"

// Note: This code uses mixed coding standard from legacy IDF code and used freemodbus stack

static TaskHandle_t  xMbTaskHandle;
static const CHAR *TAG = "MB_SERIAL";

// The UART hardware port number
static BOOL bRxStateEnabled = FALSE; // Receiver enabled flag
static BOOL bTxStateEnabled = FALSE; // Transmitter enabled flag

static RingbufHandle_t xRxBuffer;

void vMBPortSerialEnable(BOOL bRxEnable, BOOL bTxEnable)
{
    // This function can be called from xMBRTUTransmitFSM() of different task
    if (bTxEnable) {
        bTxStateEnabled = TRUE;
    } else {
        bTxStateEnabled = FALSE;
    }
    if (bRxEnable) {
        // clear Rx buffer?        
        bRxStateEnabled = TRUE;
        vTaskResume(xMbTaskHandle); // Resume receiver task
    } else {
        vTaskSuspend(xMbTaskHandle); // Block receiver task
        bRxStateEnabled = FALSE;
    }
}

static USHORT usMBPortSerialRxPoll(size_t xEventSize)
{
    BOOL xReadStatus = TRUE;
    USHORT usCnt = 0;

    if (bRxStateEnabled) {
        // Get received packet into Rx buffer
        while(xReadStatus && (usCnt < xEventSize)) {
            // Call the Modbus stack callback function and let it fill the buffers.
            xReadStatus = pxMBFrameCBByteReceived(); // callback to execute receive FSM
            usCnt++;
        }
        // Send event EV_FRAME_RECEIVED to allow stack process packet
#if !CONFIG_FMB_TIMER_PORT_ENABLED
        pxMBPortCBTimerExpired();
#endif
        ESP_LOGD(TAG, "RX: %u bytes\n", (unsigned)usCnt);
    }
    return usCnt;
}

BOOL xMBPortSerialTxPoll(void)
{
    USHORT usCount = 0;
    BOOL bNeedPoll = TRUE;

    if( bTxStateEnabled ) {
        // Continue while all response bytes put in buffer or out of buffer
        while((bNeedPoll) && (usCount++ < MB_SERIAL_BUF_SIZE)) {
            // Calls the modbus stack callback function to let it fill the UART transmit buffer.
            bNeedPoll = pxMBFrameCBTransmitterEmpty( ); // callback to transmit FSM
        }
        ESP_LOGD(TAG, "MB_TX_buffer send: (%u) bytes\n", (unsigned)usCount);
        vMBPortSerialEnable(TRUE, FALSE);
        return TRUE;
    }
    return FALSE;
}

static void vUartTask(void *pvParameters)
{
    uint8_t buffer[MB_SERIAL_BUF_SIZE];
    assert(xRxBuffer);
    for(;;) {
        int size = usb_serial_jtag_read_bytes(buffer, MB_SERIAL_BUF_SIZE, portMAX_DELAY);
        uint32_t timeout = xTaskGetTickCount() + pdMS_TO_TICKS(35);
        while (size < MB_SERIAL_BUF_SIZE && xTaskGetTickCount() < timeout) {
            int n = usb_serial_jtag_read_bytes(buffer + size, MB_SERIAL_BUF_SIZE - size, timeout - xTaskGetTickCount());
            if (n <= 0) {
                ESP_LOGI(TAG, "Timeout, no data received");
                break;
            }
            size += n;
            timeout = xTaskGetTickCount() + pdMS_TO_TICKS(35);
        }
        if (size > 0) {
            ESP_LOGI(TAG,"Data event, length: %u", (unsigned)size);
            if (xRingbufferSend(xRxBuffer, buffer, size, 0) == pdTRUE) {
                // Read received data and send it to modbus stack
                usMBPortSerialRxPoll(size);
            }
        }
    }
    vTaskDelete(NULL);
}

BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate,
                        UCHAR ucDataBits, eMBParity eParity)
{
    esp_err_t xErr = ESP_OK;
    usb_serial_jtag_driver_config_t config = {
        .rx_buffer_size = MB_SERIAL_BUF_SIZE,
        .tx_buffer_size = MB_SERIAL_BUF_SIZE
    };
    xRxBuffer = xRingbufferCreate(MB_SERIAL_BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (xRxBuffer == NULL) {
        ESP_LOGE(TAG, "xRxBuffer create failed.");
        return FALSE;
    }

    xErr = usb_serial_jtag_driver_install(&config);
    MB_PORT_CHECK((xErr == ESP_OK),
            FALSE, "mb config failure, usb_serial_jtag_driver_install() returned (0x%x).", (int)xErr);

    // Create a task to handle UART events
    BaseType_t xStatus = xTaskCreatePinnedToCore(vUartTask, "uart_queue_task",
                                                    MB_SERIAL_TASK_STACK_SIZE,
                                                    NULL, MB_SERIAL_TASK_PRIO,
                                                    &xMbTaskHandle, MB_PORT_TASK_AFFINITY);
    if (xStatus != pdPASS) {
        vTaskDelete(xMbTaskHandle);
        // Force exit from function with failure
        MB_PORT_CHECK(FALSE, FALSE,
                "mb stack serial task creation error. xTaskCreate() returned (0x%x).",
                (int)xStatus);
    } else {
        vTaskSuspend(xMbTaskHandle); // Suspend serial task while stack is not started
    }
    return TRUE;
}

void vMBPortSerialClose(void)
{
    (void)vTaskSuspend(xMbTaskHandle);
    (void)vTaskDelete(xMbTaskHandle);
    if (xRxBuffer) {
        (void)vRingbufferDelete(xRxBuffer);
        xRxBuffer = NULL;
    }
    ESP_ERROR_CHECK(usb_serial_jtag_driver_uninstall());
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
    // Send one byte to UART transmission buffer
    // This function is called by Modbus stack
    UCHAR ucLength = usb_serial_jtag_write_bytes(&ucByte, 1, 0);
    return (ucLength == 1);
}

// Get one byte from intermediate RX buffer
BOOL xMBPortSerialGetByte(CHAR* pucByte)
{
    assert(pucByte != NULL);
    size_t itemSize = 0;
    CHAR *p = xRingbufferReceiveUpTo(xRxBuffer, &itemSize, MB_SERIAL_RX_TOUT_TICKS, 1);
    if (p == NULL) {
        return FALSE;
    }
    *pucByte = *p;
    vRingbufferReturnItem(xRxBuffer, p);
    return TRUE;
}
