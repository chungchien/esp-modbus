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
#include "freertos/queue.h" // for queue support
#include "driver/gpio.h"
#include "esp_log.h"        // for esp_log
#include "esp_err.h"        // for ESP_ERROR_CHECK macro

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "sdkconfig.h"              // for KConfig options
#include "port_serial_slave.h"
#include "serialinterface.h"
#include "mbportnumber.h"

// Note: This code uses mixed coding standard from legacy IDF code and used freemodbus stack
static const CHAR *TAG = "MB_SERIAL";

// The UART hardware port number
static BOOL bRxStateEnabled = FALSE; // Receiver enabled flag
static BOOL bTxStateEnabled = FALSE; // Transmitter enabled flag

static const SerialInterface_t *s_serial_port;

void vMBPortSerialEnable(BOOL bRxEnable, BOOL bTxEnable)
{
    // This function can be called from xMBRTUTransmitFSM() of different task
    if (bTxEnable) {
        bTxStateEnabled = TRUE;
    } else {
        bTxStateEnabled = FALSE;
    }
    if (bRxEnable) {     
        bRxStateEnabled = TRUE;
        s_serial_port->resume();
    } else {
        s_serial_port->suspend();
        bRxStateEnabled = FALSE;
    }
}


USHORT usMBPortSerialRxPoll(size_t xEventSize)
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
        s_serial_port->flush_input();
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
        s_serial_port->flush_output();
        vMBPortSerialEnable(TRUE, FALSE);
        return TRUE;
    }
    return FALSE;
}

BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate,
                        UCHAR ucDataBits, eMBParity eParity)
{
    if (ucPORT < UART_NUM_MAX) {
        s_serial_port = uart_serial_get_api();
    } else if (ucPORT == SERIAL_PORT_USB_JTAG) {
        s_serial_port = usb_serial_get_api();
    } else if (ucPORT == SERIAL_PORT_BLUETOOTH) {
        s_serial_port = ble_serial_get_api();
    } else {
        return FALSE;
    }

    return s_serial_port->init(ucPORT, ulBaudRate, ucDataBits, eParity);
}

void vMBPortSerialClose(void)
{
    s_serial_port->close();
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
    return s_serial_port->put_byte((uint8_t)ucByte);
}

// Get one byte from intermediate RX buffer
BOOL xMBPortSerialGetByte(CHAR* pucByte)
{
    return s_serial_port->get_byte((uint8_t *)pucByte);
}
