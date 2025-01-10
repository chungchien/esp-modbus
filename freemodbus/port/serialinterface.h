/**
 * @file serialinterface.h
 * @author Zhong JIanfeng (chungchien@163.com)
 * @brief 串口通讯接口
 * @version 0.1
 * @date 2025-01-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "mb.h"
#include "mbport.h"

typedef struct SerialInterface_t {
    bool (*init)(uint8_t uart_num, uint32_t baudrate, uint8_t data_bits, eMBParity parity);
    void (*close)(void);
    bool (*put_byte)(uint8_t byte);
    bool (*get_byte)(uint8_t *byte_ptr);
    void (*flush_input)(void); // Flush 输入缓冲区
    void (*flush_output)(void); // Reset 输出缓冲区
    void (*resume)(void); // Resume receiver task
    void (*suspend)(void); // Block receiver task
} SerialInterface_t;

USHORT usMBPortSerialRxPoll(size_t xEventSize);


const SerialInterface_t *usb_serial_get_api(void);
const SerialInterface_t *uart_serial_get_api(void);
const SerialInterface_t *ble_serial_get_api(void);