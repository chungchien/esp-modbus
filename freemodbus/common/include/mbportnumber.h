#pragma once
#include "driver/uart.h"

#define SERIAL_PORT_USB_JTAG (UART_NUM_MAX)     // USB虚拟串口
#define SERIAL_PORT_BLUETOOTH (UART_NUM_MAX + 1) // 蓝牙虚拟串口
#define SERIAL_PORT_COUNT (UART_NUM_MAX + 2)