#include "esp_log.h"
#include "serialinterface.h"
#include "driver/uart.h"
#include "freertos/queue.h" // for queue support
#include "soc/uart_periph.h"

// A queue to handle UART event.
static QueueHandle_t xMbUartQueue;
static TaskHandle_t  xMbTaskHandle;
static const CHAR *TAG = "USB_SERIAL";

// The UART hardware port number
static UCHAR ucUartNumber = UART_NUM_MAX - 1;

static void vUartTask(void *pvParameters)
{
    uart_event_t xEvent;
    USHORT usResult = 0;
    for(;;) {
        if (xMBPortSerialWaitEvent(xMbUartQueue, (void*)&xEvent, portMAX_DELAY)) {
            ESP_LOGD(TAG, "MB_uart[%u] event:", (unsigned)ucUartNumber);
            switch(xEvent.type) {
                //Event of UART receving data
                case UART_DATA:
                    ESP_LOGD(TAG,"Data event, length: %u", (unsigned)xEvent.size);
                    // This flag set in the event means that no more
                    // data received during configured timeout and UART TOUT feature is triggered
                    if (xEvent.timeout_flag) {
                        // Get buffered data length
                        ESP_ERROR_CHECK(uart_get_buffered_data_len(ucUartNumber, &xEvent.size));
                        // Read received data and send it to modbus stack
                        usResult = usMBPortSerialRxPoll(xEvent.size);
                        ESP_LOGD(TAG,"Timeout occured, processed: %u bytes", (unsigned)usResult);
                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGD(TAG, "hw fifo overflow");
                    xQueueReset(xMbUartQueue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGD(TAG, "ring buffer full");
                    xQueueReset(xMbUartQueue);
                    uart_flush_input(ucUartNumber);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGD(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGD(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGD(TAG, "uart frame error");
                    break;
                default:
                    ESP_LOGD(TAG, "uart event type: %u", (unsigned)xEvent.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

bool serial_init(UCHAR ucPORT, ULONG ulBaudRate,
                        UCHAR ucDataBits, eMBParity eParity)
{
    esp_err_t xErr = ESP_OK;
    // Set communication port number
    ucUartNumber = ucPORT;
    // Configure serial communication parameters
    UCHAR ucParity = UART_PARITY_DISABLE;
    UCHAR ucData = UART_DATA_8_BITS;
    switch(eParity){
        case MB_PAR_NONE:
            ucParity = UART_PARITY_DISABLE;
            break;
        case MB_PAR_ODD:
            ucParity = UART_PARITY_ODD;
            break;
        case MB_PAR_EVEN:
            ucParity = UART_PARITY_EVEN;
            break;
        default:
            ESP_LOGE(TAG, "Incorrect parity option: %u", (unsigned)eParity);
            return FALSE;
    }
    switch(ucDataBits){
        case 5:
            ucData = UART_DATA_5_BITS;
            break;
        case 6:
            ucData = UART_DATA_6_BITS;
            break;
        case 7:
            ucData = UART_DATA_7_BITS;
            break;
        case 8:
            ucData = UART_DATA_8_BITS;
            break;
        default:
            ucData = UART_DATA_8_BITS;
            break;
    }
    uart_config_t xUartConfig = {
        .baud_rate = ulBaudRate,
        .data_bits = ucData,
        .parity = ucParity,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 2,
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
        .source_clk = UART_SCLK_DEFAULT,
#else
        .source_clk = UART_SCLK_APB,
#endif
    };
    // Set UART config
    xErr = uart_param_config(ucUartNumber, &xUartConfig);
    MB_PORT_CHECK((xErr == ESP_OK),
            FALSE, "mb config failure, uart_param_config() returned (0x%x).", (int)xErr);
    // Install UART driver, and get the queue.
    xErr = uart_driver_install(ucUartNumber, MB_SERIAL_BUF_SIZE, MB_SERIAL_BUF_SIZE,
                                    MB_QUEUE_LENGTH, &xMbUartQueue, MB_PORT_SERIAL_ISR_FLAG);
    MB_PORT_CHECK((xErr == ESP_OK), FALSE,
            "mb serial driver failure, uart_driver_install() returned (0x%x).", (int)xErr);
#if !CONFIG_FMB_TIMER_PORT_ENABLED
    // Set timeout for TOUT interrupt (T3.5 modbus time)
    xErr = uart_set_rx_timeout(ucUartNumber, MB_SERIAL_TOUT);
    MB_PORT_CHECK((xErr == ESP_OK), FALSE,
            "mb serial set rx timeout failure, uart_set_rx_timeout() returned (0x%x).", (int)xErr);
#endif

    // Set always timeout flag to trigger timeout interrupt even after rx fifo full
    uart_set_always_rx_timeout(ucUartNumber, true);

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

void serial_close(void)
{
    (void)vTaskSuspend(xMbTaskHandle);
    (void)vTaskDelete(xMbTaskHandle);
    ESP_ERROR_CHECK(uart_driver_delete(ucUartNumber));
}

bool put_byte(uint8_t byte)
{
    // Send one byte to UART transmission buffer
    // This function is called by Modbus stack
    UCHAR ucLength = uart_write_bytes(ucUartNumber, &byte, 1);
    return (ucLength == 1);
}

// Get one byte from intermediate RX buffer
bool get_byte(uint8_t* pucByte)
{
    assert(pucByte != NULL);
    USHORT usLength = uart_read_bytes(ucUartNumber, (uint8_t*)pucByte, 1, MB_SERIAL_RX_TOUT_TICKS);
    return (usLength == 1);
}

static void flush_output(void)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_wait_tx_done(ucUartNumber, MB_SERIAL_TX_TOUT_TICKS));
}

static void flush_input(void)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_flush_input(ucUartNumber));
}

static void resume_task(void)
{
    vTaskResume(xMbTaskHandle);
}

static void suspend_task(void)
{
    vTaskSuspend(xMbTaskHandle);
}

static const SerialInterface_t kUartSerialApi = {
    .init = serial_init,
    .close = serial_close,
    .put_byte = put_byte,
    .get_byte = get_byte,
    .flush_input = flush_input,
    .flush_output = flush_output,
    .resume = resume_task,
    .suspend = suspend_task,
};

const SerialInterface_t *uart_serial_get_api(void)
{
    return &kUartSerialApi;
}
