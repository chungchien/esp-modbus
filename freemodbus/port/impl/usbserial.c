#include "sdkconfig.h"
#include "serialinterface.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "port.h"

#define TAG "usbserial"

static RingbufHandle_t xRxBuffer;
static volatile TaskHandle_t  s_task_handle;
static volatile bool bMbExitTask = false; //  Flag to exit receiver task



static void vSerialTask(void *pvParameters)
{
    uint8_t buffer[MB_SERIAL_BUF_SIZE];
    assert(xRxBuffer);
    while(!bMbExitTask) {
        int size = usb_serial_jtag_read_bytes(buffer, MB_SERIAL_BUF_SIZE, pdMS_TO_TICKS(100));
        // ESP_LOGI(TAG, "usb_serial_jtag_read_bytes: %d", size);
        if (bMbExitTask || size < 0) {
            // ESP_LOGI(TAG, "break while reading from USB serial");
            break;
        }
        uint32_t timeout = xTaskGetTickCount() + pdMS_TO_TICKS(35);
        while (size > 0 && size < MB_SERIAL_BUF_SIZE && xTaskGetTickCount() < timeout) {
            int n = usb_serial_jtag_read_bytes(buffer + size, MB_SERIAL_BUF_SIZE - size, timeout - xTaskGetTickCount());
            if (n <= 0) {
                // ESP_LOGI(TAG, "Timeout, no data received");
                break;
            }
            size += n;
            timeout = xTaskGetTickCount() + pdMS_TO_TICKS(35);
        }
        if (size > 0) {
            // ESP_LOGI(TAG,"Data event, length: %u", (unsigned)size);
            if (xRingbufferSend(xRxBuffer, buffer, size, 0) == pdTRUE) {
                // Read received data and send it to modbus stack
                usMBPortSerialRxPoll(size);
            }
        }
    }
    ESP_LOGI(TAG, "usb_serial_jtag_driver_uninstall");
    usb_serial_jtag_driver_uninstall();
    (void)vRingbufferDelete(xRxBuffer);
    xRxBuffer = NULL;

    ESP_LOGI(TAG, "RTU Serial task exit");
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

static bool serial_init(UCHAR ucPORT, ULONG ulBaudRate,
                        UCHAR ucDataBits, eMBParity eParity)
{
    ESP_UNUSED(ucPORT);
    ESP_UNUSED(ulBaudRate);
    ESP_UNUSED(ucDataBits);
    ESP_UNUSED(eParity);

    esp_err_t xErr = ESP_OK;
    usb_serial_jtag_driver_config_t config = {
        .rx_buffer_size = MB_SERIAL_BUF_SIZE,
        .tx_buffer_size = MB_SERIAL_BUF_SIZE
    };
    xRxBuffer = xRingbufferCreate(MB_SERIAL_BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (xRxBuffer == NULL) {
        ESP_LOGE(TAG, "xRxBuffer create failed.");
        return false;
    }

    xErr = usb_serial_jtag_driver_install(&config);
    MB_PORT_CHECK((xErr == ESP_OK),
            false, "mb config failure, usb_serial_jtag_driver_install() returned (0x%x).", (int)xErr);

    // Create a task to handle UART events
    bMbExitTask = false;
    BaseType_t xStatus = xTaskCreatePinnedToCore(vSerialTask, "RTUSerialTask",
                                                    MB_SERIAL_TASK_STACK_SIZE,
                                                    NULL, MB_SERIAL_TASK_PRIO,
                                                    (TaskHandle_t *const)&s_task_handle, MB_PORT_TASK_AFFINITY);
    if (xStatus != pdPASS) {
        vTaskDelete(s_task_handle);
        // Force exit from function with failure
        MB_PORT_CHECK(FALSE, FALSE,
                "mb stack serial task creation error. xTaskCreate() returned (0x%x).",
                (int)xStatus);
    } else {
        vTaskSuspend(s_task_handle); // Suspend serial task while stack is not started
    }
    return true;
}

static void serial_close(void)
{
    bMbExitTask = true;
    while (s_task_handle != NULL)  {
        vTaskResume(s_task_handle); // Resume receiver task
        // ESP_LOGI(TAG, "Waiting for serial task to exit...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static bool put_byte(uint8_t ucByte)
{
    // Send one byte to UART transmission buffer
    // This function is called by Modbus stack
    int ucLength = usb_serial_jtag_write_bytes(&ucByte, 1, 0);
    return (ucLength == 1);
}

// Get one byte from intermediate RX buffer
static bool get_byte(uint8_t* pucByte)
{
    assert(pucByte != NULL);
    size_t itemSize = 0;
    uint8_t *p = xRingbufferReceiveUpTo(xRxBuffer, &itemSize, MB_SERIAL_RX_TOUT_TICKS, 1);
    if (p == NULL) {
        return false;
    }
    *pucByte = *p;
    vRingbufferReturnItem(xRxBuffer, p);
    return true;
}

static void resume_task(void)
{
    vTaskResume(s_task_handle);
}

static void suspend_task(void)
{
    vTaskSuspend(s_task_handle);
}

static void flush_output(void)
{
    // do nothing   
}

static void flush_input(void)
{
    // do nothing
}

static const SerialInterface_t kUsbSerialApi = {
    .init = serial_init,
    .close = serial_close,
    .put_byte = put_byte,
    .get_byte = get_byte,
    .flush_input = flush_input,
    .flush_output = flush_output,
    .resume = resume_task,
    .suspend = suspend_task,
};

const SerialInterface_t *usb_serial_get_api(void)
{
    return &kUsbSerialApi;
}
