#include "sdkconfig.h"
#include "serialinterface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "port.h"
#include "ble_server.h"

#define TAG "usbserial"

static RingbufHandle_t xRxBuffer;
static RingbufHandle_t xTxBuffer;

static volatile TaskHandle_t  s_task_handle;
static EventGroupHandle_t s_event_group;

#define EVENT_GROUP_BIT_RX  (1 << 0)
#define EVENT_GROUP_BIT_TX  (1 << 1)
#define EVENT_GROUP_BIT_EXIT    (1 << 2)


static void event_handler(void *event_handler_arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    if (event_id == BLE_SERVER_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "BLE_SERVER_EVENT_CONNECTED");
    } else if (event_id == BLE_SERVER_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "BLE_SERVER_EVENT_DISCONNECTED");
    } else if (event_id == BLE_SERVER_EVENT_RX) {
        uint16_t len = *(uint16_t *)event_data;
        ESP_LOGI(TAG, "BLE_SERVER_EVENT_RX %"PRIu16" bytes", len);
        xEventGroupSetBits(s_event_group, EVENT_GROUP_BIT_RX);
    }
}


static void vSerialTask(void *pvParameters)
{
    size_t bytes_read = 0;
    uint8_t buffer[MB_SERIAL_BUF_SIZE];
    assert(xRxBuffer);
    while(true) {
        EventBits_t event_bits = xEventGroupWaitBits(s_event_group, EVENT_GROUP_BIT_RX|EVENT_GROUP_BIT_EXIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if ((event_bits & EVENT_GROUP_BIT_EXIT)) {
            break;
        }

        if (ble_server_receive(buffer, MB_SERIAL_BUF_SIZE, &bytes_read, 0) == ESP_OK) {
            ESP_LOGI(TAG, "ble_server_receive %zu bytes", bytes_read);
            if (bytes_read > 0 && xRingbufferSend(xRxBuffer, buffer, bytes_read, 0) == pdTRUE) {
                // Read received data and send it to modbus stack
                usMBPortSerialRxPoll(bytes_read);
            }
        }
    }

    vRingbufferDelete(xRxBuffer);
    xRxBuffer = NULL;
    vRingbufferDelete(xTxBuffer);
    xTxBuffer = NULL;
    vEventGroupDelete(s_event_group);
    s_event_group = NULL;

    ESP_LOGI(TAG, "RTU Serial task exit");
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

static bool ble_serial_init(UCHAR ucPORT, ULONG ulBaudRate,
                        UCHAR ucDataBits, eMBParity eParity)
{
    ESP_UNUSED(ucPORT);
    ESP_UNUSED(ulBaudRate);
    ESP_UNUSED(ucDataBits);
    ESP_UNUSED(eParity);

    esp_event_handler_register(BLE_SERVER_EVENT, BLE_SERVER_EVENT_RX, event_handler, NULL);
#if 0
    esp_err_t err = ble_server_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ble_server_start failed: %s", esp_err_to_name(err));
        goto RETURN_FALURE;
    }
#endif
    s_event_group = xEventGroupCreate();
    xRxBuffer = xRingbufferCreate(MB_SERIAL_BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (xRxBuffer == NULL) {
        ESP_LOGE(TAG, "xRxBuffer create failed.");
        goto RETURN_FALURE;
    }
    xTxBuffer = xRingbufferCreate(MB_SERIAL_BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (xTxBuffer == NULL) {
        ESP_LOGE(TAG, "xTxBuffer create failed.");
        goto RETURN_FALURE;
    }

    // Create a task to handle UART events
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
        goto RETURN_FALURE;
    } 
    
    vTaskSuspend(s_task_handle); // Suspend serial task while stack is not started
    return true;

RETURN_FALURE:
    if (s_event_group) {
        vEventGroupDelete(s_event_group);
        s_event_group = NULL;
    }
    if (xRxBuffer) {
        vRingbufferDelete(xRxBuffer);
        xRxBuffer = NULL;
    }
    if (xTxBuffer) {
        vRingbufferDelete(xTxBuffer);
        xTxBuffer = NULL;
    }
    ble_server_stop();
    esp_event_handler_unregister(BLE_SERVER_EVENT, BLE_SERVER_EVENT_RX, event_handler); 
    return false;
}

static void ble_serial_close(void)
{
    xEventGroupSetBits(s_event_group, EVENT_GROUP_BIT_EXIT);
    while (s_task_handle != NULL)  {
        vTaskResume(s_task_handle); // Resume receiver task
        ESP_LOGI(TAG, "Waiting for serial task to exit...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    esp_event_handler_unregister(BLE_SERVER_EVENT, BLE_SERVER_EVENT_RX,
                                 event_handler);
#if 0
    ble_server_stop();
#endif
}

static bool ble_serial_put_byte(uint8_t ucByte)
{
    if (xRingbufferSend(xTxBuffer, &ucByte, 1, 0) == pdTRUE) {
        return true;
    }
    ESP_LOGE(TAG, "xRingbufferSend failed.");
    return false;
}

// Get one byte from intermediate RX buffer
static bool ble_serial_get_byte(uint8_t* pucByte)
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

static void flush_input(void)
{

}

static void flush_output(void)
{
    // 将输出缓存数据全部发送出去
    size_t itemSize = 0;
    uint8_t *p;
    while ((p = xRingbufferReceiveUpTo(xTxBuffer, &itemSize, 0, MB_SERIAL_BUF_SIZE)) != NULL) {
        ble_server_transmit(p, itemSize);
        vRingbufferReturnItem(xTxBuffer, p);
    }
}

static const SerialInterface_t kBleSerialApi = {
    .init = ble_serial_init,
    .close = ble_serial_close,
    .put_byte = ble_serial_put_byte,
    .get_byte = ble_serial_get_byte,
    .flush_input = flush_input,
    .flush_output = flush_output,
    .resume = resume_task,
    .suspend = suspend_task,
};

const SerialInterface_t *ble_serial_get_api(void)
{
    return &kBleSerialApi;
}
