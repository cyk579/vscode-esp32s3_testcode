#include "sdkconfig.h"
#include "esp_log.h"

void subject3_ble_car_main(void);
void subject3_usb_mic_probe_main(void);

void app_main(void)
{
#if CONFIG_SUBJECT3_BLE_BASELINE
    ESP_LOGI("subject3", "PHONE_ASR_BLE: recognition on phone, omni control on ESP32-S3");
    subject3_ble_car_main();
#else
    ESP_LOGI("subject3", "USB_MIC_PROBE: audio diagnostics only; motors disabled; no ASR engine yet");
    subject3_usb_mic_probe_main();
#endif
}
