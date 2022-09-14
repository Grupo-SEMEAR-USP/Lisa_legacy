#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>

#include "wifi.h"
#include "audio.h"
#include "utils.h"

static const char* main_log_tag = "MAIN";


void app_main(void) {
    ESP_LOGI(main_log_tag, "Entered app_main");

    setup_audio();
    setup_storage();
    connect_wifi();
    

    ESP_LOGI(main_log_tag, "Creating sine wave task");

    TaskHandle_t sine_task = NULL;
    xTaskCreatePinnedToCore(send_sine_wave, "sine wave", 
        configMINIMAL_STACK_SIZE, NULL, 15, &sine_task, 1
    );
    configASSERT(sine_task);


    ESP_LOGI(main_log_tag, "Entering main loop");
    //keeps pinging server
    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        http_rest_with_url();
    }
}
