#include <esp_log.h>

#include "wifi.h"
#include "audio.h"
#include "utils.h"

static const char* main_log_tag = "MAIN";

void app_main(void) {
    ESP_LOGI(main_log_tag, "Entering main");

    setup_storage();
    connect_wifi();
    http_rest_with_url();

    setup_audio();

    ESP_LOGI(main_log_tag, "Starting main loop");
    while (1) {
        ets_delay_us((double) 1/24000); //hardcoded value for sample rate
        send_sin_wave();
    }
}
