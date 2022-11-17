#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/ledc.h>

#include "servo.h"

void startServo(void* _arg){
    ledc_timer_config_t timer_config = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 50,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_config);

    ledc_channel_config_t channel_config = {
        .gpio_num = GPIO_NUM_23,
        .channel = LEDC_CHANNEL_1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&channel_config);

    while(1){
        for(int i = 0; i < 256; i++){
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, i);
            vTaskDelay(4 / portTICK_PERIOD_MS);
        }
        for(int i = 255; i >= 0; i--){
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, i);
            vTaskDelay(4 / portTICK_PERIOD_MS);
        }
    }
}
