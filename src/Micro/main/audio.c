#include <math.h>
#include <esp_task_wdt.h>
#include <unistd.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/ledc.h>

#include "audio.h"

void setup_audio(void){
    ledc_timer_config_t timer0_config = {
        .duty_resolution = LEDC_TIMER_1_BIT,
        .freq_hz = 24000*8,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer0_config);

    ledc_channel_config_t channel0_config = {
        .gpio_num = GPIO_NUM_22,
        .channel = LEDC_CHANNEL_0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 1,
        .hpoint = 0,
    };
    ledc_channel_config(&channel0_config);
    ledc_timer_config_t timer1_config = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 24000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer1_config);
    ledc_channel_config_t channel1_config = {
        .gpio_num = GPIO_NUM_23,
        .channel = LEDC_CHANNEL_1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&channel1_config);
}

/*
 * send_sin_wave generates a sine wave patten of (aprox.) 220Hz using the DAC
 * when followed by a sleep of 1/24000 s (simulating the default sample rate)
 */
void send_sine_wave(void* _arg){
    //we know we will not return for a good amount of time, and this is
    //intended, so delete the idle time to keep the watchdog from triggering

    esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(1));

    double t = 0;

    for(int counter = 0; ; counter++){
        if(counter >= 5000){
            counter = 0;

            esp_task_wdt_reset();
        }

        usleep(42); //hardcoded value for sample rate
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 127*(sin(t)+1));
        t += (double) 1/11; //hardcoded value for corret frequency
    }

    esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(1));
}
