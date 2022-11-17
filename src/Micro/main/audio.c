#include <math.h>
#include <driver/dac.h>
#include <esp_task_wdt.h>
#include <unistd.h>
#include <esp_log.h>

#include "audio.h"

void setup_audio(void){
    dac_output_enable(DAC_CHANNEL_1);
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
        dac_output_voltage(DAC_CHANNEL_1, 100*(sin(t)+1));
        t += (double) 1/11; //hardcoded value for corret frequency
    }

    esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(1));
}
