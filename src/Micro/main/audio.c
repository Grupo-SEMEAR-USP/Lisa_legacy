#include <math.h>
#include <driver/dac.h>

static double t = 0;

void setup_audio(void){
    dac_output_enable(DAC_CHANNEL_1);
}

/*
 * send_sin_wave generates a sine wave patten of (aprox.) 220Hz using the DAC
 * when followed by a sleep of 1/24000 s (simulating the default sample rate)
 */
void send_sin_wave(void){
    dac_output_voltage(DAC_CHANNEL_1, 50*(sin(t)+1));
    t += (double) 1/11; //hardcoded value for corret frequency
}