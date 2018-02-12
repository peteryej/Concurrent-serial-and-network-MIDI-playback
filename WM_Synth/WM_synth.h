//////////////////////////////////////////////////////////////
//         waveform mixer task for rtos framework           //
//  program built upon wm8731.h for arduino due (powerdue)  //
//  when using this program, note that DMAHandler is used   //
//  Will probably crash if run concurrently with other DMAs //
//////////////////////////////////////////////////////////////

#ifndef WM_SYNTH
#define WM_SYNTH


#include <FreeRTOS_ARM.h>
#include "wm8731.h"

#include "instruments.h"

//#define DEBUG
#define SYNTH_CHANNELS 16 //if too many channels, will slow down and break down the sound
#define WAVE_ARRAY_SIZE 1024 //this is the number of points in the wave arrays. 
                             //Unlike the WM8731_audio, I used a full wave. We have enough memory, and most tones are not symmetrical

typedef struct synth_channel{
    const int16_t * waveArray; //pointer to wave array
    uint32_t waveIndex;  //number that tracks where in the waveArray each channel is at.
    uint32_t stepSize;  //calculated from frequency
    uint8_t volume;       //0-255, where 0 is silence, 255 is capped with 32767 as 32767 sent to WM
};

const struct synth_channel SYNTH_CHANNEL_DEFAULT = {.waveArray=_sine, .waveIndex=0, .stepSize=0, .volume=0};

class WMSynthClass
{
  public:
    TaskHandle_t xSynthTaskHandle;
    synth_channel xChannel[SYNTH_CHANNELS];
    
    WMSynthClass();
    
    TaskHandle_t startSynth(UBaseType_t uxPriority); 
        //Registers the WMSynthClass with xTaskCreate and priority.
        //Also starts the DMA
        //please give it high priority, else sound will break up
        //returns NULL is something bad happens.    

    void stopSynth();
        //Stops DMA
        //Removes the Task with vTaskDelete

    //void infoSynth();
        //returns info on the synth status

    void setMasterVolume( unsigned char value );
        //ripped from WM8731_Audio.cpp



    uint8_t playTone(uint16_t frequency, uint8_t volume); 
        // Start a new channel at the frequency / volume
        //does nothing and returns 0 if no free channels left
    uint8_t playTone(uint16_t frequency, uint8_t volume, uint8_t channel, uint8_t waveform = 0); 
        //Changes the frequency / volume of a channel 
        //returns channel number if success. does nothing and returns 0 if channel > SYNTH_CHANNELS

    void stopTone(uint16_t channel); 
        // Stop playback of a channel. does nothing if channel > SYNTH_CHANNELS
    uint8_t stopToneAF(uint16_t frequency); 
        // Stops playback of first tone set to a certain frequency
        // returns channel of stopped tone, or 0 if no tone stopped


    uint8_t playToneN(uint16_t midinote, uint8_t volume, uint8_t waveform = 0);
      
    uint8_t stopToneN(uint16_t midinote); 
        //just for you, midi specs


    void stopAllTones();
        // Stops all channels (in case you lost track?)
 
  private:


    SemaphoreHandle_t xSynthSem;
    uint32_t midi_note_step[128];
    void setupWMchip();
    void shutdownWMchip();
    //void WavMixer( void* pvParameters ); //removed from class because FreeRTOS dislikes C++ classes

};

extern WMSynthClass WMSynth;

#endif

