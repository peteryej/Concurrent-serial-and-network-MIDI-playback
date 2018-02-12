///////////////////////////////////////////////////////////
//        waveform mixer task for rtos framework         //
//program built upon wm8731.h for arduino due (powerdue) //
///////////////////////////////////////////////////////////

#include "Arduino.h"
#include <Wire.h>
#include "WM_synth.h"
#include <FreeRTOS_ARM.h>
#include <math.h> 

WMSynthClass WMSynth = WMSynthClass();

void  WavMixer( void* pvParameters ); //implementation below

//////////////////////
// Public Interface //
//////////////////////
TaskHandle_t WMSynthClass::startSynth(UBaseType_t uxPriority){
    while(this->xSynthTaskHandle != NULL){ //multiple calls of startSynth, or memory error
      #ifdef DEBUG
        SerialUSB.println("Warning, xSynthTaskHandle not empty, did you call startSynth without calling stopSynth?");
      #endif
        this->stopSynth();
    }

    //registers wav mixing task to RTOS. Should block, waiting for signal if RTOS has started.
    xTaskCreate( WavMixer, "WMSynth", configMINIMAL_STACK_SIZE, NULL, uxPriority, &this->xSynthTaskHandle); //!figure out stack size
    if(this->xSynthTaskHandle == NULL){
      #ifdef DEBUG
        SerialUSB.println("Error, xTaskCreate failed.");
      #endif
        return NULL;
    }
  #ifdef DEBUG
    for(int i=0;i<128;i++){
      SerialUSB.println(this->midi_note_step[i]);
    }
  #endif

    Wire.begin();           //uses wire library to communicate with WM codec chip
    this->setupWMchip();    //sets it up to continuously play sounds, triggering DMA
                            //!volume is also set to maximum
                            
    //wm8731 library functions
    pio_B_SSC();   // sets up correct pinouts
    init_ssc();    // sets up SSC registers
    init_dma();    // sets up DMA registers
    ssc_dma_cfg(); // sets up DMA ping pong buffers, and starts stuff
    
    return this->xSynthTaskHandle;
}

void WMSynthClass::stopSynth(){
    //stopWMchip (which is the master in the DMA)
    this->shutdownWMchip();

    //stop/remove task
    vTaskDelete( this->xSynthTaskHandle );
    this->xSynthTaskHandle = NULL;
}

uint8_t WMSynthClass::playTone(uint16_t frequency, uint8_t volume){
    for (int i=0;i<SYNTH_CHANNELS;i++){
        if (this->xChannel[i].volume == 0){
            return playTone(frequency, volume, i+1);
        }
    }
  #ifdef DEBUG
    SerialUSB.println("No more channels available (= =\") ");
  #endif 
    return 0;
}
        // Start a new channel at the frequency / volume
        //does nothing and returns 0 if no free channels left

uint8_t WMSynthClass::playTone(uint16_t frequency, uint8_t volume, uint8_t channel, uint8_t waveform){
    
    //   (WAVE_ARRAY_SIZE*freq /raw_step_size ) = 44100Hz
    // ->freq *(WAVE_ARRAY_SIZE/44100) = raw_step_size
    // freq *(WAVE_ARRAY_SIZE/44100)*(2^10)= step_size
    uint32_t step_size = (frequency*(WAVE_ARRAY_SIZE*128))/11025; //! the *256 or 512 ???
    if (step_size >= WAVE_ARRAY_SIZE*1024){
        SerialUSB.println("SynthError: Frequency too high.");
        SerialUSB.println(step_size);
        return 0;
    }
    if (channel <= SYNTH_CHANNELS && channel > 0){
        this->xChannel[channel-1].stepSize = step_size;
        this->xChannel[channel-1].waveIndex = 0;
        this->xChannel[channel-1].volume = volume;
        this->xChannel[channel-1].waveArray = _instruments[waveform%INSTRUMENT_COUNT];
        return channel;
    }
    else{
        SerialUSB.println("SynthError: invalid channel");
        return 0;
    }
}
   
void WMSynthClass::stopTone(uint16_t channel){
    if (channel <= SYNTH_CHANNELS && channel > 0){
        this->xChannel[channel-1].volume = 0;
        this->xChannel[channel-1].waveIndex = 0;
    }
    else{
        SerialUSB.println("SynthError: invalid channel");
    }
}
uint8_t WMSynthClass::stopToneAF(uint16_t frequency){
    uint32_t step_size = (frequency*(WAVE_ARRAY_SIZE*128))/11025; //! note step size calculated no using macro

    for (int i=0;i<SYNTH_CHANNELS;i++){
        if(this->xChannel[i].stepSize == step_size && this->xChannel[i].volume != 0)
        {
            this->xChannel[i].volume = 0;
            this->xChannel[i].waveIndex = 0;
            return i+1;
        }

    }   
    return 0;
}

uint8_t WMSynthClass::playToneN(uint16_t midinote, uint8_t volume, uint8_t waveform){
    uint32_t step_size = this->midi_note_step[midinote];
    if (step_size >= WAVE_ARRAY_SIZE*1024){
        SerialUSB.println("SynthError: Frequency too high.");
        SerialUSB.println(step_size);
        return 0;
    }

    if (volume == 0){
      return this->stopToneN(midinote);
    }
    for (int i=0;i<SYNTH_CHANNELS;i++){
        if (this->xChannel[i].volume == 0){
           
            this->xChannel[i].stepSize = step_size;
            this->xChannel[i].waveIndex = 0;
            this->xChannel[i].volume = (uint8_t)volume;  //incidentally, this supports 0 volume midi note off
            this->xChannel[i].waveArray = _instruments[waveform%INSTRUMENT_COUNT];
            return i + 1;
        }
    }

    SerialUSB.println("no mo channos yo");
    return 0;
}
uint8_t WMSynthClass::stopToneN(uint16_t midinote){
    uint32_t step_size = this->midi_note_step[midinote];

    for (int i=0;i<SYNTH_CHANNELS;i++){
        if(this->xChannel[i].stepSize == step_size && this->xChannel[i].volume != 0)
        {
            this->xChannel[i].volume = 0;
            this->xChannel[i].waveIndex = 0;
            return i+1;
        }

    }   
    return 0;
}

void WMSynthClass::stopAllTones(){
    for (int i=0;i<SYNTH_CHANNELS;i++){
        this->xChannel[i].volume = 0;
        this->xChannel[i].waveIndex = 0;
    }   
}

void WMSynthClass::setMasterVolume( unsigned char value ){
    unsigned char reg;
    reg = ( 0x1FF & WM8731_LHEADOUT_LHPVOL_MASK ) | WM8731_LHEADOUT_LHPVOL(value) /* | WM8731_LHEADOUT_LZCEN */;
    set_reg( WM8731_LHEADOUT, reg );
    reg = (0x1FF & WM8731_RHEADOUT_RHPVOL_MASK ) | WM8731_RHEADOUT_RHPVOL(value) /* | WM8731_LHEADOUT_RZCEN */;
    set_reg( WM8731_RHEADOUT, reg );
}


////////////////////////
// Internal functions //
////////////////////////
WMSynthClass::WMSynthClass(){ //constructor, checks #define values and initializes values
    this->xSynthTaskHandle = NULL;
    if (SYNTH_CHANNELS>0){
        for(int i=0; i<SYNTH_CHANNELS;i++){
            this->xChannel[i] = SYNTH_CHANNEL_DEFAULT;
        }
    }
    else{
        SerialUSB.println("SYNTH_CHANNELS must be >= 1");
        while(1);
    }

    //fill out midi_note_step lookup chart values
    for(int i=0; i<128; i++){
        double value = pow(2.0, (((double)i-69.0)/12.0) )* (440*(WAVE_ARRAY_SIZE*128)/11025);
        this -> midi_note_step[i] = (uint32_t) value;
    }

}
void  WMSynthClass::setupWMchip(){ //! remove unused sampling code???
    set_buffer_addrs();
    set_reg(  WM8731_POWERDOWN, 0x50 ); //outpd=1, osc=0, clkout=1.
    set_reg(  WM8731_LLINEIN, 0x11F ); //line l&r

    set_reg( WM8731_LHEADOUT, ( 0x1FF & WM8731_LHEADOUT_LHPVOL_MASK ) | WM8731_LHEADOUT_LHPVOL(80)); //phone l&r
    set_reg( WM8731_RHEADOUT, ( 0x1FF & WM8731_LHEADOUT_LHPVOL_MASK ) | WM8731_LHEADOUT_LHPVOL(80) ); //phone l&r

    set_reg(  WM8731_ANALOG, 0x14 ); //audio path, dac, enable microphone 0001 0100
    set_reg(  WM8731_DIGITAL, 0x00 ); //digit path, dac mute -(1)000 = 0x08
    set_reg(  WM8731_INTERFACE, 0x4E ); //interface format, master -0(1)00 11 10 = 32-bit
    set_reg(  WM8731_SAMPLING, 0x20); //sampling,
    // 44.1k = 100000, non usb 11.289 MHz : 384
    set_reg(  WM8731_CONTROL, WM8731_CONTROL_ACTIVE ); //active=1
    set_reg(  WM8731_POWERDOWN, 0x40 ); //, osc=0, clkout=1.

    //maximize chip volume, use software to control volume.
    unsigned char value = 90;
    unsigned char reg;
    reg = ( 0x1FF & WM8731_LHEADOUT_LHPVOL_MASK ) | WM8731_LHEADOUT_LHPVOL(value) /* | WM8731_LHEADOUT_LZCEN */;
    set_reg( WM8731_LHEADOUT, reg );
    reg = (0x1FF & WM8731_RHEADOUT_RHPVOL_MASK ) | WM8731_RHEADOUT_RHPVOL(value) /* | WM8731_LHEADOUT_RZCEN */;
    set_reg( WM8731_RHEADOUT, reg );


}
void  WMSynthClass::shutdownWMchip(){
    set_reg(WM8731_POWERDOWN, WM8731_POWERDOWN_POWEROFF);
}



// extern char logs[1000];
// FreeRTOS does not work well with functions in classes...
void  WavMixer( void* pvParameters ){
    uint32_t ticks;
    uint16_t buf_index;
    uint8_t shift;
    uint8_t empty;

  #ifdef DEBUG
    SerialUSB.println((uint32_t)out[0],HEX); 
    SerialUSB.println((uint32_t)out[1],HEX); 
    // the output buffer is declared in wm8731 as int32_t  out[2][INP_BUFF] =  { 0};   
    // INP_BUFF is defined there (default 128)
    //!we may want to make this bigger, to decrease the processor load for IO
    // this makes the thing less responsive, but uses less processor time in context switches?
  #endif
    uint32_t t;
    while(1){
        
        ticks = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );//the same as xSemaphoreTake if only 1 task waiting
      #ifdef DEBUG
        if (ticks != 1){
            SerialUSB.println("skipped buffer");
        }
      #endif 
        //SerialUSB.println(DMAC->DMAC_CH_NUM[SSC_DMAC_TX_CH].DMAC_SADDR,HEX);
        if ((uint32_t)out[1]>DMAC->DMAC_CH_NUM[SSC_DMAC_TX_CH].DMAC_SADDR){
            buf_index = 1;
        }
        else{
            buf_index = 0;
        }
        empty = 1; //flags are bad mmmkay
        const int16_t * waveArray;
        uint32_t waveIndex;
        uint32_t stepSize; 
        uint16_t loopCounts = INP_BUFF>>3u; // divide by 8
        uint16_t index=0;
        // base algo achieves 560us for 5 channels
        for (uint16_t ch=0; ch<SYNTH_CHANNELS; ch++){
            // O1: putting these variables out here results in 400us for 5 channels
            // --> around 30% reduction in time
            waveArray = WMSynth.xChannel[ch].waveArray;
            waveIndex = WMSynth.xChannel[ch].waveIndex;
            stepSize = WMSynth.xChannel[ch].stepSize;
            if(WMSynth.xChannel[ch].volume != 0){
                if(empty){ //write if empty
                    empty = 0;
                    // O2: loop unrolling results in 290us for 5 channels
                    // --> additional 27.5% time saved
                    // --> total reduction to 52% of original time
                    for( uint16_t i = 0; i < loopCounts; i++) {
                        index = i*8;
                        if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                            out[buf_index][index++] = waveArray[waveIndex >> 10]<<12;
                            waveIndex += stepSize;
                        }
                        else{
                            waveIndex -= WAVE_ARRAY_SIZE << 10;
                            out[buf_index][index++] = waveArray[waveIndex >> 10 ]<<12;
                            waveIndex += stepSize;
                        }
                        if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                            out[buf_index][index++] = waveArray[waveIndex >> 10]<<12;
                            waveIndex += stepSize;
                        }
                        else{
                            waveIndex -= WAVE_ARRAY_SIZE << 10;
                            out[buf_index][index++] = waveArray[waveIndex >> 10 ]<<12;
                            waveIndex += stepSize;
                        }
                        if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                            out[buf_index][index++] = waveArray[waveIndex >> 10]<<12;
                            waveIndex += stepSize;
                        }
                        else{
                            waveIndex -= WAVE_ARRAY_SIZE << 10;
                            out[buf_index][index++] = waveArray[waveIndex >> 10 ]<<12;
                            waveIndex += stepSize;
                        }
                        if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                            out[buf_index][index++] = waveArray[waveIndex >> 10]<<12;
                            waveIndex += stepSize;
                        }
                        else{
                            waveIndex -= WAVE_ARRAY_SIZE << 10;
                            out[buf_index][index++] = waveArray[waveIndex >> 10 ]<<12;
                            waveIndex += stepSize;
                        }
                        if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                            out[buf_index][index++] = waveArray[waveIndex >> 10]<<12;
                            waveIndex += stepSize;
                        }
                        else{
                            waveIndex -= WAVE_ARRAY_SIZE << 10;
                            out[buf_index][index++] = waveArray[waveIndex >> 10 ]<<12;
                            waveIndex += stepSize;
                        }
                        if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                            out[buf_index][index++] = waveArray[waveIndex >> 10]<<12;
                            waveIndex += stepSize;
                        }
                        else{
                            waveIndex -= WAVE_ARRAY_SIZE << 10;
                            out[buf_index][index++] = waveArray[waveIndex >> 10 ]<<12;
                            waveIndex += stepSize;
                        }
                        if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                            out[buf_index][index++] = waveArray[waveIndex >> 10]<<12;
                            waveIndex += stepSize;
                        }
                        else{
                            waveIndex -= WAVE_ARRAY_SIZE << 10;
                            out[buf_index][index++] = waveArray[waveIndex >> 10 ]<<12;
                            waveIndex += stepSize;
                        }
                        if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                            out[buf_index][index++] = waveArray[waveIndex >> 10]<<12;
                            waveIndex += stepSize;
                        }
                        else{
                            waveIndex -= WAVE_ARRAY_SIZE << 10;
                            out[buf_index][index++] = waveArray[waveIndex >> 10 ]<<12;
                            waveIndex += stepSize;
                        }
                    }
                }
                else{ //add if not empty
                  // O2: loop unrolling
                  for( uint16_t i = 0; i < loopCounts; i++) {
                      index = i*8;
                      if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                          out[buf_index][index++] += waveArray[waveIndex >> 10]<<12;
                          waveIndex += stepSize;
                      }
                      else{
                          waveIndex -= WAVE_ARRAY_SIZE << 10;
                          out[buf_index][index++] += waveArray[waveIndex >> 10 ]<<12;
                          waveIndex += stepSize;
                      }
                      if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                          out[buf_index][index++] += waveArray[waveIndex >> 10]<<12;
                          waveIndex += stepSize;
                      }
                      else{
                          waveIndex -= WAVE_ARRAY_SIZE << 10;
                          out[buf_index][index++] += waveArray[waveIndex >> 10 ]<<12;
                          waveIndex += stepSize;
                      }
                      if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                          out[buf_index][index++] += waveArray[waveIndex >> 10]<<12;
                          waveIndex += stepSize;
                      }
                      else{
                          waveIndex -= WAVE_ARRAY_SIZE << 10;
                          out[buf_index][index++] += waveArray[waveIndex >> 10 ]<<12;
                          waveIndex += stepSize;
                      }
                      if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                          out[buf_index][index++] += waveArray[waveIndex >> 10]<<12;
                          waveIndex += stepSize;
                      }
                      else{
                          waveIndex -= WAVE_ARRAY_SIZE << 10;
                          out[buf_index][index++] += waveArray[waveIndex >> 10 ]<<12;
                          waveIndex += stepSize;
                      }
                      if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                          out[buf_index][index++] += waveArray[waveIndex >> 10]<<12;
                          waveIndex += stepSize;
                      }
                      else{
                          waveIndex -= WAVE_ARRAY_SIZE << 10;
                          out[buf_index][index++] += waveArray[waveIndex >> 10 ]<<12;
                          waveIndex += stepSize;
                      }
                      if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                          out[buf_index][index++] += waveArray[waveIndex >> 10]<<12;
                          waveIndex += stepSize;
                      }
                      else{
                          waveIndex -= WAVE_ARRAY_SIZE << 10;
                          out[buf_index][index++] += waveArray[waveIndex >> 10 ]<<12;
                          waveIndex += stepSize;
                      }
                      if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                          out[buf_index][index++] += waveArray[waveIndex >> 10]<<12;
                          waveIndex += stepSize;
                      }
                      else{
                          waveIndex -= WAVE_ARRAY_SIZE << 10;
                          out[buf_index][index++] += waveArray[waveIndex >> 10 ]<<12;
                          waveIndex += stepSize;
                      }
                      if (waveIndex>>10 < WAVE_ARRAY_SIZE){
                          out[buf_index][index++] += waveArray[waveIndex >> 10]<<12;
                          waveIndex += stepSize;
                      }
                      else{
                          waveIndex -= WAVE_ARRAY_SIZE << 10;
                          out[buf_index][index++] += waveArray[waveIndex >> 10 ]<<12;
                          waveIndex += stepSize;
                      }
                  }
                }
                WMSynth.xChannel[ch].waveIndex = waveIndex;
            }
        }
        if(empty){
            for( uint16_t i = 0; i < INP_BUFF; i++) {
                out[buf_index][i] = 0;
            }
        }
    }
    //vTaskDelete( NULL ); 
}



void DMAC_Handler(void)
{ 
    if (DMAC->DMAC_EBCISR & (DMAC_EBCISR_BTC0 << SSC_DMAC_TX_CH)) { //DMA send buffer finished

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR( WMSynth.xSynthTaskHandle, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        //vTaskNotifyGiveFromISR( WMSynth.xSynthTaskHandle, NULL);
    }
}

