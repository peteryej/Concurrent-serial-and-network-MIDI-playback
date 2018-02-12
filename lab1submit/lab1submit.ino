/*
* CESA Lab1
* 
* Play MIDI audio stream from serial input and network input concurrently
* Use two tasks to read the two MIDI input streams and put the callback  
* functions in a queue. A third task reads from the queue and calls WMSynth  
* to generate sound according to the events. 
* 
* Author: Jun Ye  (MIDI parser code and TcpMIDIStream function was given to 
* us by the TA and I implemented the event handling in the callbacks)  
* 
* Date: 2/9/2018
*/

#include <instruments.h>
#include <wm8731.h>
#include <WM_synth.h>
//#include <PowerDue.h>    //conflicts with <PowerDueWiFi.h>
#include <assert.h>
#include <FreeRTOS_ARM.h>
#include <IPAddress.h>
#include <PowerDueWiFi.h>
#include <Wire.h>
#include "MIDIParser.h"
//#include <WM8731_Audio.h>  //used for codec functions

#define SERVER_ADDR "10.230.8.1" 
#define SERVER_PORT 3000 
#define ANDREW_ID "jye1\n" // place your andrew id follow by a \n (newline char) 
#define BUFSIZE 64 

#define WIFI_SSID "PowerDue"
#define WIFI_PASS "powerdue"

MidiCallbacks_t midiCallbacks;
QueueHandle_t Input_Queue_Handle;

uint32_t _bpm =120; //default
uint32_t _ticksPerNote =96; //default
bool volumeChange = false;
bool TrackEnd = false;


uint16_t freqTable(int note){ 
  int freq = pow(2, (float)(note-69)/12)*440;
  return (uint16_t)freq;
}

struct midiEvent_t { 
  uint8_t status; 
  uint8_t note; 
  uint8_t velocity;
  uint8_t volume;
} midiEvent;



void midi_handleEvent(uint64_t delayTicks, uint8_t status, uint8_t note, uint8_t velocity){
  //handle note on and note off event 
  uint32_t waitTimeMS = TRACK_TIMER_PERIOD_MS(_ticksPerNote, _bpm) * delayTicks;
  delay(waitTimeMS);
  midiEvent.status = status;
  midiEvent.note = note;
  midiEvent.velocity = velocity;
  if (!xQueueSend(Input_Queue_Handle, &midiEvent, 500)){
    SerialUSB.println("Failed to send to queue. Queue is full.");
  }

}

void midi_volumeChanged(uint8_t volume){
//  Codec.setOutputVolume(volume);
  midiEvent.volume = volume;
  volumeChange = true;
//  SerialUSB.print("volume: ");
//  SerialUSB.println(volume);
//    WMSynth.setMasterVolume(volume);
}

void midi_trackStart(void){
  //handle track start event
}

void midi_trackEnd(void){
  //handle track end event
  TrackEnd = true;
}

void midi_setTicksPerNote(uint16_t ticksPerNote){
  _ticksPerNote = ticksPerNote;
}

void midi_setBPM(uint32_t bpm){
  _bpm = bpm;
}

//handle network input data
void TcpMIDIStream(void *arg){ 
  uint8_t buf[BUFSIZE]; 
  struct sockaddr_in serverAddr; 
  socklen_t socklen; 
  memset(&serverAddr, 0, sizeof(serverAddr)); 

  MidiParser parser = MidiParser(&midiCallbacks);

  serverAddr.sin_len = sizeof(serverAddr); 
  serverAddr.sin_family = AF_INET; 
  serverAddr.sin_port = htons(SERVER_PORT); 
  inet_pton(AF_INET, SERVER_ADDR, &(serverAddr.sin_addr)); 
  int s = lwip_socket(AF_INET, SOCK_STREAM, 0); 
  
  if(lwip_connect(s, (struct sockaddr *)&serverAddr, sizeof(serverAddr))){ 
    SerialUSB.println("Failed to connect to server"); 
    assert(false); 
  } 
  
  // register with the server 
  lwip_write(s, ANDREW_ID, strlen(ANDREW_ID)); 
  SerialUSB.println("Connected and waiting for events!"); 
  // listen for incoming events 
  while(1){ 
    memset(&buf, 0, BUFSIZE); 
    int n = lwip_read(s, buf, BUFSIZE); 
    for (int i=0; i<n; i++){ 
      // we received 'n' bytes 
      parser.feed(buf[i]);
    }
  }
  // close socket after everything is done 
  lwip_close(s); 
} 

//read queued input data
void ReadQueue(void *arg){
  while(1){
    if (!xQueueReceive(Input_Queue_Handle, &midiEvent, 5000)){
      SerialUSB.println("Failed to read data from queue. Queue is empty.");
    }else{
      //handle volume change event
      if(volumeChange){
        SerialUSB.print("volume: ");
        SerialUSB.println(midiEvent.volume);
        WMSynth.setMasterVolume(midiEvent.volume);
        volumeChange = false;
      }
      //handle track end event
      if(TrackEnd){
        WMSynth.stopAllTones();
        TrackEnd = false;
      }
      if(midiEvent.status == STATUS_NOTE_ON && midiEvent.velocity >0){
//          SerialUSB.println("note on.\n freq: ");
//          SerialUSB.println(freqTable(midiEvent.note));
        SerialUSB.println("note: ");
        SerialUSB.println(midiEvent.note);
        WMSynth.playToneN(midiEvent.note, midiEvent.velocity, 0);
      }else{
        SerialUSB.println("note off");
        WMSynth.stopToneN(midiEvent.note);
      }
    }
  }
  
}

//handle USB input MIDI stream data
void SerialUSBStream(void *arg){
  MidiParser parser = MidiParser(&midiCallbacks);
  while(1){
    parser.feed(SerialUSB.read());
  }
}

void onError(int errorCode){
  SerialUSB.print("Error received: ");
  SerialUSB.println(errorCode);
}

void onReady(){
  SerialUSB.println("Device ready");  
  SerialUSB.print("Device IP: ");
  SerialUSB.println(IPAddress(PowerDueWiFi.getDeviceIP()));  
  
  xTaskCreate(TcpMIDIStream, "TcpMIDIStream", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

void initLEDPins(){
  // turn off LEDs
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  turnOffLED();
}

void turnOffLED(){
  digitalWrite(6,LOW);
  digitalWrite(7,LOW);
  digitalWrite(8,LOW);
}

void setup() {
  // put your setup code here, to run once:
  portBASE_TYPE s1, s2;
  initLEDPins();
  
  midiCallbacks.HandleEvent = midi_handleEvent;
  midiCallbacks.VolumeChanged = midi_volumeChanged;
  midiCallbacks.TrackStart = midi_trackStart;
  midiCallbacks.TrackEnd = midi_trackEnd;
  midiCallbacks.SetTicksPerNote = midi_setTicksPerNote;
  midiCallbacks.SetBPM = midi_setBPM;

  SerialUSB.begin(0);
  while(!SerialUSB);

  PowerDueWiFi.init(WIFI_SSID, WIFI_PASS);
  PowerDueWiFi.setCallbacks(onReady, onError);

  s1 = xTaskCreate(ReadQueue, "ReadQueue", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  s2 = xTaskCreate(SerialUSBStream, "SerialUSBStream", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  WMSynth.startSynth(1);

  //create a queue with 500 entries of events
  Input_Queue_Handle = xQueueCreate(500,sizeof(midiEvent));
  
  vTaskStartScheduler();
  SerialUSB.println("Insufficient RAM");
  while(1);
}



void loop() {
  // put your main code here, to run repeatedly:

}
