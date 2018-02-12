#ifndef __MIDIPARSER_H
#define __MIDIPARSER_H

#include <Arduino.h>

#define FILE_HEADER_SEQUENCE  0x4d546864    // MThd
#define TRACK_HEADER_SEQUENCE 0x4d54726b    // MTrk
#define END_OF_TRACK          0x00FF2F00    // EoT

#define STATUS_NOTE_OFF       0x80
#define STATUS_NOTE_ON        0x90

#define TRACK_TIMER_PERIOD_MS(ticks, bpm) (60000 / (ticks * bpm))   // macro to compute the period of a timer given a BPM

typedef struct {
  void (*HandleEvent)(uint64_t delay, uint8_t status, uint8_t note, uint8_t velocity);
  void (*VolumeChanged)(uint8_t volume);
  void (*TrackStart)(void);
  void (*TrackEnd)(void);
  void (*SetTicksPerNote)(uint16_t ticksPerNote);
  void (*SetBPM)(uint32_t bpm);
} MidiCallbacks_t;

class MidiParser {
  public:
    MidiParser(MidiCallbacks_t *callbacks);
  
    void feed(uint8_t in);
    
  private:
    MidiCallbacks_t *_callbacks;
    uint64_t _delay;
    uint32_t _buf;
    uint8_t _status;
    uint8_t _runningStatus;
    uint32_t _argsCollected;
    uint8_t _metaType;
    uint8_t _metaLength;
    boolean _readingTrack;
    boolean _readingFileHeader;
    boolean _readingTrackHeader;
    boolean _isDelay;
    boolean _isStatus;
};

#endif //__MIDIPARSER_H