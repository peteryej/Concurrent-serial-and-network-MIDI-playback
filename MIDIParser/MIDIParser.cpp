#include "MIDIParser.h"

#include <Assert.h>

MidiParser::MidiParser(MidiCallbacks_t *callbacks){
  _callbacks = callbacks;
  _readingFileHeader = false;
  _readingTrackHeader = false;
  _readingTrack = false;
  _runningStatus = 0;
  _buf = 0;
  _delay = 0;
  _status = 0;
  _argsCollected = 0;
  _metaType = 0;
  _metaLength = 0;
  _isDelay = false;
  _isStatus = false;
}

void MidiParser::feed(uint8_t in){
  _buf = (_buf << 8) + in;
  
  if(_readingFileHeader){
    _argsCollected++;
    // 10 bytes follows a header chunk
    if(_argsCollected == 10){
      // the last two bytes tell the number of ticks per note
      uint16_t ticksPerNote = (_buf & 0x0000FFFF);
      _callbacks->SetTicksPerNote(ticksPerNote);
      _readingFileHeader = false;
      _readingTrackHeader = true;
      _readingTrack = false;
      _argsCollected = 0;
      _isDelay = true;
    }
  } else if(_readingTrackHeader){
    _argsCollected++;
    // 8 bytes in a track header
    if(_argsCollected == 8){
      _readingTrackHeader = false;
      _readingTrack = true;
      _argsCollected = 0;
      _isDelay = true;
    }
  } else if(_readingTrack){
    
    if(_isDelay){
      _delay |= (in & 0x7F); 
      if(in & 0x80){ 
        // next byte is part of delay
        _delay = (_delay << 7);
      } else { 
        // this is the last delay byte
        _isDelay = false;
        _buf = 0;
        _status = 0;
        _isStatus = true;
      }
    } else {
      // get the event
      // first byte after delay should be status
      if(_isStatus){
        _status = in;
        _argsCollected = 0;
        _buf = 0;
        // check if this is a compressed event
        if( !(_status & 0x80) ){
          if(_runningStatus){
            _status = _runningStatus;
            _buf = in; // first byte of argument
            _argsCollected++;
            // remove channel parameters
            if((_status & 0xF0) != 0xF0){
              _status = _status & 0xF0;
            }
          } else {
            // cannot not have a status byte
            assert(false);
          }
        } else {
          // prep arguments
          _buf = 0;
        }
        _isStatus = false;  
        _runningStatus = _status;
      } else {
        _argsCollected++;
      }
      
      if(!_isStatus){
        switch(_status){
          // these midi events take two byte args
          case 0x80:
          case 0x90:
          case 0xA0:
          case 0xB0:
          case 0xE0:
            if(_argsCollected == 2){
              uint8_t note = (_buf >> 8) & 0xFF;
              uint8_t velocity = (_buf >> 0) & 0xFF;
              _callbacks->HandleEvent(_delay, _status, note, velocity);
              _isDelay = true;
              _delay = 0;
              _buf = 0;
            }
            break;
          
          // these midi events take one byte after
          case 0xC0:
          case 0xD0:
            if(_argsCollected == 1){
              _isDelay = true;
              _delay = 0;
              _buf = 0;
            }
            break;
            
          // these are meta events and have variable lengths
          // only interpret the important ones
          case 0xFF: {
            if(_argsCollected == 1){
              // this is the metatype, save it!
              _metaType = in;
            }
            if(_argsCollected == 2){
              // this is the meta length
              // it tells us how many more bytes to expect
              _metaLength = in;
            }
            if( (_argsCollected - 2) == _metaLength ){
              // got all meta bytes
              // handle it!
              switch(_metaType){
                case 0x2F:
                  // this is the end of track!
                  _readingTrack = false;
                  _runningStatus = 0;
                  _callbacks->TrackEnd();
                  break;
                
                case 0x51:
                  // tempo change!
                  // this should have three bytes
                  _callbacks->SetBPM(60000000/(_buf & 0x00FFFFFF));
                  break;
                  
                default:
                  // ignore other events
                  break;
              }
              // end of meta event
              _isDelay = true;
              _delay = 0;
              _buf = 0;
            }
          }
          
          // ignore SysEx events
          case 0xF0:
          case 0xF7:
            if(in == 0xF7){
              // end SysEx
              _isDelay = true;
              _delay = 0;
            }
          default:
            // ignore all other events
            break;
        }
      }
    }
    
  } else if(_buf == FILE_HEADER_SEQUENCE){
    
    // start of file
    _readingTrack = false;
    _readingTrackHeader = false;
    _readingFileHeader = true;
    _argsCollected = 0;
    _buf = 0;
    _delay = 0;
    _isDelay = true; // first message is delay
    _runningStatus = 0;
    _isStatus = false;
    _callbacks->TrackStart();
    
  } else {
    // real-time commands
    // buf has 3 bytes in at this point,
    if( ((_buf & 0x00FF0000) == 0x00800000) ){
      // Note Off
      uint8_t note = (_buf >> 8) & 0xFF;
      uint8_t velocity = (_buf >> 0) & 0xFF;
      _callbacks->HandleEvent(0, STATUS_NOTE_OFF, note, velocity);
      _buf = 0;
    } else if( ((_buf & 0x00FF0000) == 0x00900000) ){
      // Note On
      uint8_t note = (_buf >> 8) & 0xFF;
      uint8_t velocity = (_buf >> 0) & 0xFF;
      _callbacks->HandleEvent(0, STATUS_NOTE_ON, note, velocity);
      _buf = 0;
    } else if( ((_buf & 0x00FFFF00) == 0x00F00400) ){
      // Volume Change
      uint8_t volume = (_buf >> 0) & 0xFF;
      _callbacks->VolumeChanged(volume);
      _buf = 0;
    }
  }
}
