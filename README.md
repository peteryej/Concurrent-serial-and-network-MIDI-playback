# Concurrent-serial-and-network-MIDI-playback
Play MIDI audio stream from serial input and network input concurrently. Use two tasks to read the two MIDI input streams and put the callback functions in a queue. A third task reads from the queue and calls WMSynth to generate sound according to the events. 
