#!/bin/bash

# Usage: ./configure_audio.sh -s <speaker_name> -m <mic_name>
#
# To get a list of valid speaker names, run:
#     pactl list short sinks
# To get a list of valid microphone names, run:
#     pactl list short sources
# Note that with a microphone, there is no need to specify a device name if there is
# only one external mic plugged in; that one will **always** be prioritized over the
# robot's built-in microphone. The only way to use the robot's built-in microphone
# is to disconnect all external microphones.
SPEAKER_NAME="alsa_output.pci-0000_00_1f.3.analog-stereo"
if getopts ":s:" opt && [[ $opt == "s" ]]; then
    SPEAKER_NAME="$OPTARG"
fi
MIC_NAME=""
if getopts ":m:" opt && [[ $opt == "m" ]]; then
    MIC_NAME="$OPTARG"
fi

echo "Configuring audio...Note that this script may not work if X-11 forwarding is enabled."

echo "Setting speaker to $SPEAKER_NAME"
pactl set-default-sink $SPEAKER_NAME

if [ ${#MIC_NAME} -gt 0 ]; then
    echo "Setting microphone to $MIC_NAME"
    pactl set-default-source $MIC_NAME
fi

echo "Unmuting the speaker and setting its volume to 100%"
amixer set 'Master' unmute -q
amixer sset 'Master' 100% -q

echo "Unmuting the microphone and setting its gain to 100%"
amixer set 'Capture' cap -q
amixer sset 'Capture' 100% -q

echo "Done configuring audio!"
