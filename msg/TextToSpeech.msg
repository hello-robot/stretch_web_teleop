# The text to say
string text

# The voice to use. Valid options for this depend on the engine.
string voice

# The speed to speak at, in words per minute. Defaults to 110.
uint8 speed_wpm

# If a message is already being spoken, this flag controls what to do with this message:
# add it to a queue to be executed sequentially (Default), or interrupt the
# current message and queue to speak this message (in this case, the old queue gets
# discarded).
uint8 OVERRIDE_BEHAVIOR_QUEUE = 0
uint8 OVERRIDE_BEHAVIOR_INTERRUPT = 1
uint8 override_behavior
