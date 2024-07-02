# The text to say
string text

# Which engine to use
uint8 ENGINE_PYTTSX = 0
uint8 ENGINE_GTTS = 1
uint8 engine

# The voice to use. Valid options for this depend on the engine.
string voice

# If false, allow future messages to override this one. If false,
# add this message to queue of waiting message, guaranteeing that
# it gets executed.
bool queue
