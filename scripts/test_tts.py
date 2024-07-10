test_gtts = True
if test_gtts:
    import time

    # Although this has more accent choices and all voices are female,
    # it is always possible that Google will buffer us...
    from io import BytesIO

    import sounddevice  # suppress ALSA warnings # noqa: F401
    from gtts import gTTS
    from pydub import AudioSegment
    from pydub.playback import _play_with_simpleaudio  # simpleaudio is a necessary dep

    text = "Hello, my name is Stretch, and I am a robot here to assist you."

    tts = gTTS(text=text, lang="en", tld="us")
    fp = BytesIO()
    tts.write_to_fp(fp)
    fp.seek(0)
    audio = AudioSegment.from_file(fp, format="mp3")
    playback = _play_with_simpleaudio(audio)
    time.sleep(1.0)
    playback.stop()

test_pyttsx3 = False
if test_pyttsx3:
    import pyttsx3

    engine = pyttsx3.init()
    voices = engine.getProperty("voices")
    for voice in voices:
        print(voice.age, voice.gender, voice.id, voice.languages, voice.name)
    # engine.setProperty('voice', "english")
    engine.setProperty("voice", "english+f4")
    engine.say("Hello, my name is Stretch")
    engine.runAndWait()