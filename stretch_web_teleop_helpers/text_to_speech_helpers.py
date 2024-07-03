# Standard imports
from abc import ABC, abstractmethod
from enum import Enum
from typing import List

# Third-party imports
import pyttsx3
from rclpy.impl.rcutils_logger import RcutilsLogger


class TextToSpeechEngineType(Enum):
    """
    The TextToSpeechEngineType class enumerates the possible text-to-speech
    engines.
    """

    PYTTSX3 = 1


class TextToSpeechEngine(ABC):
    """
    Abstract base class for a text-to-speech engine that supports:
      - Setting the voice ID.
      - Setting the speed to default or slow.
      - Asynchronously speaking text.
      - Interrupting speech.
    """

    def __init__(self, logger: RcutilsLogger):
        """
        Initialize the text-to-speech engine.

        Parameters
        ----------
        logger : Logger
            The logger to use for logging messages.
        """
        self._logger = logger
        self._voice_ids: List[str] = []
        self._voice_id = ""
        self._is_slow = False

        # Whether or not this engine can speak asynchronously or not.
        self._can_say_async = False

    @property
    def voice_ids(self) -> List[str]:
        """
        Get the list of voice IDs available for the text-to-speech engine.
        """
        return self._voice_ids

    @property
    def voice_id(self) -> str:
        """
        Get the current voice ID for the text-to-speech engine.
        """
        return self._voice_id

    @voice_id.setter
    def voice_id(self, voice_id: str) -> None:
        """
        Set the current voice ID for the text-to-speech engine.
        """
        self._voice_id = voice_id

    @property
    def is_slow(self) -> bool:
        """
        Get whether the text-to-speech engine is set to speak slowly.
        """
        return self._is_slow

    @is_slow.setter
    def is_slow(self, is_slow: bool):
        """
        Set whether the text-to-speech engine is set to speak slowly.
        """
        self._is_slow = is_slow

    @abstractmethod
    def say_async(self, text: str):
        """
        Speak the given text asynchronously.
        """
        raise NotImplementedError

    @abstractmethod
    def say(self, text: str):
        """
        Speak the given text synchronously.
        """
        raise NotImplementedError

    @abstractmethod
    def stop(self):
        """
        Stop speaking the current text.
        """
        raise NotImplementedError


class PyTTSx3(TextToSpeechEngine):
    """
    Text-to-speech engine using pyttsx3.
    """

    def __init__(self, logger: RcutilsLogger):
        """
        Initialize the text-to-speech engine.

        Parameters
        ----------
        logger : Logger
            The logger to use for logging messages.
        """
        super().__init__(logger)
        self._engine = pyttsx3.init()

        # Initialize the voices
        voices = self._engine.getProperty("voices")
        # Variants documentation: https://espeak.sourceforge.net/languages.html
        variants = [
            "m1",
            "m2",
            "m3",
            "m4",
            "m5",
            "m6",
            "m7",
            "f1",
            "f2",
            "f3",
            "f4",
            "croak",
            "whisper",
        ]
        for voice in voices:
            self._voice_ids.append(voice.id)
            for variant in variants:
                self._voice_ids.append(voice.id + "+" + variant)
        self.voice_id = "default"

        # Initialize the speeds
        self.slow_speed = 100  # wpm
        self.default_speed = 150  # wpm

    @TextToSpeechEngine.voice_id.setter  # type: ignore
    def voice_id(self, voice_id: str) -> None:
        """
        Set the current voice ID for the text-to-speech engine.
        """
        self._voice_id = voice_id
        self._engine.setProperty("voice", voice_id)

    @TextToSpeechEngine.is_slow.setter  # type: ignore
    def is_slow(self, is_slow: bool):
        """
        Set whether the text-to-speech engine is set to speak slowly.
        """
        self._is_slow = is_slow
        if is_slow:
            self._engine.setProperty("rate", self.slow_speed)
        else:
            self._engine.setProperty("rate", self.default_speed)

    def say_async(self, text: str):
        """
        Speak the given text asynchronously.
        """
        self._logger.warn(
            "Asynchronous speaking is not supported for PyTTSx3 on Linux."
        )

    def say(self, text: str):
        """
        Speak the given text synchronously.
        """
        self._engine.say(text)
        self._engine.runAndWait()

    def stop(self):
        """
        Stop speaking the current text.
        """
        # Although interruptions are nominally supported in pyttsx3
        # (https://pyttsx3.readthedocs.io/en/latest/engine.html#examples),
        # in practice, the Linux implementation spins of an ffmpeg process
        # which can't be interrupted in its current implementation:
        # https://github.com/nateshmbhat/pyttsx3/blob/5d3755b060a980f48fcaf81df018dd06cbd17a8f/pyttsx3/drivers/espeak.py#L175 # noqa: E501
        self._logger.warn(
            "Asynchronous stopping is not supported for PyTTSx3 on Linux."
        )
