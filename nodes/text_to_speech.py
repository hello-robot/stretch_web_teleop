#!/usr/bin/env python3

# Standard imports
import threading
from enum import Enum
from typing import List, Optional

# Third-party imports
import pyttsx3
import rclpy
import sounddevice  # suppress ALSA warnings # noqa: F401
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Local Imports
from stretch_web_teleop.msg import TextToSpeech


class TextToSpeechEngine(Enum):
    """
    The TextToSpeechEngine class enumerates the possible text-to-speech
    engines that the TextToSpeechNode has implemented.
    """

    PYTTSX3 = 1


class TextToSpeechNode(Node):
    """
    The TextToSpeech node subscribes to a stream of text-to-speech commands
    from a topic and executes them.
    """

    def __init__(self, rate_hz: float = 10.0):
        """
        Initialize the TextToSpeechNode.
        """
        # Initialize the node
        super().__init__("text_to_speech")

        # Declare the attributes for the text-to-speech engine
        self.engine: Optional[pyttsx3.Engine] = None
        self.voice_ids: List[str] = []
        self.voice_id: Optional[str] = None
        self.speed_wpm = 200
        self.initialized = False

        # Declare the attributes for the run thread
        self.rate_hz = rate_hz
        self.queue: List[str] = []
        self.queue_lock = threading.Lock()

        # Create the subscription
        self.create_subscription(
            TextToSpeech,
            "text_to_speech",
            self.text_to_speech_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def initialize(self, engine: TextToSpeechEngine = TextToSpeechEngine.PYTTSX3):
        """
        Initialize the text-to-speech engine.

        Parameters
        ----------
        engine : TextToSpeechEngine
            The text-to-speech engine to use.
        """
        if engine == TextToSpeechEngine.PYTTSX3:
            self.engine = pyttsx3.init()
            self.initialize_voices(engine)
            self.set_speed(self.speed_wpm)
            self.initialized = True
        else:
            self.get_logger().error(f"Unsupported text-to-speech {engine}")

    def initialize_voices(self, engine: TextToSpeechEngine):
        """
        Initialize the options for voice IDs and the default voice ID.

        Parameters
        ----------
        engine : TextToSpeechEngine
            The text-to-speech engine to use.
        """
        if engine == TextToSpeechEngine.PYTTSX3:
            self.voice_ids = []
            voices = self.engine.getProperty("voices")
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
                self.voice_ids.append(voice.id)
                for variant in variants:
                    self.voice_ids.append(voice.id + "+" + variant)
            self.set_voice("default")
        else:
            self.get_logger().error(f"Unsupported text-to-speech {engine}")

    def set_voice(self, voice_id: str):
        """
        Set the voice ID for the text-to-speech engine.

        Parameters
        ----------
        voice_id : str
            The voice ID to use.
        """
        if voice_id in self.voice_ids:
            self.get_logger().info(f"Setting voice ID to {voice_id}")
            self.voice_id = voice_id
            self.engine.setProperty("voice", self.voice_id)
        else:
            self.get_logger().error(f"Unsupported voice ID {voice_id}")

    def set_speed(self, speed_wpm: int):
        """
        Set the speed in words per minute for the text-to-speech engine.

        Parameters
        ----------
        speed_wpm : int
            The speed in words per minute.
        """
        self.get_logger().info(f"Setting speed to {speed_wpm} wpm")
        self.speed_wpm = speed_wpm
        self.engine.setProperty("rate", speed_wpm)

    def text_to_speech_callback(self, msg: TextToSpeech):
        """
        Callback for the text-to-speech topic.

        Parameters
        ----------
        msg : TextToSpeech
            The message containing the text to speak.
        """
        # Interrupt if requested
        if msg.override_behavior == TextToSpeech.OVERRIDE_BEHAVIOR_INTERRUPT:
            self.engine.stop()

        # Process the voice
        if len(msg.voice) > 0:
            if msg.voice != self.voice_id:
                self.set_voice(msg.voice)

        # Process the speed
        if msg.speed_wpm > 0 and msg.speed_wpm != self.speed_wpm:
            self.set_speed(msg.speed_wpm)

        # Queue the text
        with self.queue_lock:
            self.queue.extend(msg.text.split())

    def run(self):
        """
        Run the text-to-speech engine.
        """
        rate = self.create_rate(self.rate_hz)
        while rclpy.ok():
            # TODO: Will need to abstract this once we have different engines
            self.get_logger().info(
                (
                    f"Running text-to-speech engine with voice {self.engine.getProperty('voice')} "
                    f"and speed {self.engine.getProperty('rate')} wpm"
                ),
                throttle_duration_sec=1.0,
            )

            # Send all queued text to the text-to-speech engine
            with self.queue_lock:
                if len(self.queue) > 0:
                    text = self.queue.pop(0)
                    self.get_logger().info(f"Popping '{text}' from queue")
                    self.engine.say(text)

            # Run the text-to-speech engine. Note that this blocks, but can be
            # interrupted with engine.stop()
            self.engine.runAndWait()

            # Sleep
            rate.sleep()


def main():
    rclpy.init()

    node = TextToSpeechNode()
    node.get_logger().info("Created!")

    # Spin in the background, as the node initializes
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spin_thread.start()

    # Run text-to-speech
    try:
        node.initialize()
        node.get_logger().info("Running!")
        node.run()
    except KeyboardInterrupt:
        pass

    # Spin in the foreground
    spin_thread.join()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
