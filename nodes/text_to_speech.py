#!/usr/bin/env python3

# Standard imports
import threading
from typing import List, Optional

# Third-party imports
import rclpy
import sounddevice  # suppress ALSA warnings # noqa: F401
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Local Imports
from stretch_web_teleop.msg import TextToSpeech
from stretch_web_teleop_helpers.text_to_speech_helpers import (
    GTTS,
    PyTTSx3,
    TextToSpeechEngine,
    TextToSpeechEngineType,
)


class TextToSpeechNode(Node):
    """
    The TextToSpeech node subscribes to a stream of text-to-speech commands
    from a topic and executes them.
    """

    def __init__(
        self,
        engine_type: TextToSpeechEngineType = TextToSpeechEngineType.PYTTSX3,
        rate_hz: float = 10.0,
    ):
        """
        Initialize the TextToSpeechNode.

        Parameters
        ----------
        engine_type : TextToSpeechEngineType
            The text-to-speech engine to use.
        rate_hz : float
            The rate at which to run the text-to-speech engine.
        """
        # Initialize the node
        super().__init__("text_to_speech")

        # Declare the attributes for the text-to-speech engine
        self.engine_type = engine_type
        self.engine: Optional[TextToSpeechEngine] = None
        self.initialized = False

        # Declare the attributes for the run thread
        self.rate_hz = rate_hz
        self.queue: List[TextToSpeech] = []
        self.queue_lock = threading.Lock()

        # Create the subscription
        self.create_subscription(
            TextToSpeech,
            "text_to_speech",
            self.text_to_speech_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
        )

    def initialize(self):
        """
        Initialize the text-to-speech engine.
        """
        if self.engine_type == TextToSpeechEngineType.PYTTSX3:
            self.engine = PyTTSx3(self.get_logger())
            self.initialized = True
        elif self.engine_type == TextToSpeechEngineType.GTTS:
            self.engine = GTTS(self.get_logger())
            self.initialized = True
        else:
            self.get_logger().error(f"Unsupported text-to-speech {self.engine_type}")

    def text_to_speech_callback(self, msg: TextToSpeech):
        """
        Callback for the text-to-speech topic.

        Parameters
        ----------
        msg : TextToSpeech
            The message containing the text to speak.
        """
        self.get_logger().info(f"Received: {msg}")
        # Interrupt if requested
        if msg.override_behavior == TextToSpeech.OVERRIDE_BEHAVIOR_INTERRUPT:
            if self.engine._can_say_async:
                self.engine.stop()
                with self.queue_lock:
                    self.queue.clear()
            else:
                self.get_logger().warn("Engine does not support interrupting speech")

        # Queue the text
        if len(msg.text) > 0:
            with self.queue_lock:
                self.queue.append(msg)

    def run(self):
        """
        Run the text-to-speech engine.
        """
        rate = self.create_rate(self.rate_hz)
        while rclpy.ok():
            # Sleep
            rate.sleep()

            # Send a single queued utterance to the text-to-speech engine
            if not self.engine.is_speaking():
                msg = None
                with self.queue_lock:
                    if len(self.queue) > 0:
                        msg = self.queue.pop(0)
                if msg is not None:
                    # Process the voice
                    if len(msg.voice) > 0:
                        if msg.voice != self.engine.voice_id:
                            self.engine.voice_id = msg.voice

                    # Process the speed
                    if msg.is_slow != self.engine.is_slow:
                        self.engine.is_slow = msg.is_slow

                    # Speak the text
                    if self.engine._can_say_async:
                        self.engine.say_async(msg.text)
                    else:
                        self.engine.say(msg.text)
                    self.get_logger().info(f"Saying: {msg.text}")


def main():
    rclpy.init()

    node = TextToSpeechNode(
        engine_type=TextToSpeechEngineType.GTTS,
    )
    node.get_logger().info("Created!")

    # Spin in the background, as the node initializes
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        kwargs={"executor": executor},
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
