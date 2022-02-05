import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import random
from pocketsphinx import LiveSpeech
from enum import Enum
from word2number import w2n
# from text_to_speech import speak


WakeWord        = 'ar3'
LanguageModel   = '/home/daniel/Repos/Cobots/dev_ws/src/voicemove/voicemove/resources/ar3.lm'
Dict            = '/home/daniel/Repos/Cobots/dev_ws/src/voicemove/voicemove/resources/ar3.dict'


class VoiceListener(Node):

    def __init__(self):

        self.wakeWordSpoken = False

        self.speech = LiveSpeech(
            lm=LanguageModel,
            dic=Dict
        )

        super().__init__('voicemove')

        self._logger = self.get_logger()
        self._logger.info('voice_listener init')

        # Create publisher to send voice commands to voice_commander
        self._publisher = self.create_publisher(String, 'voice_listener', 10)

        self._timer = self.create_timer(0.1, self._listen)


    def _listen(self):

        self._logger.info("Listening...")

        if self.wakeWordSpoken:
            self._logger.info(random.choice(['Yes?', 'Go on.', 'Go ahead.']))

        # For continuous speech recognition, LiveSpeech (self.speech) must be iterated over
        for phrase in self.speech:

            if self.wakeWordSpoken:
                msg = String()
                msg.data = str(phrase)
                self._publisher.publish(msg)
                self.wakeWordSpoken = False
            else:
                if WakeWord in str(phrase):
                    self.wakeWordSpoken = True  # Wait for the command in the next phrase

            return
        


def main(args=None):

    rclpy.init(args=args)
    voice_listener = VoiceListener()
    rclpy.spin(voice_listener)

    voice_listener.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()