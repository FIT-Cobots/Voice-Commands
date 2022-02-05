import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import random
from pocketsphinx import LiveSpeech
from enum import Enum
from word2number import w2n
# from text_to_speech import speak


class VoiceCommandException(Exception):
    pass
class OpenError(VoiceCommandException):
    pass
class MoveError(VoiceCommandException):
    pass
class PickUpError(VoiceCommandException):
    pass
class SetDownError(VoiceCommandException):
    pass
class GiveError(VoiceCommandException):
    pass
class NotRecognizedError(VoiceCommandException):
    pass


WakeWord        = 'ar3'
LanguageModel   = '/home/daniel/Repos/Cobots/dev_ws/src/voicemove/voicemove/resources/ar3.lm'
Dict            = '/home/daniel/Repos/Cobots/dev_ws/src/voicemove/voicemove/resources/ar3.dict'


# To add a new voice command, item, etc.:
# 1. Add to Enum below
# 2. Add words to word list
# 3. Add to VoiceCommander.voiceCmdFunc
# 4. Add corresponding function to VoiceCommander
# For steps 1 through 3, ensure consistent ordering between all three

class VoiceCmd(Enum):
    IGNORE      = 0     # Do not try to execute a command
    OPEN        = 1     # Open the claw
    CLOSE       = 2     # Close the claw
    MOVE        = 3     # Move/turn by a stated degree amount and direction
    PICKUP      = 4     # Pick up a stated item
    SETDOWN     = 5     # Set down the currently held item
    GIVE        = 6     # Give the operator a stated item
    NONE        = 7

class Item(Enum):
    CAN         = 0
    BLOCK       = 1
    BALL        = 2
    UNKNOWN     = 3
    NONE        = 4

class Direction(Enum):
    LEFT        = 0
    RIGHT       = 1
    UP          = 2
    DOWN        = 3
    NONE        = 4

class WordsType(Enum):
    VOICECMD    = 0
    ITEM        = 1
    DIRECTION   = 2
    DEGREES     = 3


VoiceCmdWords   = [ ['ignore', 'never mind', 'wait'],
                    ['drop', 'let it', 'let go', 'open', 'release'],
                    ['close', 'grab', 'grasp', 'take'],
                    ['move', 'turn', 'rotate'],
                    ['pick', 'get'],
                    ['set', 'put', 'leave'],
                    ['give', 'gimme', 'hand the', 'hand it', 'hand me', 'pass'] ]
        
ItemWords       = [ ['a can', 'the can', 'that can', 'this can', 'soda can'],
                    ['block', 'cube'],
                    ['ball', 'sphere'] ]

DirectionWords  = [ ['left', 'clockwise'],
                    ['right', 'counter clockwise'],
                    ['up'],
                    ['down'] ]

ConfirmWords    = ['yes', 'confirm', 'go ahead', 'do it']
DenyWords       = ['no', 'deny', 'stop', 'wait', 'never mind', 'ignore', 'hold on']

# Used for choosing which set of word lists to use for word parsing
EnumType = [VoiceCmd, Item, Direction]
Words = [VoiceCmdWords, ItemWords, DirectionWords]


class VoiceCommander(Node):
    
    def __init__(self):

        self.VoiceCmdFunc   = [ self.ignore,
                                self.open,
                                self.close,
                                self.move,
                                self.pickUp,
                                self.setDown,
                                self.give,
                                self.nothing ]

        self.holdingItem    = Item.NONE     # Item currently being held
        self.currentPhrase  = None          # Most recent phrase heard
        self.wakeWordSpoken = False         # Whether or not wakeword has been spoken

        self.speech = LiveSpeech(
            lm=LanguageModel,
            dic=Dict
        )

        super().__init__('voicemove')

        self._logger = self.get_logger()
        self._logger.info('voice_commander init')

        # Subscribe to voice_listener
        self.subscription = self.create_subscription(
            String,
            'voice_listener',
            self._process_speech,
            10)

        # # Subscribe to hand node for info about holding object
        # self.subscription = self.create_subscription(
        #     String,
        #     'topic',
        #     self._hand_callback,
        #     10)
        

    def _hand_callback(self, msg):
        # what object is being held
        self._logger.info(f'I heard: "{msg.data}" from the hand')


    def _process_speech(self, phrase):

        self.currentPhrase = phrase.data
        requestedVoiceCmd = self.parseWords(WordsType.VOICECMD)
        print(f"Heard '{self.currentPhrase}'")

        try:
            self.VoiceCmdFunc[requestedVoiceCmd.value]()
        except VoiceCommandException as e:
            self.tts(f'Error: {e}')

        self.wakeWordSpoken = False
        self.currentPhrase = None

        return


    def open(self):

        requestedItem = self.parseWords(WordsType.ITEM)

        if requestedItem != Item.NONE:
            if requestedItem != self.holdingItem:
                raise OpenError(f"Can't drop {requestedItem} because currently holding {self.holdingItem}")
            
        if self.holdingItem == Item.NONE:
            self.tts('Attempting to open the claw')
        else:
            self.tts(f'Attempting to drop the {self.holdingItem}')



        # Send command to action server to open the claw

        # If successful, set self.holdingItem to nothing
        self.holdingItem = Item.NONE


    def close(self):

        self.tts('Attempting to close the claw')

        # Send command to action server to close the claw

        # Ask ? if the hand is now grabbing an object
            # If so, ask video recognition what the object is

        self.holdingItem = Item.UNKNOWN


    def move(self):
        
        degrees = self.parseWords(WordsType.DEGREES)
        direction = self.parseWords(WordsType.DIRECTION)
        
        if direction == Direction.NONE:
            if degrees % 180 == 0:
                # Left/right does not matter
                direction = Direction.LEFT
            else:
                raise MoveError('A direction to turn must be specified')

        self.tts(f'Attempting to move {degrees} degrees {direction}')

        # Calculate new position from current position + degrees to rotate

        # Send command to action server to move to new position


    def pickUp(self):

        requestedItem = self.parseWords(WordsType.ITEM)

        # Make sure hand is currently empty
        if self.holdingItem != Item.NONE:
            raise PickUpError(f"Can't pick up {requestedItem}, already holding {self.holdingItem}")

        if requestedItem == Item.NONE:
            raise PickUpError(f'Did not hear a valid item specified to pick up')

        self.tts(f'Attempting to pick up {requestedItem}')

        # Send command to action server to open the claw

        # Ask video recognition topic for position of requested item

        # Move to current item position, checking if it has moved
        # While robot hand is beyond some threshold distance to the item

            # Ask video recognition topic for position of item

            # If the item has moved by some threshold distance 
            # (this has to be true at least once in order to work -- set intial value arbitrarily high)
            
                # Send command to action server to move to that position

        # Send command to action server to close the claw

        # If all successful, set self.holdingItem
        self.holdingItem = requestedItem


    def setDown(self):
    
        # Make sure hand is currently holding something
        if self.holdingItem == Item.NONE:
            raise SetDownError(f'Not currently holding anything to set down')

        requestedItem = self.parseWords(WordsType.ITEM)

        # If an item was mentioned
        if requestedItem != Item.NONE:
            # Make sure it is the currently held item
            if requestedItem != self.holdingItem:
                raise SetDownError(f"Can't set down {requestedItem}, not currently holding it")

        self.tts(f'Attempting to set down {requestedItem}')

        # Send command to action server to go down to the table/surface

        # Send command to action server to open the claw

        # If successful, set self.holdingItem to nothing
        self.holdingItem = Item.NONE


    def give(self):
        
        # If the claw is not currently holding anything
        if self.holdingItem == Item.NONE:

            # Pick up the item first
            self.pickUp()

        else:

            requestedItem = self.parseWords(WordsType.ITEM)

            # If a specific item was requested
            if requestedItem != Item.NONE:

                # Make sure the requested item is currently being held
                if requestedItem != self.holdingItem:
                    raise GiveError(f"Can't give you {requestedItem}, already holding {self.holdingItem}")


        self.tts(f'Attempting to give {self.holdingItem} to the operator')

        # Move to current hand position, checking if hand has moved
        # While robot hand is beyond some threshold distance to operator hand

            # Ask video recognition topic for position of operator hand

            # If operator hand has moved by some threshold distance 
            # (this has to be true at least once in order to work -- set intial value arbitrarily high)
            
                # Send command to action server to move to that position

        # While item has not yet been grabbed by user
            # Ask ? whether the item is being grabbed by user

        self.holdingItem = Item.NONE

        # Send command to action server to open the claw


    def ignore(self):
        
        self.tts('Ignoring')


    def nothing(self):
        
        raise NotRecognizedError("Sorry, I didn't get that.")


    def parseWords(self, wordsType):

        if wordsType == WordsType.DEGREES:
            try:
                return w2n.word_to_num(self.currentPhrase.replace('a hundred', 'one hundred'))
            except Exception:
                raise MoveError('Failed to interpret number of degrees')

        # Search for any command word in currentPhrase
        for i, wordList in enumerate(Words[wordsType.value]):
            for word in wordList:
                if word in self.currentPhrase:
                    return EnumType[wordsType.value](i)

        return EnumType[wordsType.value].NONE


    def tts(self, phrase):
        self._logger.info(phrase)
        # speak(phrase)



def main(args=None):

    rclpy.init(args=args)
    voice_commander = VoiceCommander()
    rclpy.spin(voice_commander)

    voice_commander.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()