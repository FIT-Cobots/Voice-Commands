from pocketsphinx import LiveSpeech
from enum import Enum
from word2number import w2n


class VoiceCommandException(Exception):
    pass
class NotRecognizedError(VoiceCommandException):
    pass
class MoveError(VoiceCommandException):
    pass
class PickUpError(VoiceCommandException):
    pass
class SetDownError(VoiceCommandException):
    pass
class GiveError(VoiceCommandException):
    pass


class VoiceCmd(Enum):
    RELEASE     = 0     # Open the claw
    MOVE        = 1     # Move/turn by a stated degree amount and direction
    PICKUP      = 2     # Pick up a stated item
    SETDOWN     = 3     # Set down the currently held item
    GIVE        = 4     # Give the operator a stated item
    NONE        = 5

class Item(Enum):
    CAN         = 0
    BLOCK       = 1
    BALL        = 2
    NONE        = 3

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


VoiceCmdWords   = [ ['drop', 'let', 'open', 'release'],
                    ['move', 'turn', 'rotate'],
                    ['pick', 'grab', 'take', 'get'],
                    ['set', 'put', 'leave'],
                    ['give', 'gimme', 'hand', 'hand it', 'hand me', 'pass'] ]
        
ItemWords       = [ ['a can', 'the can', 'that can', 'this can', 'soda can'],
                    ['block', 'cube'],
                    ['ball', 'sphere'] ]

DirectionWords  = [ ['left', 'clockwise'],
                    ['right', 'counter clockwise'],
                    ['up'],
                    ['down'] ]

# Used for choosing which set of word lists to use
EnumType = [VoiceCmd, Item, Direction]
Words = [VoiceCmdWords, ItemWords, DirectionWords]

WakeWord = 'ar3'


class VoiceCommander:
    
    def __init__(self):

        self.VoiceCmdFunc   = [ self.release,
                                self.move,
                                self.pickUp,
                                self.setDown,
                                self.give,
                                self.nothing ]

        self.holdingItem = Item.NONE  # Item currently being held
        self.currentPhrase = None  # Most recent phrase heard


    def nothing(self):
        
        raise NotRecognizedError('No voice command was recognized')


    def release(self):
        
        print('Attempting to release the claw')

        # Send command to action server to open the claw

        # If successful, set self.holdingItem
        self.holdingItem = Item.NONE


    def move(self):
        
        degrees     = self.parseWords(WordsType.DEGREES)
        direction   = self.parseWords(WordsType.DIRECTION)
        
        if direction == Direction.NONE:
            if degrees % 180 == 0:
                # If a multiple of 180, left/right does not matter
                direction = Direction.LEFT
            raise MoveError('A direction to turn must be specified')

        print(f'Attempting to move {degrees} degrees {direction}')

        # Calculate new position from current position + degrees to rotate

        # Send command to action server to move to new position


    def pickUp(self):

        requestedItem = self.parseWords(WordsType.ITEM)

        # Make sure hand is currently empty
        if self.holdingItem != Item.NONE:
            raise PickUpError(f'Cannot pick up {requestedItem}, already holding {self.holdingItem}')

        if requestedItem == Item.NONE:
            raise PickUpError(f'Did not hear an item specified to pick up')

        print(f'Attempting to pick up {requestedItem}')

        # Ask video recognition topic for position of requested item

        # Send command to action server to move arm to that position

        # Send command to action server to grab the item

        # If successful, set self.holdingItem
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
                raise SetDownError(f'Cannot set down {requestedItem}, not currently holding it')

        print(f'Attempting to set down {requestedItem}')

        # Send command to action server to go down to the table/surface

        # Send command to action server to release the claw

        # If successful, set self.holdingItem
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
                    raise GiveError(f'Cannot give you {requestedItem}, already holding {self.holdingItem}')


        print(f'Attempting to give {self.holdingItem} to the operator')

        # Move to current hand position, checking if hand has moved
        # While robot hand is beyond some threshold distance to operator hand

            # Ask video recognition topic for position of operator hand

            # If operator hand has moved by some threshold distance 
            # (this has to be true at least once in order to work -- set intial value arbitrarily high)
            
                # Send command to action server to move to that position

        # While item has not yet been grabbed by user
            # Ask ? whether the item is being grabbed by user

        self.release()


    def parseWords(self, wordsType):

        if wordsType != WordsType.DEGREES:

            enumType = EnumType[wordsType.value]

            for i, wordList in enumerate(Words[wordsType.value]):
                for word in wordList:
                    if word in self.currentPhrase:
                        return enumType(i)

            return enumType.NONE


        phrase = self.currentPhrase
        phrase = phrase.replace('a hundred', 'one hundred')

        try:
            return w2n.word_to_num(phrase)
        except Exception:
            raise MoveError('Failed to interpret number of degrees')


    def run(self):

        speech = LiveSpeech(
            lm='/home/daniel/Courses/SeniorDesign/voiceCommands/PocketsphinxPython/main/ar3.lm',
            dic='/home/daniel/Courses/SeniorDesign/voiceCommands/PocketsphinxPython/main/ar3.dict'
        )

        print(f"Listening for the wake word '{WakeWord}'...")

        wakeWordSpoken = False
        for phrase in speech:

            if not wakeWordSpoken:

                if WakeWord in str(phrase):

                    # Wait for the command in the next phrase
                    wakeWordSpoken = True
                    print(f"Heard '{WakeWord}', waiting for a command...")
                
                continue

            # Only need to set self.currentPhrase if wake word was spoken
            self.currentPhrase = str(phrase)
            print(f"Heard '{self.currentPhrase}'")

            requestedVoiceCmd = self.parseWords(WordsType.VOICECMD)

            try:
                self.VoiceCmdFunc[requestedVoiceCmd.value]()
            except VoiceCommandException as e:
                print(f'Error: {e}')

            wakeWordSpoken = False
            self.currentPhrase = None

            print(f"\nListening for the wake word '{WakeWord}'...")


VoiceCommander = VoiceCommander()
VoiceCommander.run()