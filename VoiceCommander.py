from typing import Dict
from pocketsphinx import LiveSpeech
from enum import Enum
from word2number import w2n


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



# To add a new voice command, item, etc.:
# 1. Add to Enum
# 2. Add words to word list
# 3. Add the function to VoiceCommander
# 4. Add to VoiceCommander.voiceCmdFunc

class VoiceCmd(Enum):
    OPEN        = 0     # Open the claw
    CLOSE       = 1     # Close the claw
    MOVE        = 2     # Move/turn by a stated degree amount and direction
    PICKUP      = 3     # Pick up a stated item
    SETDOWN     = 4     # Set down the currently held item
    GIVE        = 5     # Give the operator a stated item
    NONE        = 6

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
                    ['close', 'grab', 'grasp'],
                    ['move', 'turn', 'rotate'],
                    ['pick', 'take', 'get'],
                    ['set', 'put', 'leave'],
                    ['give', 'gimme', 'hand', 'hand it', 'hand me', 'pass'] ]
        
ItemWords       = [ ['a can', 'the can', 'that can', 'this can', 'soda can'],
                    ['block', 'cube'],
                    ['ball', 'sphere'] ]

DirectionWords  = [ ['left', 'clockwise'],
                    ['right', 'counter clockwise'],
                    ['up'],
                    ['down'] ]

# Used for choosing which set of word lists to use for word parsing
EnumType = [VoiceCmd, Item, Direction]
Words = [VoiceCmdWords, ItemWords, DirectionWords]

WakeWord        = 'ar3'
LanguageModel   = 'ar3.lm'
Dict            = 'ar3.dict'


class VoiceCommander:
    
    def __init__(self):

        self.VoiceCmdFunc   = [ self.open,
                                self.close,
                                self.move,
                                self.pickUp,
                                self.setDown,
                                self.give,
                                self.nothing ]

        self.holdingItem = Item.NONE  # Item currently being held
        self.currentPhrase = None  # Most recent phrase heard


    def open(self):

        requestedItem = self.parseWords(WordsType.ITEM)

        if requestedItem != self.holdingItem:
            if requestedItem != Item.NONE:
                raise OpenError(f'Unable to drop {requestedItem} because currently holding {self.holdingItem}')

        print('Attempting to open the claw')

        # Send command to action server to open the claw

        # If successful, set self.holdingItem to nothing
        self.holdingItem = Item.NONE


    def close(self):

        print('Attempting to close the claw')

        # Send command to action server to close the claw

        # Ask ? if the hand is now grabbing an object
            # If so, ask video recognition what the object is

        # For now, just assume it grabbed a can
        self.holdingItem = Item.CAN


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
                raise SetDownError(f'Cannot set down {requestedItem}, not currently holding it')

        print(f'Attempting to set down {requestedItem}')

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

        # Send command to action server to open the claw


    def nothing(self):
        
        raise NotRecognizedError('No voice command was recognized')



    def parseWords(self, wordsType):

        if wordsType != WordsType.DEGREES:

            enumType = EnumType[wordsType.value]

            # Search for any command word in currentPhrase
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
            lm=LanguageModel,
            dic=Dict
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