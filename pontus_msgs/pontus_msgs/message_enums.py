import inspect
from enum import Enum
from pontus_msgs.msg import CommandMode

def generateEnumDict(messageType):
    commandModeDict = {}
    for member in inspect.getmembers(messageType):
        if isinstance(member, tuple) and len(member) == 2:
            if isinstance(member[1], int):
                commandModeDict[member[0]] = member[1]
    return commandModeDict

CommandModeEnum = Enum('CommandModeEnum', generateEnumDict(CommandMode))