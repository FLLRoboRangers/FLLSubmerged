from pybricks.hubs import PrimeHub # type: ignore
from pybricks.pupdevices import Motor, ColorSensor # type: ignore
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis # type: ignore
from pybricks.robotics import DriveBase # type: ignore
from pybricks.tools import wait, StopWatch # type: ignore
from pybricks.tools import multitask, run_task # type: ignore
from exits import *
from util import Robot

hub = PrimeHub()
hub.system.set_stop_button(Button.BLUETOOTH)
hub.imu.reset_heading(0)

leftDrive = Motor(Port.D, Direction.COUNTERCLOCKWISE)
rightDrive = Motor(Port.A)
leftAttachment = Motor(Port.C)
rightAttachment = Motor(Port.E)

driveBase = DriveBase(leftDrive, rightDrive, 62.4, 81.4)
colorLeft = ColorSensor(Port.B)
colorRight = ColorSensor(Port.F)

robot = Robot(hub, leftDrive, rightDrive, leftAttachment, rightAttachment, driveBase, colorLeft, colorRight)

exits = [exit1, exit2, sampleExit3, exit4, sampleExit5, sampleExit6,sampleExit7, sampleExit8, sampleExit9, sampleExit11,leftMotorControl, rightMotorControl, gyroTest]

async def main():
    selectedProgram = programSelect(1)
    exitcontinue = False
    while True:
        robot.hub.light.on(Color.VIOLET)

        returnValues = await multitask(programQuit(), exits[selectedProgram - 1](robot), race = True)

        driveBase.stop()
        leftAttachment.stop()
        rightAttachment.stop()

        #If the coroutine that finished was the exit and this wasn't the last exit, select next program
        if (returnValues[0] == None):
            if (selectedProgram < len(exits)):
                selectedProgram += 1
            print(f'Exit has finished in {returnValues[1] / 1000} seconds')
            selectedProgram = programSelect(selectedProgram, False)
            exitcontinue = True
        else:
            print('Exit was interrupted')
            exitcontinue = False


        if(exitcontinue ==  False):
            selectedProgram = programSelect(selectedProgram)

async def programQuit():
    middleButtonPressed = False
    clicked = False

    while clicked == False:
        if Button.CENTER in hub.buttons.pressed():
            middleButtonPressed = True

        if (Button.CENTER not in hub.buttons.pressed()) and middleButtonPressed == True:
            clicked = True
        
        await wait(10)
                                                                                                                                                                                                                                                    
    return 1

def hubdisplay(programIndex):
    if(programIndex <= 10):
        hub.display.number(programIndex)
    else:
    
        if(programIndex == 11): 
            hub.display.char("L")
        if(programIndex == 12): 
            hub.display.char("R")
        if(programIndex == 13): 
            hub.display.char("G")


def programSelect(programIndex, selectingProgram = True):
    robot.hub.light.blink(Color.BLUE, [250, 250])
    rightButtonPressed = False
    leftButtonPressed = False
    middleButtonPressed = False
    hubdisplay(programIndex)
    while selectingProgram:
        if Button.RIGHT in hub.buttons.pressed():
            rightButtonPressed = True
        
        if Button.LEFT in hub.buttons.pressed():
            leftButtonPressed = True

        if Button.CENTER in hub.buttons.pressed():
            middleButtonPressed = True

        if (Button.RIGHT not in hub.buttons.pressed()) and rightButtonPressed == True:
            programIndex += 1
            if programIndex > len(exits):
                programIndex = 1
            hubdisplay(programIndex)
            rightButtonPressed = False
            
        if (Button.LEFT not in hub.buttons.pressed()) and leftButtonPressed == True:
            programIndex -= 1
            if programIndex < 1:
                programIndex = len(exits)
            hubdisplay(programIndex)
            leftButtonPressed = False

        if (Button.CENTER not in hub.buttons.pressed()) and middleButtonPressed == True:
            selectingProgram = False
            middleButtonPressed = False
    
    print('Selected program: ' + str(programIndex))

    return programIndex

hub.speaker.beep()
run_task(main())
