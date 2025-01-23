from pybricks.hubs import PrimeHub # type: ignore
from pybricks.pupdevices import Motor, ColorSensor # type: ignore
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis # type: ignore
from pybricks.robotics import DriveBase # type: ignore
from pybricks.tools import wait, StopWatch # type: ignore
from util import Robot, LaunchSettings, gyroStraightRotations, gyroSpin, gyroStraightTime, gyroPivot, resetGyro, waitForStart, alignToStructure, archivePivot, archiveSpin, gyroSpinFS


front = 0 #constant used to indicate pivot direction
back = 1 #constant used to indicate pivot direction

#default values for PID constants
#GyroStraight : kp = 4, ki = 0, kd = 0.09
#GyroTurn : kp = 6, ki = 1.7, kd = 0.15, safetyThreshold = 3500

async def exit1(robot: Robot):
    launch1StraightSettings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    launch1TurnSettings = LaunchSettings(kp = 3, ki = 2.5, kd = 0.25, safetyThreshold = 3000)
    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)
    exitTimer = StopWatch()
   
    
    robot.rightAttachment.run_angle(1000 , 900, wait = False)
    robot.leftAttachment.run_time(-100 , 500 , wait = False)
    await gyroPivot(robot, front, 50)
    await gyroStraightRotations(robot, launch1StraightSettings, 1.1, 50, 65)
    await gyroPivot(robot, front, 25)
    await gyroStraightRotations(robot, launch1StraightSettings, 0.65, 23, 65)
    await gyroStraightRotations(robot, launch1StraightSettings, 0.7, 20, 50)
    await robot.rightAttachment.run_time(-1000, 1550)
    
    await gyroPivot(robot, back, -5)
    await gyroStraightRotations(robot, launch1StraightSettings, 0.5, -8, 50)
    await gyroSpin(robot, 89)
    await gyroStraightTime(robot, launch1StraightSettings, 2.4, 89, -30)
    doAlign = await alignToStructure(robot, launch1StraightSettings, back, 90, 2)
    if doAlign == 0:
        await gyroStraightTime(robot, launch1StraightSettings, 0.5, 90, -60)
    await robot.leftAttachment.run_time(170 , 3000)
    robot.leftAttachment.run_time(170 , 2000 , wait = False)
    await gyroStraightTime(robot, launch1StraightSettings, 1, 88, 40)
    robot.leftAttachment.run_angle(-100, 140, wait = False)
    await wait(300)

    #corabie
    await gyroStraightTime(robot, launch1StraightSettings, 2.2, 87, 40)
    await gyroStraightTime(robot, launch1StraightSettings, 1.5, 90, -30)
    
    await gyroPivot(robot, back, 30)
    await gyroStraightTime(robot, launch1StraightSettings, 2, 40, -100)

    return exitTimer.time()



async def exit2(robot: Robot):
    #launch2TurnSettings = LaunchSettings(kp = 7.5, ki = 1.7, kd = 0.15, safetyThreshold = 3000)
    launch2StraightSettings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)

    await resetGyro(robot)
    await waitForStart(robot)

    exitTimer = StopWatch()

    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)

    await gyroStraightRotations(robot, launch2StraightSettings, 1.3, 60, 70)
    await gyroStraightRotations(robot, launch2StraightSettings, 1.5, 60, 50)
    await gyroPivot(robot, front, 42)
    await gyroStraightRotations(robot, launch2StraightSettings, 0.25, 42, 25)
    await robot.leftAttachment.run_angle(-300, 140)
    await gyroStraightTime(robot, launch2StraightSettings, 1.7, 42, 50)
    await alignToStructure(robot, launch2StraightSettings, front, 42, 3)
    await gyroStraightRotations(robot, launch2StraightSettings, 0.2, 42, -23)
    await robot.rightAttachment.run_angle(200, 170)

    robot.rightAttachment.run_time(180, 3200, wait=False)
    await gyroStraightTime(robot, launch2StraightSettings, 1, 42, 18)
    await wait(2000)

    robot.rightAttachment.run_time(-170, 3500, wait=False)
    await wait(300)
    await gyroStraightRotations(robot, launch2StraightSettings, 0.08, 42, -30)
    await wait(2000)

    await gyroStraightRotations(robot, launch2StraightSettings, 0.7, 42, -60)
    await gyroSpin(robot, 75)
    await gyroStraightRotations(robot, launch2StraightSettings, 0.7, 75, -60)
    await gyroStraightRotations(robot, launch2StraightSettings, 2, 75, -100)


    return exitTimer.time()

async def sampleExit3(robot: Robot):
    '''while True:
        if Button.LEFT in robot.hub.buttons.pressed():
            robot.leftAttachment.run(-300)
        elif Button.RIGHT in robot.hub.buttons.pressed():
            robot.leftAttachment.run(300)
        else:
            robot.leftAttachment.stop()
        await wait(10)'''

    launch3StraightSettings = LaunchSettings(kp = 3.5, ki = 0, kd = 0.09)
    launch3TurnSettings = LaunchSettings(kp = 2.5, ki = 2.5, kd = 0.25, safetyThreshold = 3000)
    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)


    # robot.leftAttachment.run_time(-1000, 500 , wait=False)
    # await gyroStraightRotations(robot, launch3StraightSettings, 3.45, 35, 80)
    # await gyroSpin(robot, 0)
    # await gyroStraightTime(robot, launch3StraightSettings, 2, 0, 30)
    # await robot.leftAttachment.run_time(200, 1000)
    # await robot.leftAttachment.run_time(-70, 200)
    # await gyroStraightRotations(robot, launch3StraightSettings, 0.5, 0, -50)
    # await gyroSpin(robot, 90)
    # await gyroStraightRotations(robot, launch3StraightSettings, 2.5, 90, 50)

    robot.leftAttachment.run_time(-200,1000 , wait=False)
    await gyroSpin(robot, 34)
    await gyroStraightRotations(robot, launch3StraightSettings, 3.5, 34, 50)
    await gyroPivot(robot , front , 0)
    await gyroStraightTime(robot, launch3StraightSettings, 3, 0, 30)
    await robot.leftAttachment.run_time(1000, 700)
    await wait(500)
    
    await robot.leftAttachment.run_time(-100, 500)
    await gyroStraightRotations(robot, launch3StraightSettings, 0.5, 0, -30)

    await gyroPivot(robot , front , 90)
    await gyroStraightRotations(robot, launch3StraightSettings, 1.8, 90, 50)
    await gyroSpin(robot, 181)
    await gyroStraightTime(robot, launch3StraightSettings, 3, 181, -30)
    await gyroStraightRotations(robot, launch3StraightSettings, 0.4, 180, 50)
    await gyroSpin(robot, 280)
    robot.rightAttachment.run_time(500, 1000, wait=False)

    await gyroStraightRotations(robot, launch3StraightSettings, 3.05, 280, -40)
    await gyroSpin(robot, 255)
    await gyroStraightRotations(robot, launch3StraightSettings, 1.3, 255, -40)
    await wait(100)
    await gyroStraightRotations(robot, launch3StraightSettings, 0.55, 255, 40)
    await gyroSpin(robot, 275)
    await gyroStraightRotations(robot, launch3StraightSettings, 0.7, 275, -40)
    await wait(500)

    # await gyroStraightRotations(robot, launch3StraightSettings, 0.7, 270, 50)
    # await gyroPivot(robot , back , 300)
    # await gyroStraightRotations(robot, launch3StraightSettings, 0.4, 300, 50)
    # await gyroPivot(robot, back, 350)
    # await gyroStraightRotations(robot, launch3StraightSettings, 0.1, 350, -100)
    # await gyroPivot(robot, back, 360)
    # await gyroStraightRotations(robot, launch3StraightSettings, 2.5, 360, -100)
    

    await gyroStraightRotations(robot, launch3StraightSettings, 0.7, 275, 50)
    await gyroSpin(robot, 360)
    await gyroStraightRotations(robot, launch3StraightSettings, 1.8, 360, -80)
    await gyroStraightRotations(robot, launch3StraightSettings, 3, 315, -100)


async def sampleExit4(robot: Robot):
    launch4StraightSettings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)

    await resetGyro(robot)
    await waitForStart(robot)
    exitTimer = StopWatch()
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)


    
    robot.leftAttachment.run_time(800, 1500, wait=False)

    await gyroStraightRotations(robot, launch4StraightSettings, 3.8, 0, 50)
    await gyroSpin(robot, -25)
    await gyroStraightRotations(robot, launch4StraightSettings, 0.7, -25, 50)
    await gyroPivot(robot, front, 0)



    await gyroStraightTime(robot, launch4StraightSettings, 1.5, 0, 70)
    await robot.leftAttachment.run_time(-800, 1500)
    await gyroStraightRotations(robot, launch4StraightSettings, 1.6, 0, -40)
    await robot.rightAttachment.run_time(1000, 650)
    await gyroSpin(robot, 45)
    await gyroStraightRotations(robot, launch4StraightSettings, 2, 45, 70)
    await gyroStraightTime(robot, launch4StraightSettings, 1, 45, 70)
    await wait(1000)
    await gyroStraightRotations(robot, launch4StraightSettings, 1, 45, -70)
    await gyroSpin(robot, 0)
    await gyroStraightRotations(robot, launch4StraightSettings, 1.5, 0, -100)
    await gyroStraightRotations(robot, launch4StraightSettings, 2.5, -45, -100)
    

    return exitTimer.time()


async def sampleExit5(robot: Robot): #iesire david, pls nu stergeti iar
    launch5StraightSetings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    launch5TurnSetings =  LaunchSettings(kp = 6, ki = 1.7, kd = 0.15, safetyThreshold = 3500)

    await resetGyro(robot)
    await waitForStart(robot)

    await gyroStraightRotations(robot, launch5StraightSetings, 1.3, 0, -70)
    await wait(100)
    await gyroSpin(robot, -45)
    await wait(100)
    await gyroStraightRotations(robot, launch5StraightSetings, 2.3, -45, -70)
    await wait(100)
    await gyroSpin(robot, 0)
    await wait(100)
    await robot.rightAttachment.run_time(-50,100)
    await wait(500)
    await robot.rightAttachment.run_time(50,100)
    await gyroStraightTime(robot, launch5StraightSetings, 3, 0, -70)
    await gyroStraightRotations(robot, launch5StraightSetings, 1, 0, 50)
    await gyroSpin(robot, 90)
    await gyroStraightRotations(robot, launch5StraightSetings, 2)

async def sampleExit6(robot: Robot):


    launch6StraightSetings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)



    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)


    await gyroStraightTime(robot , launch6StraightSetings , 2 , 0 , -50)
    await gyroStraightTime(robot , launch6StraightSetings , 2 , 0 , 50)

    # await gyroStraightRotations(robot, launch5StraightSetings, 5.6, 90, 50)
    # await gyroPivot(robot , front , 0)
    # await gyroStraightRotations(robot, launch5StraightSetings, 2.4, 0, 50)
    # await gyroPivot(robot , front , -45)
    # await gyroStraightTime(robot, launch5StraightSetings, 1, -45, 50)
    
    # await robot.rightAttachment.run_time(-1000 , 2000)
    # await wait(2000)
    # await gyroPivot(robot , back , 0)
    # await gyroStraightRotations(robot, launch5StraightSetings, 0.7, 0, -50)

async def sampleExit7(robot: Robot):
    await resetGyro(robot)
    await waitForStart(robot)

    launch7StraightSetings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    await gyroStraightRotations(robot, launch7StraightSetings , 0.2 , -90 , 50)
    await gyroPivot(robot , front , 0 )
    await gyroStraightRotations(robot, launch7StraightSetings , 2.2 , 0 , 100)
    await gyroSpin(robot , 30 )
    robot.leftDrive.run_time(1000 , 2000 , wait=False)
    await robot.rightDrive.run_time(1000 , 2000)
    await robot.rightAttachment.run_time(-1000 , 2000)
    await gyroStraightTime(robot, launch7StraightSetings , 1 , 3 , -100)
    await gyroStraightTime(robot, launch7StraightSetings , 0.5 , 0 , -30)
    await robot.rightAttachment.run_time(1000 , 2000)
    await gyroPivot(robot , front , -47 )
    await gyroStraightRotations(robot, launch7StraightSetings , 3.5 , -47 , 50)
    await gyroSpin(robot , -90 )
    await gyroStraightRotations(robot, launch7StraightSetings , 1.7 , -90 , 50)
    await gyroPivot(robot , front , -135 )
    await gyroStraightTime(robot, launch7StraightSetings , 1 , -135 , 30)
    await robot.rightAttachment.run_time(-1000 , 2000)
    await gyroPivot(robot , back , -90 )
    await gyroStraightRotations(robot, launch7StraightSetings ,1 , -90 , -50)

    # await gyroStraightRotations(robot, launch7StraightSetings, 5.6, 90, 50)
    # await gyroPivot(robot , front , 0)
    # await gyroStraightRotations(robot, launch7StraightSetings, 2.4, 0, 50)
    # await gyroPivot(robot , front , -45)
    # await gyroStraightTime(robot, launch7StraightSetings, 1, -45, 50)
    
    # await robot.rightAttachment.run_time(-1000 , 2000)
    # await wait(2000)
    # await gyroPivot(robot , back , 0)
    # await gyroStraightRotations(robot, launch7StraightSetings, 0.7, 0, -50)

async def leftMotorControl(robot: Robot):
    while True:
        if Button.LEFT in robot.hub.buttons.pressed():
            robot.leftAttachment.run(-3000)
        elif Button.RIGHT in robot.hub.buttons.pressed():
            robot.leftAttachment.run(3000)
        else:
            robot.leftAttachment.stop()
        await wait(10)

async def rightMotorControl(robot: Robot):
    while True:
        if Button.LEFT in robot.hub.buttons.pressed():
            robot.rightAttachment.run(-3000)
        elif Button.RIGHT in robot.hub.buttons.pressed():
            robot.rightAttachment.run(3000)
        else:
            robot.rightAttachment.stop()
        await wait(10)
    
