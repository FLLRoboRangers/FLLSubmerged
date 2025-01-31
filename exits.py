from pybricks.hubs import PrimeHub # type: ignore
from pybricks.pupdevices import Motor, ColorSensor # type: ignore
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis # type: ignore
from pybricks.robotics import DriveBase # type: ignore
from pybricks.tools import wait, StopWatch # type: ignore
from util import Robot, LaunchSettings, gyroStraightRotations, play_song, gyroSpin, gyroStraightTime, gyroPivot, resetGyro, waitForStart, alignToStructure


front = 0 #constant used to indicate pivot direction
back = 1 #constant used to indicate pivot direction

#default values for PID constants
#GyroStraight : kp = 4, ki = 0, kd = 0.09
#GyroTurn : kp = 6, ki = 1.7, kd = 0.15, safetyThreshold = 3500

async def exit1(robot: Robot):
    launch1StraightSettings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    launch1TurnSettings = LaunchSettings(kp = 6, ki = 1.7, kd = 0.15, safetyThreshold = 2000, turnTolerance = 1)
    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)
    exitTimer = StopWatch()
   
    
    robot.leftAttachment.run_angle(1000 , 900, wait = False)
    robot.rightAttachment.run_time(100 , 200 , wait = False)


    await gyroPivot(robot, launch1TurnSettings, front, 40)
    await gyroStraightRotations(robot, launch1StraightSettings, 2.1, 40, 80)
    await gyroPivot(robot, launch1TurnSettings, front, 25)
    #await gyroStraightRotations(robot, launch1StraightSettings, 0.1, 25, 40)
    await robot.leftAttachment.run_time(-1000, 1550)
    await gyroPivot(robot, launch1TurnSettings, front, -5)
    
    await gyroStraightRotations(robot, launch1StraightSettings, 0.5, -5, 50)
    # await gyroPivot(robot, front, 90)
    await gyroSpin(robot, launch1TurnSettings, 91)
    await gyroStraightTime(robot, launch1StraightSettings, 2.4, 92, -30)
    doAlign = await alignToStructure(robot, launch1StraightSettings, back, 90, 2)
    if doAlign == 0:
        await gyroStraightTime(robot, launch1StraightSettings, 0.5, 90, -60)
    await robot.rightAttachment.run_time(-500 , 750)
    await wait(100)
    await robot.rightAttachment.run_time(200 , 300)
    await wait(100)
    await gyroStraightTime(robot, launch1StraightSettings, 0.1, 90, 50)
    await wait(100)
    await robot.rightAttachment.run_time(400 , 500)

    #corabie
    await gyroStraightTime(robot, launch1StraightSettings, 1.5, 89, 50)
    await gyroStraightTime(robot, launch1StraightSettings, 1.5, 90, -30)
    
    await gyroPivot(robot, launch1TurnSettings, back, 40, 4)
    await gyroStraightTime(robot, launch1StraightSettings, 2, 40, -100)

    return exitTimer.time()


async def exit2(robot: Robot):
    launch2StraightSettings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    launch2TurnSettings = LaunchSettings(kp = 3, ki = 2.5, kd = 0.25, safetyThreshold = 3000)

    await resetGyro(robot)
    await waitForStart(robot)
    exitTimer = StopWatch()
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)

    await gyroStraightRotations(robot, launch2StraightSettings, 1.3, 60, 80)
    await gyroStraightRotations(robot, launch2StraightSettings, 1.55, 60, 60)
    await gyroPivot(robot, launch2TurnSettings, front, 42)
    await gyroStraightRotations(robot, launch2StraightSettings, 0.25, 42, 25)
    await robot.leftAttachment.run_angle(-300, 140)
    await gyroStraightTime(robot, launch2StraightSettings, 1.3, 42, 60)
    await alignToStructure(robot, launch2StraightSettings, front, 42, 3)
    await gyroStraightRotations(robot, launch2StraightSettings, 0.2, 42, -23)
    await robot.rightAttachment.run_angle(200, 170)

    # robot.rightAttachment.run_time(180, 3200, wait=False)
    robot.rightAttachment.run_time(180, 500, wait=False)
    await gyroStraightTime(robot, launch2StraightSettings, 0.4, 42, 20)
    await wait(100)
    robot.rightAttachment.dc(100)
    await gyroStraightTime(robot, launch2StraightSettings, 0.9, 42, 30)
    robot.rightAttachment.stop()

    # ridica tridentu din holder
    robot.rightAttachment.run_time(-300, 3000, wait=False)
    await wait(100)
    await gyroStraightRotations(robot, launch2StraightSettings, 0.08, 42, -30)
    await wait(1600)

    await gyroStraightRotations(robot, launch2StraightSettings, 0.2, 42, -80)
    # await gyroSpin(robot, launch2TurnSettings, 75)
    await gyroStraightRotations(robot, launch2StraightSettings, 3, 75, -100)
    return exitTimer.time()


async def sampleExit3(robot: Robot):
    launch3StraightSettings = LaunchSettings(kp = 3.5, ki = 0, kd = 0.09)
    launch3TurnSettings = LaunchSettings(kp = 2.5, ki = 2.5, kd = 0.25, safetyThreshold = 3000)
    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)


    exitTimer = StopWatch()


    robot.leftAttachment.run_time(-200,1000 , wait=False)
    await gyroSpin(robot, launch3TurnSettings, 34)
    await gyroStraightRotations(robot, launch3StraightSettings, 3.5, 34, 70)
    await gyroPivot(robot, launch3TurnSettings, front, 0)
    await gyroStraightTime(robot, launch3StraightSettings, 1, 0, 50)
    robot.leftAttachment.run_time(1000, 700, wait=False)
    await gyroStraightTime(robot, launch3StraightSettings, 0.8, 0, 90)
    await wait(200)
    
    await robot.leftAttachment.run_time(-100, 500)
    await gyroStraightRotations(robot, launch3StraightSettings, 0.5, 0, -35)

    await gyroPivot(robot, launch3TurnSettings, front , 90)
    await gyroStraightRotations(robot, launch3StraightSettings, 1.8, 90, 60)
    await gyroSpin(robot, launch3TurnSettings, 181)
    await gyroStraightTime(robot, launch3StraightSettings, 3, 181, -30)
    await gyroStraightRotations(robot, launch3StraightSettings, 0.4, 180, 50)
    await gyroSpin(robot, launch3TurnSettings, 280)
    robot.rightAttachment.run_time(500, 1000, wait=False)

    robot.leftAttachment.run_time(-200, 1000, wait=False)
    await gyroStraightRotations(robot, launch3StraightSettings, 3.05, 280, -60)
    await gyroSpin(robot, launch3TurnSettings, 255)
    await gyroStraightRotations(robot, launch3StraightSettings, 1.2, 255, -40)
    await wait(100)
    await gyroStraightRotations(robot, launch3StraightSettings, 0.55, 255, 40)
    
    # await gyroSpin(robot, 275)
    # await gyroStraightRotations(robot, launch3StraightSettings, 0.7, 275, -40)
    # await wait(500)
    # await gyroStraightRotations(robot, launch3StraightSettings, 0.7, 275, 50)
    
    await gyroSpin(robot, launch3TurnSettings, 360)
    await gyroStraightRotations(robot, launch3StraightSettings, 1.8, 360, -90)
    await gyroStraightRotations(robot, launch3StraightSettings, 3, 315, -100)
    return exitTimer.time()




async def sampleExit4(robot: Robot):
    launch4StraightSettings = LaunchSettings(kp = 3, ki = 0, kd = 0.09)
    launch4TurnSettings = LaunchSettings(kp = 2.5, ki = 2.5, kd = 0.25, safetyThreshold = 3000)

    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)

    exitTimer = StopWatch()

    await gyroStraightRotations(robot, launch4StraightSettings, 0.7, 0, -80)
    await gyroStraightRotations(robot, launch4StraightSettings, 0.6, 0, -40)
    await wait(100)
    robot.rightAttachment.dc(-100)
    await wait(200)
    robot.rightAttachment.dc(0)
    await wait(100)
    await gyroStraightRotations(robot, launch4StraightSettings, 0.15, 4, 50)
    await gyroStraightTime(robot, launch4StraightSettings, 0.5, 0, 100)

    await waitForStart(robot)
    robot.hub.imu.reset_heading(0)

    await gyroStraightTime(robot, launch4StraightSettings, 1.5, 0, 80)
    await wait(100)
    await gyroStraightRotations(robot, launch4StraightSettings, 1.7, 0, -100)
    return exitTimer.time()



async def sampleExit5(robot: Robot):
    launch5StraightSettings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    launch5TurnSettings = LaunchSettings(kp = 2.5, ki = 2.5, kd = 0.25, safetyThreshold = 3000)

    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)
    exitTimer = StopWatch()

    robot.leftAttachment.run_time(-800, 1300, wait=False)

    await gyroStraightRotations(robot, launch5StraightSettings, 3.8, 0, 60)
    await gyroSpin(robot, launch5TurnSettings, -25)
    await gyroStraightRotations(robot, launch5StraightSettings, 0.7, -25, 50)
    await gyroPivot(robot, launch5TurnSettings, front, 0)

    # fail = await gyroPivotFS(robot, front, 0, 2400, 200, 2000)
    # if(fail == True):
    #     await gyroPivot(robot, front, 0, 2400, 500)


    await gyroStraightTime(robot, launch5StraightSettings, 1.5, 0, 60)
    await robot.leftAttachment.run_time(500, 1700)
    await gyroStraightRotations(robot, launch5StraightSettings, 1.6, 0, -40)
    await robot.rightAttachment.run_time(1000, 650)
    await gyroSpin(robot, launch5TurnSettings, 45)
    await gyroStraightRotations(robot, launch5StraightSettings, 2, 45, 70)
    await gyroStraightTime(robot, launch5StraightSettings, 1, 45, 70)
    await wait(500)
    await gyroStraightRotations(robot, launch5StraightSettings, 1, 45, -70)
    await gyroSpin(robot, launch5TurnSettings, 0)
    await gyroStraightRotations(robot, launch5StraightSettings, 1.5, 0, -100)
    await gyroSpin(robot, launch5TurnSettings, -45)
    await gyroStraightRotations(robot, launch5StraightSettings, 2.5, -45, -100)


    await waitForStart(robot)

    robot.hub.imu.reset_heading(0)
    await gyroStraightTime(robot, launch5StraightSettings , 1 , 0 , 20)
    await wait(100)
    await gyroStraightTime(robot, launch5StraightSettings , 0.5 , 0 , -100)
    return exitTimer.time()

async def sampleExit6(robot: Robot):
    launch6StraightSetings = LaunchSettings(kp = 3.5, ki = 0, kd = 0.09)
    launch6TurnSetings =  LaunchSettings(kp = 6, ki = 1.7, kd = 0.15, safetyThreshold = 3500)

    await resetGyro(robot)
    await waitForStart(robot)
    exitTimer = StopWatch()


    await gyroStraightRotations(robot, launch6StraightSetings, 2.4, 0, 50)
    robot.leftAttachment.dc(100)
    await wait(300)
    robot.leftAttachment.dc(0)
    await gyroStraightRotations(robot, launch6StraightSetings, 3.5, -2, 60)
    await gyroStraightRotations(robot, launch6StraightSetings, 3, -15, 100)
    return exitTimer.time()


async def sampleExit7(robot: Robot):
    launch7StraightSetings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    launch7TurnSetings =  LaunchSettings(kp = 6, ki = 1.7, kd = 0.15, safetyThreshold = 3500)

    await resetGyro(robot)
    await waitForStart(robot)

    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)
    exitTimer = StopWatch()


    await gyroStraightTime(robot, launch7StraightSetings , 0.8 , 1 , -100)
    await robot.leftAttachment.run_time(-700, 500)
    await gyroStraightTime(robot , launch7StraightSetings , 0.6 , 0 , 100)
    return exitTimer.time()


async def sampleExit8(robot: Robot):    

    await waitForStart(robot)
    launch8StraightSetings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    launch8TurnSetings =  LaunchSettings(kp = 6, ki = 1.7, kd = 0.15, safetyThreshold = 3500)

    exitTimer = StopWatch()

    await gyroStraightRotations(robot, launch8StraightSetings , 1.2 , 170, 100)
    await gyroStraightRotations(robot, launch8StraightSetings , 1 , 180, 100)
    await gyroSpin(robot, launch8TurnSetings, 210)


    robot.leftDrive.run_time(1000 , 2000 , wait=False)
    await robot.rightDrive.run_time(1000 , 2000)
    await robot.rightAttachment.run_time(-1000 , 2000)
    await gyroStraightTime(robot, launch8StraightSetings , 1 , 179 , -80)
    await gyroStraightTime(robot, launch8StraightSetings , 0.7 , 180 , -70)
    robot.rightAttachment.run_time(1000 , 2000, wait=False)
    await wait(100)
    await gyroStraightRotations(robot, launch8StraightSetings , 0.1 , 180 , 70)
    await gyroPivot(robot, launch8TurnSetings, front , 125, 3000, 500)
    await gyroStraightRotations(robot, launch8StraightSetings , 1.6 , 125 , 60)
    await gyroPivot(robot , launch8TurnSetings, front , 180)
    await gyroStraightRotations(robot, launch8StraightSetings , 1 , 180 , 60)
    await gyroSpin(robot , launch8TurnSetings, 90 )
    await gyroStraightRotations(robot, launch8StraightSetings , 1.15 , 90 , 60)

    
    await gyroStraightRotations(robot, launch8StraightSetings , 1.7 , 90 , 60)
    await gyroPivot(robot , launch8TurnSetings, front , 45 )
    await gyroStraightTime(robot, launch8StraightSetings , 1 , 45 , 30)
    await robot.rightAttachment.run_time(-1000 , 2000)
    await gyroStraightRotations(robot, launch8StraightSetings , 0.2 , 45 , -60)
    await gyroPivot(robot , launch8TurnSetings, back , 90 )
    await gyroStraightRotations(robot, launch8StraightSetings , 0.8 , 90 , -70)
    
    return exitTimer.time()
    # await play_song(robot, 100)

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
    