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
    launch1StraightSettings = LaunchSettings(kp = 3, ki = 0, kd = 0.09)

    launch1StraightSettings2 = LaunchSettings(kp = 3.5, ki = 0, kd = 0.09)
    launch1TurnSettings = LaunchSettings(kp = 4, ki = 3, kd = 0.8, safetyThreshold = 1200, minSpeed=30)
    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)

    exitTimer = StopWatch()

    robot.rightAttachment.dc(30)

    await gyroStraightRotations(robot, launch1StraightSettings, 1.9, 0, 85)
    await gyroStraightTime(robot, launch1StraightSettings, 0.5, 1, 90)

    while(robot.hub.imu.heading() < 0):
        robot.leftDrive.dc(90)
        robot.rightDrive.dc(-90)
    robot.leftDrive.dc(0)
    robot.rightDrive.dc(0)

    
    robot.leftAttachment.run_time(1000 , 1200, wait=False)
    await wait(1000)
    robot.rightAttachment.run_time(-200, 1400, wait=False)
    await wait(500)

    robot.leftAttachment.run_time(-900, 1100)

    await wait(600)

    await gyroStraightTime(robot, launch1StraightSettings, 0.5, 0, -90)
    await gyroStraightTime(robot, launch1StraightSettings, 1.2, 30, -100)


    return exitTimer.time()


async def exit2(robot: Robot):
    launch2StraightSetings = LaunchSettings(kp = 3.5, ki = 0, kd = 0.09)
    launch2TurnSetings =  LaunchSettings(kp = 5, ki = 1.7, kd = 0.15, safetyThreshold = 3500)

    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)
    exitTimer = StopWatch()


    await gyroPivot(robot, launch2TurnSetings, front, -50)
    await gyroStraightRotations(robot, launch2StraightSetings, 1.55, -50, 80)
    await gyroSpin(robot, launch2TurnSetings, 0)

    await gyroStraightTime(robot, launch2StraightSetings, 1.2, 0, 50)
    await wait(50)
    await gyroStraightRotations(robot, launch2StraightSetings, 0.9, 0, -80)

    await gyroPivot(robot, launch2TurnSetings, back, -60, 2)
    await gyroStraightRotations(robot, launch2StraightSetings, 2, -60, -100)

    
    return exitTimer.time()


async def sampleExit3(robot: Robot):
    launch3StraightSettings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    launch3TurnSettings = LaunchSettings(kp = 3, ki = 2.5, kd = 0.25, safetyThreshold = 2000)

    await resetGyro(robot)
    await waitForStart(robot)
    exitTimer = StopWatch()
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)

    await gyroStraightRotations(robot, launch3StraightSettings, 2.3, 60, 80)
    await gyroStraightRotations(robot, launch3StraightSettings, 0.5, 60, 60)
    await gyroPivot(robot, launch3TurnSettings, front, 42)
    await gyroStraightRotations(robot, launch3StraightSettings, 0.25, 42, 40)
    await robot.leftAttachment.run_angle(-300, 140)
    await gyroStraightTime(robot, launch3StraightSettings, 1.3, 42, 60)
    doAlign = await alignToStructure(robot, launch3StraightSettings, front, 42, 4)
    if doAlign == 0:
        await gyroStraightTime(robot, launch3StraightSettings, 0.2, 42, 60)
    # await gyroStraightRotations(robot, launch2StraightSettings, 0.2, 42, -30)
    # await robot.rightAttachment.run_angle(200, 130)

    # robot.rightAttachment.run_time(180, 400, wait=False)
    # await gyroStraightTime(robot, launch2StraightSettings, 0.4, 42, 20)
    robot.rightAttachment.dc(100)
    await gyroStraightTime(robot, launch3StraightSettings, 0.8, 42, 30)
    await wait(400)
    robot.rightAttachment.stop()

    # ridica tridentu din holder
    await robot.rightAttachment.run_time(-500, 1500)

    await gyroStraightRotations(robot, launch3StraightSettings, 0.3, 42, -80)
    # await gyroSpin(robot, launch2TurnSettings, 75)
    await gyroStraightRotations(robot, launch3StraightSettings, 3.5, 75, -100)
    return exitTimer.time()


async def sampleExit4(robot: Robot): #Trage corabia in baza
    launch4StraightSettings = LaunchSettings(kp = 3.5, ki = 0, kd = 0.09)

    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)
    exitTimer = StopWatch()
    robot.hub.imu.reset_heading(0)

    await gyroStraightTime(robot, launch4StraightSettings, 0.8, 0, 80)
    await robot.leftAttachment.run_time(-1000, 700)
    await wait(150)
    await gyroStraightTime(robot, launch4StraightSettings, 1, 0, -100)

    return exitTimer.time()

async def sampleExit5(robot: Robot):
    launch5StraightSettings = LaunchSettings(kp = 3.5, ki = 0, kd = 0.09)
    launch5TurnSettings = LaunchSettings(kp = 8, ki = 3, kd = 0.8, safetyThreshold = 1200, minSpeed=30)
    launch5TurnSettings2 = LaunchSettings(kp = 6, ki = 3, kd = 0.8, safetyThreshold = 1300, minSpeed=30)
    launch5StraightSettings3 = LaunchSettings(kp = 5, ki = 3, kd = 0.8, safetyThreshold = 1200)

    
    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)


    exitTimer = StopWatch()


    robot.leftAttachment.run_time(-200,1000 , wait=False)
    await gyroSpin(robot, launch5TurnSettings, 34)
    await gyroStraightRotations(robot, launch5StraightSettings, 3, 34, 85)
    await gyroPivot(robot, launch5TurnSettings, front, 10)
    # await gyroStraightRotations(robot, launch5StraightSettings, 3.5, 34, 70)
    # await gyroPivot(robot, launch3TurnSettings, front, 0)
    await gyroStraightTime(robot, launch5StraightSettings, 0.7, 10, 70)
    doAlign = await alignToStructure(robot, launch5StraightSettings, front, 0, 2)
    if doAlign == 0:
        await gyroStraightTime(robot, launch5StraightSettings, 0.3, 0, 60)
    robot.leftAttachment.run_time(1000, 700, wait=False)
    await gyroStraightTime(robot, launch5StraightSettings, 1, 0, 100)
    await wait(100)
    
    await robot.leftAttachment.run_time(-100, 500)
    await gyroStraightRotations(robot, launch5StraightSettings, 0.5, 0, -40)

    await gyroPivot(robot, launch5TurnSettings, front , 90)
    robot.leftAttachment.run_time(-200, 1000, wait=False)
    await gyroStraightRotations(robot, launch5StraightSettings, 1.75, 90, 60)
    await gyroSpin(robot, launch5TurnSettings, 181)
    await gyroStraightTime(robot, launch5StraightSettings, 1.5, 181, -40)
    await gyroStraightRotations(robot, launch5StraightSettings, 0.3, 180, 55)
    await gyroPivot(robot, launch5TurnSettings2, front, 275)
    robot.rightAttachment.run_time(500, 1000, wait=False)


    await gyroStraightRotations(robot, launch5StraightSettings, 3.1, 275, -80)
    await gyroSpin(robot, launch5TurnSettings, 260, 3)
    await gyroStraightRotations(robot, launch5StraightSettings, 1.3, 260, -70)
    await wait(50)
    await gyroStraightRotations(robot, launch5StraightSettings, 0.15, 260, 70)
    await wait(50)

    await gyroSpin(robot, launch5StraightSettings3, 370)
    await gyroStraightRotations(robot, launch5StraightSettings, 1.8, 370, -100)
    await gyroPivot(robot, launch5TurnSettings, back, 315, 8)
    await gyroStraightRotations(robot, launch5StraightSettings, 3, 315, -100)
    return exitTimer.time()

async def sampleExit6(robot: Robot):
    launch6StraightSettings = LaunchSettings(kp = 3, ki = 0, kd = 0.09)

    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)

    exitTimer = StopWatch()

    await gyroStraightRotations(robot, launch6StraightSettings, 0.7, 0, -80)
    await gyroStraightRotations(robot, launch6StraightSettings, 0.6, 0, -40)
    await wait(100)
    robot.rightAttachment.dc(-100)
    await wait(200)
    robot.rightAttachment.dc(0)
    await wait(100)
    await gyroStraightRotations(robot, launch6StraightSettings, 0.15, 4, 50)
    await gyroStraightTime(robot, launch6StraightSettings, 0.5, 0, 100)

    await waitForStart(robot)
    robot.hub.imu.reset_heading(0)

    await gyroStraightTime(robot, launch6StraightSettings, 1, 0, 90)
    await gyroStraightTime(robot, launch6StraightSettings, 0.5, 0, 60)
    await wait(100)
    await gyroStraightRotations(robot, launch6StraightSettings, 1.7, 0, -100)
    return exitTimer.time()



async def sampleExit7(robot: Robot): #David Exit / just for orientation
    launch7StraightSettings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    launch7TurnSettings = LaunchSettings(kp = 8, ki = 3.3, kd = 0.9, safetyThreshold = 1200, minSpeed=30)

    await resetGyro(robot)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)
    exitTimer = StopWatch()

    robot.leftAttachment.run_time(-800, 1300, wait=False)
    robot.rightAttachment.dc(30)

    await gyroStraightRotations(robot, launch7StraightSettings, 4.4, -10, 80)
    await gyroStraightTime(robot, launch7StraightSettings, 0.8, -10, 80)

    doAlign = await alignToStructure(robot, launch7StraightSettings, front, 0, 4)
    if doAlign == 0:
        await gyroStraightTime(robot, launch7StraightSettings, 0.3, 0, 60)
    robot.leftAttachment.run_time(1000, 700, wait=False)

    robot.leftDrive.stop()
    
    await gyroStraightTime(robot, launch7StraightSettings, 0.4, 0, 100)

    robot.rightAttachment.stop()
    await robot.leftAttachment.run_time(500, 1500)
    await gyroStraightRotations(robot, launch7StraightSettings, 1.8, 0, -70)
    robot.rightAttachment.run_time(-1000, 700, wait=False)
    await gyroSpin(robot, launch7TurnSettings, 45)
    await gyroStraightRotations(robot, launch7StraightSettings, 0.9, 45, 70)
    await gyroStraightTime(robot, launch7StraightSettings, 0.9, 45, 75)
    await wait(500)

    # robot.leftDrive.run_time(-1000 , 1500 , wait=False)
    # await robot.rightDrive.run_time(-750 , 1500)

    # # await gyroStraightTime(robot, launch7StraightSettings, 1, -40, 90)

    robot.leftDrive.dc(-100)
    robot.rightDrive.dc(-80)
    await wait(600)
    robot.leftDrive.dc(-100)
    robot.rightDrive.dc(-100)
    await wait(700)
    await gyroStraightTime(robot, launch7StraightSettings, 1.3, -45, -100)
    
    

    return exitTimer.time()

async def sampleExit8(robot: Robot): # Corabia in doc
    launch8StraightSettings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    await waitForStart(robot)
    robot.leftAttachment.reset_angle(0)
    robot.rightAttachment.reset_angle(0)
    exitTimer = StopWatch()
    robot.hub.imu.reset_heading(0)

    await gyroStraightTime(robot, launch8StraightSettings , 1.1 , 0 , 50)
    await robot.leftAttachment.run_time(1000, 500)
    await wait(300)
    await gyroStraightTime(robot, launch8StraightSettings , 1 , 0 , -100)

    return exitTimer.time()

# async def sampleExit9(robot: Robot): #Corali
#     launch9StraightSettings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)

#     await waitForStart(robot)
#     robot.leftAttachment.reset_angle(0)
#     robot.rightAttachment.reset_angle(0)
#     exitTimer = StopWatch()

#     robot.hub.imu.reset_heading(0)
#     await gyroStraightTime(robot, launch9StraightSettings , 0.7 , 0 , 30)
#     await wait(100)
#     await gyroStraightTime(robot, launch9StraightSettings , 0.5 , 0 , -100)

#     return exitTimer.time()

async def sampleExit9(robot: Robot): #Iesire johnny
    launch9StraightSetings = LaunchSettings(kp = 2, ki = 0, kd = 0.09)
    launch10TurnSetings =  LaunchSettings(kp = 6, ki = 1.7, kd = 0.15, safetyThreshold = 3500)

    await resetGyro(robot)
    await waitForStart(robot)
    exitTimer = StopWatch()

    await gyroStraightRotations(robot, launch9StraightSetings, 2, 0, 100)
    robot.leftAttachment.dc(100)
    await wait(300)
    robot.leftAttachment.dc(0)
    await gyroStraightRotations(robot, launch9StraightSetings, 3.5, 2, 100)
    await gyroStraightRotations(robot, launch9StraightSetings, 1, 0, 100)
    await gyroStraightRotations(robot, launch9StraightSetings, 1.5, -15, 100)

    # await gyroSpin(robot , launch10TurnSetings , -20)
    # await gyroStraightRotations(robot, launch10StraightSetings, 2.2, -20, -100)
    # await gyroSpin(robot , launch10TurnSetings , -90)
    # await gyroStraightRotations(robot, launch10StraightSetings, 3.2, -90, -100)
    # await gyroPivot(robot , launch10TurnSetings ,front , -135)
    # robot.rightAttachment.dc(-100)
    # await wait(5000)
    # robot.rightAttachment.dc(0)


    return exitTimer.time()

async def sampleExit11(robot: Robot): #last exit

    await resetGyro(robot)
    await waitForStart(robot)
    launch11StraightSetings = LaunchSettings(kp = 3.5, ki = 0, kd = 0.09)
    launch11TurnSetings = LaunchSettings(kp = 6, ki = 3, kd = 0.8, safetyThreshold = 1300)
    launch11TurnSetings2 =  LaunchSettings(kp = 6, ki = 3, kd = 0.8, safetyThreshold = 2300, minSpeed=40)
    launch11TurnSettings3 = LaunchSettings(kp = 8, ki = 3, kd = 0.8, safetyThreshold = 1500, minSpeed=40)
    exitTimer = StopWatch()

    await gyroStraightRotations(robot, launch11StraightSetings, 5.45, -19, 100)
    await gyroSpin(robot, launch11TurnSetings, 90)
    await gyroStraightRotations(robot, launch11StraightSetings, 2.7, 90, -100)
    await gyroPivot(robot, launch11TurnSetings, back, 45)

    await gyroStraightTime(robot, launch11StraightSetings , 0.8 , 45 , -90)
    robot.leftDrive.dc(-40)
    robot.rightDrive.dc(-40)
    await robot.rightAttachment.run_time(-1000, 700)
    await wait(600)
    robot.leftDrive.dc(0)
    robot.rightDrive.dc(0)
    await gyroStraightTime(robot, launch11StraightSetings , 0.1 , 45 , 80)



    return exitTimer.time()

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
    

async def gyroTest(robot: Robot):
    await resetGyro(robot)
    while True:
        robot.hub.display.number(abs(robot.hub.imu.heading()))
        await wait(10)