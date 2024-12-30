from pybricks.hubs import PrimeHub # type: ignore
from pybricks.pupdevices import Motor, ColorSensor # type: ignore
from pybricks.parameters import Button, Color # type: ignore
from pybricks.robotics import DriveBase # type: ignore
from pybricks.tools import wait, StopWatch # type: ignore

class Robot:
    def __init__(self, hub: PrimeHub, leftDrive: Motor, rightDrive: Motor, leftAttachment: Motor, rightAttachment: Motor, driveBase: DriveBase, colorLeft: ColorSensor, colorRight: ColorSensor):
        self.hub = hub
        self.leftDrive = leftDrive
        self.rightDrive = rightDrive
        self.leftAttachment = leftAttachment
        self.rightAttachment = rightAttachment
        self.driveBase = driveBase
        self.colorLeft = colorLeft
        self.colorRight = colorRight

class PIDController:
    integralError = 0
    lastError = 0
    derivativeError = 0

    def __init__(self, setPoint: float, targetTolerance: float, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setPoint = setPoint
        self.timer = StopWatch()
        self.lastTime = self.timer.time()
        self.targetTolerance = targetTolerance

    def calculate(self, measuredValue: float):
        error = self.setPoint - measuredValue
        
        timePassed = self.timer.time() - self.lastTime
        timePassed /= 1000
        self.lastTime = self.timer.time()

        if (self.lastError >= 0 and error < 0) or (self.lastError <= 0 and error > 0):
            self.integralError = 0

        if abs(error) < self.targetTolerance:
            self.integralError = 0
        else:
            self.integralError += timePassed * error

        if timePassed != 0:
            self.derivativeError = (error - self.lastError) / timePassed
        self.lastError = error

        correction = error * self.kp + self.integralError * self.ki + self.derivativeError * self.kd

        return correction

class LaunchSettings:
    def __init__(self, kp: float, ki: float, kd: float, safetyThreshold: float = 3500, turnTolerance: float = 0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.safetyThreshold = safetyThreshold
        self.turnTolerance = turnTolerance

async def gyroStraightRotations(robot: Robot, launchSettings: LaunchSettings, rotations: float, targetHeading: float = 0, targetPower: float = 60, accelDist: float = 0, deaccelDist: float = 0, targetTolerance:float = 0):
    robot.leftDrive.reset_angle(0);
    robot.rightDrive.reset_angle(0);

    kp = launchSettings.kp
    ki = launchSettings.ki
    kd = launchSettings.kd

    controller = PIDController(targetHeading, targetTolerance, kp, ki, kd)

    print("Starting gyro straight rotations")

    loopCount = 0
    totalError = 0
    while abs(min(robot.leftDrive.angle(), robot.rightDrive.angle())) / 360 < rotations:
        correction = controller.calculate(robot.hub.imu.heading())

        robot.leftDrive.dc(targetPower + correction)
        robot.rightDrive.dc(targetPower - correction + 3)

        loopCount += 1
        totalError += abs(robot.hub.imu.heading() - targetHeading)
        await wait(10)

    if loopCount > 0:
        print(f"Completed gyro straight rotations with average error {totalError / loopCount}")
    else:
        print("Gyro straight rotations hasn't entered the while loop")
        
    robot.leftDrive.hold()
    robot.rightDrive.hold()

async def gyroStraightTime(robot: Robot, launchSettings: LaunchSettings, seconds: float, targetHeading: float = 0, targetPower: float = 60, accelDist: float = 0, deaccelDist: float = 0, targetTolerance:float = 0):
    robot.leftDrive.reset_angle(0);
    robot.rightDrive.reset_angle(0);
    timer = StopWatch()

    kp = launchSettings.kp
    ki = launchSettings.ki
    kd = launchSettings.kd

    controller = PIDController(targetHeading, targetTolerance, kp, ki, kd)

    print("Starting gyro straight time")

    loopCount = 0
    totalError = 0
    while timer.time() < seconds * 1000:
        correction = controller.calculate(robot.hub.imu.heading())

        robot.leftDrive.dc(targetPower + correction)
        robot.rightDrive.dc(targetPower - correction + 3)

        loopCount += 1
        totalError += abs(robot.hub.imu.heading() - targetHeading)
        await wait(10)

    if loopCount > 0:
        print(f"Completed gyro straight time with average error {totalError / loopCount}")
    else:
        print("Gyro straight time hasn't entered the while loop")
        
    robot.leftDrive.hold()
    robot.rightDrive.hold()

async def resetGyro(robot: Robot):
    robot.hub.light.on(Color.RED)
    leftButtonPressed = False
    clicked = False
    while clicked == False:
        if Button.LEFT in robot.hub.buttons.pressed():
            leftButtonPressed = True

        if (Button.LEFT not in robot.hub.buttons.pressed()) and leftButtonPressed == True:
            robot.hub.imu.reset_heading(0)
            robot.hub.light.on(Color.GREEN)
            clicked = True
            leftButtonPressed = False

        await wait(10)
    print("Gyro has been reset")

async def waitForStart(robot: Robot):
    leftButtonPressed = False
    clicked = False
    while clicked == False:
        if Button.RIGHT in robot.hub.buttons.pressed():
            leftButtonPressed = True

        if (Button.RIGHT not in robot.hub.buttons.pressed()) and leftButtonPressed == True:
            clicked = True
            leftButtonPressed = False
        await wait(10)
    robot.hub.light.on(Color.VIOLET)
    print("Exit has started")

async def alignToStructure(robot: Robot, launchSettings: LaunchSettings, direction: float, targetHeading: float, targetTolerance: float, speed: float = 1000):
    stopWatch = StopWatch()
    error = abs(robot.hub.imu.heading() - targetHeading)
    
    if(abs(robot.hub.imu.heading()) < abs(targetHeading) - targetTolerance):
        while(error > 1):
            error = abs(robot.hub.imu.heading() - targetHeading)
            robot.leftDrive.run(speed)
            robot.rightDrive.run(-speed)
        robot.leftDrive.hold()
        robot.rightDrive.hold()
        if(direction == 0):
            await gyroStraightTime(robot, launchSettings, 1, targetHeading, 60)
        elif(direction == 1):
            await gyroStraightTime(robot, launchSettings, 1, targetHeading, -60)

        return 1

    elif(abs(robot.hub.imu.heading()) > abs(targetHeading) + targetTolerance):
        while(error > 1):
            error = abs(robot.hub.imu.heading() - targetHeading)
            robot.leftDrive.run(-speed)
            robot.rightDrive.run(speed)
        robot.leftDrive.hold()
        robot.rightDrive.hold()
        if(direction == 0):
            await gyroStraightTime(robot, launchSettings, 1, targetHeading, 60)
        elif(direction == 1):
            await gyroStraightTime(robot, launchSettings, 1, targetHeading, -60)

        return 1
    else:
        return 0
        pass

    robot.leftDrive.hold()
    robot.rightDrive.hold()


async def gyroPivot(robot: Robot, direction: float, targetHeading:float, maxSpeed: float = 2400, minSpeed: float = 70):
    turnDegrees = abs(robot.hub.imu.heading() - targetHeading)
    turnRot = turnDegrees / 360

    safetyWatch = StopWatch()

    if (robot.hub.imu.heading() < targetHeading):
        while robot.hub.imu.heading() < targetHeading:
            currentSpeed = maxSpeed * abs(robot.hub.imu.heading() - targetHeading) / turnDegrees * turnRot

            if(currentSpeed < minSpeed):
                currentSpeed = minSpeed
            
            if (direction == 0):
                robot.leftDrive.run(currentSpeed)
                robot.rightDrive.run(0)
            else:
                robot.leftDrive.run(0)
                robot.rightDrive.run(-currentSpeed)

            await wait(10)
    else:
        while robot.hub.imu.heading() > targetHeading:
            currentSpeed = maxSpeed * abs(robot.hub.imu.heading() - targetHeading) / turnDegrees * turnRot

            if(currentSpeed < minSpeed):
                currentSpeed = minSpeed
            
            if (direction == 0):
                robot.leftDrive.run(0)
                robot.rightDrive.run(currentSpeed)
            else:
                robot.leftDrive.run(-currentSpeed)
                robot.rightDrive.run(0)

            await wait(10)

    robot.leftDrive.hold()
    robot.rightDrive.hold()

    finalError = abs(robot.hub.imu.heading() - targetHeading)
    print(f"Gyro pivot took {safetyWatch.time()}ms to complete, finished with {finalError} degrees error")

async def gyroSpin(robot: Robot, targetHeading: float, maxSpeed: float = 1200, minSpeed: float = 35):
    turnDegrees = abs(robot.hub.imu.heading() - targetHeading)
    turnRot = turnDegrees / 360

    if (robot.hub.imu.heading() < targetHeading):
        while robot.hub.imu.heading() < targetHeading:
            currentSpeed = maxSpeed * abs(robot.hub.imu.heading() - targetHeading) / turnDegrees * turnRot

            if(currentSpeed < minSpeed):
                currentSpeed = minSpeed
            
            robot.leftDrive.run(currentSpeed)
            robot.rightDrive.run(-currentSpeed)

            await wait(10)
    else:
        while robot.hub.imu.heading() > targetHeading:
            currentSpeed = maxSpeed * abs(robot.hub.imu.heading() - targetHeading) / turnDegrees * turnRot

            if(currentSpeed < minSpeed):
                currentSpeed = minSpeed
            
            robot.leftDrive.run(-currentSpeed)
            robot.rightDrive.run(currentSpeed)

            await wait(10)

    robot.leftDrive.hold()
    robot.rightDrive.hold()

async def archiveSpin(robot: Robot, launchSettings: LaunchSettings, targetHeading: float):
    lockGate = False

    stopWatch = StopWatch()
    safetyWatch = StopWatch()
    
    targetTolerance = launchSettings.turnTolerance

    kp = launchSettings.kp
    ki = launchSettings.ki
    kd = launchSettings.kd
    safetyThreshold = launchSettings.safetyThreshold

    controller = PIDController(targetHeading, targetTolerance, kp, ki, kd)

    currentHeading = robot.hub.imu.heading()
    currentError = abs(targetHeading - currentHeading)

    print("Starting gyro spin")
    while currentError > targetTolerance or (currentError < targetTolerance and stopWatch.time() < 500):
        currentHeading = robot.hub.imu.heading()
        currentError = abs(targetHeading - currentHeading)

        turnRate = controller.calculate(currentHeading)

        robot.leftDrive.run(turnRate)
        robot.rightDrive.run(-turnRate)

        if currentError < targetTolerance and lockGate == False: 
            stopWatch.reset()
            stopWatch.resume()
            lockGate = True
        elif currentError > targetTolerance: 
            stopWatch.pause()
            lockGate = False

        if safetyWatch.time() > safetyThreshold:
            break
        
        await wait(10)

    finalError = abs(robot.hub.imu.heading() - targetHeading)
    if safetyWatch.time() > safetyThreshold:
        print(f"Gyro spin took more than {safetyThreshold}ms, finished with {finalError} degrees error, moving on")
    else:
        print(f"Gyro spin took {safetyWatch.time()}ms to complete, finished with {finalError} degrees error")

    robot.leftDrive.hold()
    robot.rightDrive.hold()

async def archivePivot(robot: Robot, launchSettings: LaunchSettings, direction: float, targetHeading: float):

    lockGate = False

    stopWatch = StopWatch()
    safetyWatch = StopWatch()

    targetTolerance = launchSettings.turnTolerance

    kp = launchSettings.kp * 2
    ki = launchSettings.ki * 2
    kd = launchSettings.kd * 2
    safetyThreshold = launchSettings.safetyThreshold

    controller = PIDController(targetHeading, targetTolerance, kp, ki, kd)

    currentHeading = robot.hub.imu.heading()
    startingHeading = currentHeading
    currentError = abs(targetHeading - currentHeading)

    print("Starting gyro pivot")

    #0 front
    if direction == 0:
        while currentError > targetTolerance or (currentError < targetTolerance and stopWatch.time() < 500):
            currentHeading = robot.hub.imu.heading()
            currentError = abs(targetHeading - currentHeading)

            turnRate = controller.calculate(currentHeading)

            if(targetHeading < startingHeading):
                robot.rightDrive.run(-turnRate)
            else:
                robot.leftDrive.run(turnRate)

            if currentError < targetTolerance and lockGate == False: 
                stopWatch.reset()
                stopWatch.resume()
                lockGate = True
            elif currentError > targetTolerance: 
                stopWatch.pause()
                lockGate = False

            if safetyWatch.time() > safetyThreshold:
                break
            await wait(10)

        finalError = abs(robot.hub.imu.heading() - targetHeading)
        if safetyWatch.time() > safetyThreshold:
            print(f"Gyro pivot took more than {safetyThreshold}ms, finished with {finalError} degrees error, moving on")
        else:
            print(f"Gyro pivot took {safetyWatch.time()}ms to complete, finished with {finalError} degrees error")

        robot.leftDrive.hold()
        robot.rightDrive.hold()
    #1 back
    elif direction == 1:
        while currentError > targetTolerance or (currentError < targetTolerance and stopWatch.time() < 500):
            currentHeading = robot.hub.imu.heading()
            currentError = abs(targetHeading - currentHeading)

            turnRate = controller.calculate(currentHeading)

            if(targetHeading > startingHeading):
                robot.rightDrive.run(-turnRate)
            else:
                robot.leftDrive.run(turnRate)

            if currentError < targetTolerance and lockGate == False: 
                stopWatch.reset()
                stopWatch.resume()
                lockGate = True
            elif currentError > targetTolerance: 
                stopWatch.pause()
                lockGate = False

            if safetyWatch.time() > safetyThreshold:
                break
            await wait(10)

        robot.leftDrive.hold()
        robot.rightDrive.hold()

        finalError = abs(robot.hub.imu.heading() - targetHeading)
        if safetyWatch.time() > safetyThreshold:
            print(f"Gyro pivot took more than {safetyThreshold}ms, finished with {finalError} degrees error moving on")
        else:
            print(f"Gyro pivot took {safetyWatch.time()}ms to complete, finished with {finalError} degrees error")
    else:
        print("WARN: GYRO PIVOT DIRECTION INVALID")


async def gyroSpinFS(robot: Robot, targetHeading: float, maxSpeed: float = 1200, minSpeed: float = 35):
    launchFSStraightSettings = LaunchSettings(kp = 4, ki = 0, kd = 0.09)
    turnDegrees = abs(robot.hub.imu.heading() - targetHeading)
    turnRot = turnDegrees / 360

    if (robot.hub.imu.heading() < targetHeading):
        exit = 0
        pd = robot.hub.imu.heading()
        while robot.hub.imu.heading() < targetHeading and exit == 0:
            pd = robot.hub.imu.heading()
            currentSpeed = maxSpeed * abs(robot.hub.imu.heading() - targetHeading) / turnDegrees * turnRot

            if(currentSpeed < minSpeed):
                currentSpeed = minSpeed
            
            robot.leftDrive.run(currentSpeed)
            robot.rightDrive.run(-currentSpeed)

            if(pd == robot.hub.imu.heading()):
                print("Failed Spining")
                exit = 1
                cdg = pd - 10

            await wait(10)
        
        if(exit == 1):
            print("Failed Spining. Initiating corection")
            await wait(2000)
            gyroSpin(robot, cdg)
            await wait(2000)
            gyroStraightRotations(robot, launchFSStraightSettings, 0.15, cdg, -30)
            await wait(2000)
            gyroSpin(robot, targetHeading)
            await wait(2000)

    else:
        while robot.hub.imu.heading() > targetHeading:
            currentSpeed = maxSpeed * abs(robot.hub.imu.heading() - targetHeading) / turnDegrees * turnRot

            if(currentSpeed < minSpeed):
                currentSpeed = minSpeed
            
            robot.leftDrive.run(-currentSpeed)
            robot.rightDrive.run(currentSpeed)

            await wait(10)

    robot.leftDrive.hold()
    robot.rightDrive.hold()