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
    def __init__(self, kp: float, ki: float, kd: float, safetyThreshold: float = 3500, turnTolerance: float = 0.5, minSpeed: float = 10):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.safetyThreshold = safetyThreshold
        self.turnTolerance = turnTolerance
        self.minSpeed = minSpeed

async def gyroStraightRotations(robot: Robot, launchSettings: LaunchSettings, rotations: float, targetHeading: float = 0, targetPower: float = 60, accelDist: float = 0, deaccelDist: float = 0, targetTolerance:float = 0):
    robot.leftDrive.reset_angle(0)
    robot.rightDrive.reset_angle(0)

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
    robot.leftDrive.reset_angle(0)
    robot.rightDrive.reset_angle(0)
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



async def alignToStructure(robot: Robot, launchSettings: LaunchSettings, direction: float, targetHeading: float, targetTolerance: float, speed: float = 1000, safetyThreshold: float = 2000):
    stopWatch = StopWatch()
    
    error = abs(robot.hub.imu.heading() - targetHeading)

    if robot.hub.imu.heading() < targetHeading - targetTolerance:
        while error > 1 and stopWatch.time() < safetyThreshold:  # Timeout to prevent infinite loop
            error = abs(robot.hub.imu.heading() - targetHeading)
            speedFactor = max(0.3, error / 20)  # Reduce speed as it gets closer
            robot.leftDrive.run(speed * speedFactor)
            robot.rightDrive.run(-speed * speedFactor)
        robot.leftDrive.hold()
        robot.rightDrive.hold()

        if direction == 0:
            await gyroStraightTime(robot, launchSettings, 1, targetHeading, 60)
        elif direction == 1:
            await gyroStraightTime(robot, launchSettings, 1, targetHeading, -60)

        return 1

    elif robot.hub.imu.heading() > targetHeading + targetTolerance:
        while error > 1 and stopWatch.time() < safetyThreshold:
            error = abs(robot.hub.imu.heading() - targetHeading)
            speedFactor = max(0.3, error / 20)
            robot.leftDrive.run(-speed * speedFactor)
            robot.rightDrive.run(speed * speedFactor)
        robot.leftDrive.hold()
        robot.rightDrive.hold()

        if direction == 0:
            await gyroStraightTime(robot, launchSettings, 1, targetHeading, 60)
        elif direction == 1:
            await gyroStraightTime(robot, launchSettings, 1, targetHeading, -60)

        return 1

    else:
        return 0

async def gyroSpin(robot: Robot, launchSettings: LaunchSettings, targetHeading: float, turnTolerance: float = 1):
    lockGate = False

    stopWatch = StopWatch()
    safetyWatch = StopWatch()

    targetTolerance = turnTolerance

    kp = launchSettings.kp
    ki = launchSettings.ki
    kd = launchSettings.kd
    safetyThreshold = launchSettings.safetyThreshold
    minSpeed = launchSettings.minSpeed

    controller = PIDController(targetHeading, targetTolerance, kp, ki, kd)

    currentHeading = robot.hub.imu.heading()
    currentError = abs(targetHeading - currentHeading)

    print("Starting gyro spin")
    while currentError > targetTolerance or (currentError < targetTolerance and stopWatch.time() < 500):
        currentHeading = robot.hub.imu.heading()
        currentError = abs(targetHeading - currentHeading)

        turnRate = controller.calculate(currentHeading)

        if(abs(turnRate) < minSpeed and turnRate > 0):
            turnRate = minSpeed
        elif (abs(turnRate) < minSpeed and turnRate < 0):
            turnRate = -minSpeed

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

async def gyroPivot(robot: Robot, launchSettings: LaunchSettings, direction: float, targetHeading: float, turnTolerance: float = 1):

    lockGate = False

    stopWatch = StopWatch()
    safetyWatch = StopWatch()

    targetTolerance = turnTolerance

    kp = launchSettings.kp * 2
    ki = launchSettings.ki * 2
    kd = launchSettings.kd * 2
    safetyThreshold = launchSettings.safetyThreshold
    minSpeed = launchSettings.minSpeed

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
                if(abs(turnRate) < minSpeed):
                    turnRate = minSpeed
                robot.rightDrive.run(-turnRate)
            else:
                if(abs(turnRate) < minSpeed):
                    turnRate = minSpeed
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
                if(abs(turnRate) < minSpeed):
                    turnRate = minSpeed
                robot.rightDrive.run(-turnRate)
            else:
                if(abs(turnRate) < minSpeed):
                    turnRate = minSpeed
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
            print(f"Gyro pivot took more than {safetyThreshold}ms, finished with {finalError} degrees error - moving on")
        else:
            print(f"Gyro pivot took {safetyWatch.time()}ms to complete, finished with {finalError} degrees error")
    else:
        print("WARN: GYRO PIVOT DIRECTION INVALID")

'''async def gyroPivot(robot: Robot, direction: float, targetHeading:float, maxSpeed: float = 2400, minSpeed: float = 70):
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
'''

'''async def gyroPivotFS(robot: Robot, direction: float, targetHeading: float, maxSpeed: float = 2400, minSpeed: float = 70, timeout_ms: float = 3000):
    turnDegrees = abs(robot.hub.imu.heading() - targetHeading)
    turnRot = turnDegrees / 360

    safetyWatch = StopWatch()
    timed_out = False 

    if (robot.hub.imu.heading() < targetHeading):
        while robot.hub.imu.heading() < targetHeading:
            currentSpeed = maxSpeed * abs(robot.hub.imu.heading() - targetHeading) / turnDegrees * turnRot
            currentSpeed = max(currentSpeed, minSpeed)
            
            if direction == 0:
                robot.leftDrive.run(currentSpeed)
                robot.rightDrive.run(0)
            else:
                robot.leftDrive.run(0)
                robot.rightDrive.run(-currentSpeed)
            
            await wait(10)
            
            if safetyWatch.time() > timeout_ms:
                timed_out = True
                break
    else:
        while robot.hub.imu.heading() > targetHeading:
            currentSpeed = maxSpeed * abs(robot.hub.imu.heading() - targetHeading) / turnDegrees * turnRot
            currentSpeed = max(currentSpeed, minSpeed)
            
            if direction == 0:
                robot.leftDrive.run(0)
                robot.rightDrive.run(currentSpeed)
            else:
                robot.leftDrive.run(-currentSpeed)
                robot.rightDrive.run(0)
            
            await wait(10)
            
            if safetyWatch.time() > timeout_ms:
                timed_out = True
                break

    robot.leftDrive.hold()
    robot.rightDrive.hold()

    finalError = abs(robot.hub.imu.heading() - targetHeading)
    if timed_out:
        print(f"Gyro pivot TIMED OUT after {safetyWatch.time()}ms with {finalError} degrees error")
        return True
    else:
        print(f"Gyro pivot took {safetyWatch.time()}ms to complete, finished with {finalError} degrees error")
        return False
'''

'''async def gyroSpin(robot: Robot, targetHeading: float, maxSpeed: float = 1200, minSpeed: float = 35):
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
'''

async def play_song(robot: Robot, volume: int = 40):
    robot.hub.speaker.volume(volume)
    crab_rave_notes = [
        "D6/8",
        "Bb5/8", 
        "G5/8", 
        "G5/16",
        "D6/8", 
        "D6/16", 
        "A5/8",
        "F5/8",
        "F5/8",
        "D6/8", 
        "D6/16", 
        "A5/8",
        "F5/8",
        "F5/16",
        "C5/8",
        "C5/8",
        "E5/8",
        "E5/16",
        "F5/16",
    ]
    while(True):
        robot.hub.speaker.play_notes(crab_rave_notes, 125)
        await wait(10)


