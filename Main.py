######################################################################
# IMPORTS
######################################################################
from hub import light, light_matrix, port, motion_sensor, sound
import runloop
import motor
import color
import color_sensor
import distance_sensor
import motor_pair
import math
import time

######################################################################
# HELPER CLASSES
######################################################################
"""
Timer Class: By measuring the tick count, which increment each
millisecond, the Timer class is able to calculate differences in
time as a float.
"""
class Timer():
    def __init__(self): # Start time
        self.start_ticks = 0

    def now(self): # Returns time in seconds since start
        return time.ticks_diff(time.ticks_ms(), self.start_ticks) / 1000

    def reset(self): # Resets the timer
        self.start_ticks = time.ticks_ms()

######################################################################
# SETUP
######################################################################
# Reset yaw and pair motors
motion_sensor.set_yaw_face(motion_sensor.LEFT)
motion_sensor.reset_yaw(0)
motor_pair.unpair(motor_pair.PAIR_1)
motor_pair.pair(motor_pair.PAIR_1, port.E, port.A)
motor.reset_relative_position(port.C, 0)
motor.reset_relative_position(port.D, 0)

# Assign variables for organization
wheels = motor_pair.PAIR_1

lGear = port.C
rGear = port.D

left_sensor = port.F
right_sensor = port.B

# Create an instance of the Timer Class that starts counting as soon as the code begins
timer = Timer()

# PID Constants
# kP_line = 0.32
# kI_line = 0.00007
# kD_line = 0.2
kP_line = 0.5
kI_line = 0.0001
kD_line = 0.25

kP_fwd = 0.3
kI_fwd = 0.0005
kD_fwd = 0.8

kpTurning = 0.22
kiTurning = 0.00020
kdTurning = 0.22

######################################################################
# HELPER FUNCTIONS
######################################################################
"""
PID Line Follower: Requires seconds you want to follow the line and
the positions of black and white on the line (True if black is right and
white is left, False if black is left and white is right)
"""
def line_follow(secsToFollow, black_r_white_l):
    integral = 0
    lastError = 0

    timer.reset() # Reset the timer

    while timer.now() < secsToFollow:
        # Define the error to be the current reflected light - 30, 
        # as the black averages about 10-20 and the white averages
        # about 60-70, 30 is assumed as the midpoint. The sensor 
        # is swapped based on the side for smoother operation.
        if black_r_white_l:
            error = (color_sensor.reflection(right_sensor) - 30)
        else:
            error = (color_sensor.reflection(left_sensor) - 30)

        # PID Calculations
        kP = error * kP_line
        integral = integral + error
        kI = integral * kI_line
        derivative = error - lastError
        lastError = error
        kD = derivative * kD_line
        PID = (kP+kI+kD)*8
        
        # Depending on the side of black (or white), the wheels turn in different ways
        if black_r_white_l:
            motor_pair.move_tank(wheels, round(150+PID), round(150-PID))
        else:
            motor_pair.move_tank(wheels, round(150-PID), round(150+PID))

    motor_pair.stop(wheels, stop=motor.COAST) # Put wheels into COAST to transition easier

"""
PID Move forward until colour is sensed: Needs the colour that you want the robot
to stop at. An optional parameter is provided for if you want the robot
to travel fast or not.
"""
def forward_until_color(colour, fast = False):
    integral = 0
    lastError = 0

    yaw = motion_sensor.tilt_angles()[0] # Store the initial yaw angle

    base_speed = 200 if not fast else 500 # Ternary condition that updates the based speed

    while color_sensor.color(right_sensor) != colour:
        # Define the error to be the difference of the current yaw angle
        # and the initial yaw angle. As the function is used to travel in
        # a straight line, any variation in the yaw is seen as error.
        error = motion_sensor.tilt_angles()[0] - yaw

        # PID Calculation
        kP = error * kP_fwd
        integral = integral + error
        kI = integral * kI_fwd
        derivative = error - lastError
        lastError = error
        kD = derivative * kD_fwd
        PID = (kP+kI+kD)*8

        # Positive error means the car is too far right, so the left
        # wheel turns faster, and vice versa for negative error.
        motor_pair.move_tank(wheels, round(base_speed+PID), round(base_speed-PID))

    motor_pair.stop(wheels, stop=motor.COAST) # Put wheels into COAST to transition easier

"""
PID Turning relative to the starting position of the robot: Provide the angle,
positive is clockwise and negative is counterclockwise rotation.
"""
def turn_to_absolute(target):
    error = 0
    lastError = 0
    integral = 0
    derivative = 0

    basePower = 0
    yawAngle = 1801 # some random number
    turnNotFinished = True

    # Modify the function logic to take in positive numbers as
    # clockwise rotation and negative as counter.
    target *= -1
    if target < 0:
        target += 360
    
    while turnNotFinished:
        yawAngle = motion_sensor.tilt_angles()[0] #get the current yaw angle
        error = target*10 - yawAngle # Multiply target by 10 as SPIKE measures in decidegrees

        # Check error to move the minimum distance
        if error > 1800:
            error -= 3600
        elif error < -1800:
            error += 3600

        # PID Calculation
        integral = integral + error
        derivative = error - lastError
        P_steering = error * kpTurning
        I_steering = integral * kiTurning
        D_steering = derivative * kdTurning
        Steering = P_steering + I_steering + D_steering

        leftVelocity = round(basePower - Steering)
        rightVelocity = round(basePower + Steering)

        motor_pair.move_tank(wheels, leftVelocity, rightVelocity)

        lastError = error
        time.sleep_ms(10)

        # Only when the error is very small will the turn end.
        if math.fabs(error) <= 5:
            turnNotFinished = False

    motor_pair.stop(wheels, stop=motor.COAST) # Put wheels into COAST to transition easier

"""
PID Turning relative to the current position of the robot: Provide the angle,
positive is clockwise and negative is counterclockwise rotation.
"""
def turn_to_relative(target):
    error = 0
    lastError = 0
    integral = 0
    derivative = 0

    basePower = 0
    initial_yaw = motion_sensor.tilt_angles()[0] # Store the initial yaw angle
    turnNotFinished = True

    # Modify the function logic to take in positive numbers as
    # clockwise rotation and negative as counter.
    target *= -1
    if target < 0:
        target += 360

    while turnNotFinished:
        yawAngle = motion_sensor.tilt_angles()[0] #get the current yaw angle
        # Add the initial yaw to recenter the target to the angle relative from it
        error = (target*10 + initial_yaw) - yawAngle # Multiply target by 10 as SPIKE measures decidegrees

        # Check error to move the minimum distance
        if error > 1800:
            error -= 3600
        elif error < -1800:
            error += 3600

        # PID Calculation
        integral = integral + error
        derivative = error - lastError
        P_steering = error * kpTurning
        I_steering = integral * kiTurning
        D_steering = derivative * kdTurning
        Steering = P_steering + I_steering + D_steering

        leftVelocity = round(basePower - Steering)
        rightVelocity = round(basePower + Steering)

        motor_pair.move_tank(wheels, leftVelocity, rightVelocity)

        lastError = error
        time.sleep_ms(10)

        # Only when the error is very small the turn will end.
        if math.fabs(error) <= 5:
            turnNotFinished = False

    motor_pair.stop(wheels, stop=motor.COAST) # Put wheels into COAST to transition easier

"""
PID Move forward until a colour has been detected for a user specified time. Requires
the colour to detect and how long the colour should be detected for.
"""
def color_for_time(colour, seconds):
    integral = 0
    lastError = 0

    yaw = motion_sensor.tilt_angles()[0] # Store the intial yaw

    base_speed = 200

    timer.reset() # reset the timer

    while timer.now() < seconds: # Loop until the timer has reached the value specified
        # Define the error to be the difference of the current yaw angle
        # and the initial yaw angle. As the function is used to travel in
        # a straight line, any variation in the yaw is seen as error.
        error = motion_sensor.tilt_angles()[0] - yaw

        # PID Calculation
        kP = error * kP_fwd
        integral = integral + error
        kI = integral * kI_fwd
        derivative = error - lastError
        lastError = error
        kD = derivative * kD_fwd
        PID = (kP+kI+kD)*8

        motor_pair.move_tank(wheels, round(base_speed+PID), round(base_speed-PID))

        # If the color sensor detects a different colour at any point in the 
        # while loop, reset the timer so as to not count any time detecting
        # the wrong colour.
        if color_sensor.color(right_sensor) != colour:
            timer.reset()

    motor_pair.stop(wheels, stop=motor.COAST) # Put wheels into COAST to transition easier


######################################################################
# MAIN FUNCTION
######################################################################

async def main():
    # write your code here
    turn_to_absolute(40)
    forward_until_color(colour=color.BLACK)
    line_follow(6, True)
    turn_to_absolute(93)
    await motor_pair.move_for_time(wheels, 1000, 0, velocity=200)
    forward_until_color(colour=color.BLACK, fast=True)
    line_follow(6, True)
    

runloop.run(main())