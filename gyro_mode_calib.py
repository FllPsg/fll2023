#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Color
from pybricks.tools import wait
from pybricks.parameters import Button
from pybricks.ev3devices import GyroSensor 
from pybricks.parameters import Direction
import uerrno

"""
- Calibrate gyro sensor by mode switch method (from mode rate to mode angle)
- After clabibrating the gyro, do drift test. Continuously reading the gyro
and making sure that the gyro reading does not change rapidly
"""
# Initialize the EV3 Brick.
ev3 = EV3Brick()

ev3.screen.clear()

#Instantiate gyro sensor object. By default the gyro starts in ANGLE mode
while True:
    try:
        gyro = GyroSensor(Port.S2, Direction.CLOCKWISE)
        break
    except OSError as ex:
    # OSError was raised, check for the kind of error
    # If no device error (ENODEV), notify on the console
        if ex.args[0] == uerrno.ENODEV:
             # ENODEV is short for "Error, no device."
            ev3.screen.print("Gyro: No Device!")
        else:
            ev3.screen.print("Gyro: OSError {}".format(ex.args[0]))
    wait(100)

#Read current angle to make sure that it is not 0
gyro_ang = gyro.angle()

if gyro_ang == 0:
    ev3.screen.print("Rotate Robot!")
    while (gyro_ang < 5 and gyro_ang > -5):
        gyro_ang = gyro.angle()

s = "Angle: " + str(gyro_ang)
ev3.screen.print(s)

ev3.screen.print("Reset?")
while True:
    if Button.CENTER in ev3.buttons.pressed():
        ev3.speaker.beep()
        ev3.light.on(Color.RED)
        break
    wait(100)

#Change the gyro mode to RATE to reset the gyro
gyro_speed = gyro.speed()

#Wait till the gyro is reset
while gyro_ang != 0:
    gyro_ang = gyro.angle()
    s = "Angle: " + str(gyro_ang)
    ev3.screen.print(s)

ev3.speaker.beep()
ev3.light.on(Color.GREEN)

# Show gyro reading to check drifting
# Make sure that the reading does not change rapidly
gyro.reset_angle(0)
while True:
    if Button.CENTER in ev3.buttons.pressed():
        break
    gyro_ang = gyro.angle()
    s = "Angle: " + str(gyro_ang)
    ev3.screen.print(s)
    wait(100)