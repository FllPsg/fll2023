#!/usr/bin/env pybricks-micropython

import io
import uerrno
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Button, Direction, Stop, Color
from pybricks.ev3devices import GyroSensor, Motor 
from pybricks.robotics import DriveBase
from pybricks.ev3devices import ColorSensor
from pybricks.iodevices import Ev3devSensor
from pybricks.tools import wait, DataLog, StopWatch
from pybricks.experimental import run_parallel

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Instantiate the left large motor
try:
    left_motor = Motor(Port.B, Direction.CLOCKWISE)
except OSError as ex:
    # OSError was raised, check for the kind of error
    # If no device error (ENODEV), notify on the console
    if ex.args[0] == uerrno.ENODEV:
        # ENODEV is short for "Error, no device."
        ev3.screen.print("Left LM: No device!")
    else:
        ev3.screen.print("Left LM: OSError {}".format(ex.args[0]))
    ev3.speaker.beep()
    wait(3000)

# Instantiate the right large motor
try:
    right_motor = Motor(Port.C,Direction.CLOCKWISE)
except OSError as ex:
    # OSError was raised, check for the kind of error
    # If no device error (ENODEV), notify on the console
    if ex.args[0] == uerrno.ENODEV:
        # ENODEV is short for "Error, no device."
        ev3.screen.print("Right LM: No device!")
    else:
        ev3.screen.print("Right LM: OSError {}".format(ex.args[0]))
    ev3.speaker.beep()
    wait(3000)

#Insntantiate left color sensor
try:
    LeftColorSensor = ColorSensor(Port.S1)
except OSError as ex:
    # OSError was raised, check for the kind of error
    # If no device error (ENODEV), notify on the console
    if ex.args[0] == uerrno.ENODEV:
        # ENODEV is short for "Error, no device."
        ev3.screen.print("Left CS: No device!")
    else:
        ev3.screen.print("Left CS: OSError {}".format(ex.args[0]))
    ev3.speaker.beep()
    wait(3000)

# Instantiate right color sensor
try:
    RightColorSensor = ColorSensor(Port.S4)
except OSError as ex:
    # OSError was raised, check for the kind of error
    # If no device error (ENODEV), notify on the console
    if ex.args[0] == uerrno.ENODEV:
        # ENODEV is short for "Error, no device."
        ev3.screen.print("Right CS: No device!")
    else:
        ev3.screen.print("Right CS: OSError {}".format(ex.args[0]))
    ev3.speaker.beep()
    wait(3000)

# Instantiate Gyro sensor
try:
    gyro = GyroSensor(Port.S2, Direction.CLOCKWISE)
except OSError as ex:
    # OSError was raised, check for the kind of error
    # If no device error (ENODEV), notify on the console
    if ex.args[0] == uerrno.ENODEV:
        # ENODEV is short for "Error, no device."
        ev3.screen.print("Gyro: No device!")
    else:
        ev3.screen.print("Gyro: OSError {}".format(ex.args[0]))
    ev3.speaker.beep()
    wait(3000)

# Instantiate the left medium motor
try:
    left_medium_motor = Motor(Port.A, Direction.CLOCKWISE)
except OSError as ex:
    # OSError was raised, check for the kind of error
    # If no device error (ENODEV), notify on the console
    if ex.args[0] == uerrno.ENODEV:
        # ENODEV is short for "Error, no device."
        ev3.screen.print("Left MM: No device!")
    else:
        ev3.screen.print("Left MM: OSError {}".format(ex.args[0]))
    ev3.speaker.beep()
    wait(3000)

# Instantiate the right medium motor
try:
    right_medium_motor = Motor(Port.D, Direction.CLOCKWISE)
except OSError as ex:
    # OSError was raised, check for the kind of error
    # If no device error (ENODEV), notify on the console
    if ex.args[0] == uerrno.ENODEV:
        # ENODEV is short for "Error, no device."
        ev3.screen.print("Right MM: No device!")
    else:
        ev3.screen.print("Right MM: OSError {}".format(ex.args[0]))
    ev3.speaker.beep()
    wait(3000)

# Instantiate the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=120)

ev3.screen.clear()
robot.drive(350, 0)
while True:
    battery_level = ev3.battery.voltage()/1000
    ev3.screen.clear()
    ev3.screen.print("\n\nBattery Level: {:.2f}".format(battery_level))
    wait(300)
    if battery_level <= 7.95:
        robot.stop()
        exit()
