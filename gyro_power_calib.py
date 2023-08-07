#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.parameters import Button
from pybricks.ev3devices import GyroSensor 
from pybricks.parameters import Direction
import uerrno

# Initialize the EV3 Brick.
ev3 = EV3Brick()
ev3.screen.clear()

button_pressed = []

""" 
Instantiate gyro sensor object. If there is no error in instantiating gyro device
continuously read the gyro and display. If there is error reading the gyro, go back
and re-instantiate device again. 
During these loops if CENTER button is pressed, come out of the program.
"""
while True:
    if Button.CENTER in button_pressed:
        break
    try:
        gyro = GyroSensor(Port.S2, Direction.CLOCKWISE)
        gyro.reset_angle(0)
        while True:
            button_pressed = ev3.buttons.pressed()
            if Button.CENTER in button_pressed:
                break
            try:
                gyro_ang = gyro.angle()
                s = "Angle: " + str(gyro_ang)
                ev3.screen.print(s)
            except OSError as ex:
                # OSError was raised, check for the kind of error
                # If no device error (ENODEV), notify on the console
                if ex.args[0] == uerrno.ENODEV:
                    # ENODEV is short for "Error, no device."
                    ev3.screen.print("Gyro: No device!")
                else:
                    ev3.screen.print("Gyro: OSError {}".format(ex.args[0]))

                # Break to re-instantiate device again
                break 
            wait(100)
    except OSError as ex:
        # OSError was raised, check for the kind of error
        # If no device error (ENODEV), notify on the console
        if ex.args[0] == uerrno.ENODEV:
             # ENODEV is short for "Error, no device."
            ev3.screen.print("Gyro: No device!")
        else:
            ev3.screen.print("Gyro: OSError {}".format(ex.args[0]))
    wait(100)


