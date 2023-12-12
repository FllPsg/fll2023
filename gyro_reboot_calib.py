#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.parameters import Button
from pybricks.ev3devices import GyroSensor 
from pybricks.parameters import Direction
from pybricks.iodevices import AnalogSensor
import uerrno

# Initialize the EV3 Brick.
ev3 = EV3Brick()
ev3.screen.clear()

button_pressed = []

""" 
First reboot the Gyro by calling AnalogSensor(). Once reboot is done then do the
following

Instantiate gyro sensor object. If there is no error in instantiating gyro device
continuously read the gyro and display. If there is error reading the gyro, go back
and re-instantiate device again. 
During these loops if CENTER button is pressed, come out of the program.
"""
ev3.screen.print("Rebooting Gyro...")
wait(1000)
AnalogSensor(Port.S2)
ev3.screen.print("Reboot complete!")
while True:
    if Button.CENTER in button_pressed:
        break
    try:
        ev3.screen.print("Creating Gyro...")
        gyro = GyroSensor(Port.S2, Direction.CLOCKWISE)
        ev3.screen.print("Gyro created!")
        ev3.screen.print("Initializing Gyro...")
        gyro.reset_angle(0)
        read_count = 0
        while True:
            button_pressed = ev3.buttons.pressed()
            if Button.CENTER in button_pressed:
                break
            try:
                gyro_ang = gyro.angle()
                read_count = read_count + 1
                ev3.screen.print("{}. Angle: {}".format(read_count, gyro_ang))
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
