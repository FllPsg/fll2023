#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.parameters import Button
from pybricks.ev3devices import GyroSensor 
from pybricks.parameters import Direction

# Initialize the EV3 Brick.
ev3 = EV3Brick()

ev3.screen.clear()

#Instantiate gyro sensor object. By default the gyro starts in ANGLE mode
gyro = GyroSensor(Port.S2, Direction.CLOCKWISE)

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
        break

#Change the gyro mode to RATE to reset the gyro
gyro_speed = gyro.speed()

#Wait till the gyro is reset
while gyro_ang != 0:
    gyro_ang = gyro.angle()
    s = "Angle: " + str(gyro_ang)
    ev3.screen.print(s)

ev3.screen.print("Gyro reset complete!")

s = "Angle: " + str(gyro_ang)
ev3.screen.print(s)

wait(10000)