#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.ev3devices import ColorSensor
from pybricks.tools import wait
from pybricks.parameters import Button
import io

# Initialize the EV3 Brick.
ev3 = EV3Brick()

LeftColorSensor = ColorSensor(Port.S1)
RightColorSensor = ColorSensor(Port.S4)

ev3.screen.clear()
#ev3.screen.draw_text(5,5,"Port 1:Dark",Color.BLACK,background_color=None)

while True:
    p1w = LeftColorSensor.reflection()
    s = "Port 1: White: " + str(p1w)
    ev3.screen.clear() 
    ev3.screen.print(s)
    if Button.CENTER in ev3.buttons.pressed():
        wait(150)
        ev3.speaker.beep()
        break

ev3.screen.clear()
while True:
    p1b = LeftColorSensor.reflection()
    s = "Port 1: Black: " + str(p1b)
    ev3.screen.clear()
    ev3.screen.print(s)
    if Button.CENTER in ev3.buttons.pressed():
        wait(150)
        ev3.speaker.beep()
        break

while True:
    p4w = RightColorSensor.reflection()
    s = "Port 4: White: " + str(p4w)
    ev3.screen.clear() 
    ev3.screen.print(s)
    if Button.CENTER in ev3.buttons.pressed():
        wait(150)
        ev3.speaker.beep()
        break

ev3.screen.clear()
while True:
    p4b = RightColorSensor.reflection()
    s = "Port 4: Black: " + str(p4b)
    ev3.screen.clear()
    ev3.screen.print(s)
    if Button.CENTER in ev3.buttons.pressed():
        wait(150)
        ev3.speaker.beep()
        break

outfile = io.open("lval.txt", "w")
if outfile:
    s = str(p1w) + ' ' + str(p1b) + ' ' + str(p4w) + ' ' + str(p4b)+ "\n"
    outfile.write(s)
    outfile.close()
infile = io.open('lval.txt', 'r')
ev3.screen.print(infile.read())
wait(10000)