#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.parameters import Button
from pybricks.ev3devices import GyroSensor 
from pybricks.parameters import Direction
from pybricks.ev3devices import Motor
from pybricks.robotics import DriveBase
from pybricks.parameters import Stop
from pybricks.ev3devices import ColorSensor
from pybricks.tools import wait
import io

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B, Direction.CLOCKWISE)
right_motor = Motor(Port.C,Direction.CLOCKWISE)
LeftColorSensor = ColorSensor(Port.S1)
RightColorSensor = ColorSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=120)
gyro = GyroSensor(Port.S2, Direction.CLOCKWISE)

try:
    infile = io.open("lval.txt",'r')
except:
    ev3.screen.print("lval.txt does not \nexist!. \nPlease do light \nreading!")
    wait(10000)
    exit

line = infile.readline()
infile.close()
lvals = line.split(' ')
p1w = int(lvals[0])
p1b = int(lvals[1])
p4w = int(lvals[2])
p4b = int(lvals[3])

p1_45 = (p1w - p1b)*0.45+p1b
p1_55 = (p1w - p1b)*0.55+p1b
p1_50 = (p1w - p1b)*0.50+p1b

p4_45 = (p4w - p4b)*0.45+p4b
p4_55 = (p4w - p4b)*0.55+p4b
p4_50 = (p4w - p4b)*0.50+p4b

def gems2blackfwd(white_level,black_level, speed, port):
    gyro.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    gyro_error = 0
    if port == 1:
        white_level = (p1w - p1b)*white_level+p1b
        black_level = (p1w - p1b)*black_level+p1b
        while LeftColorSensor.reflection() <= white_level:
            gyro_error = 0-gyro.angle()
            robot.drive(speed, 3*gyro_error)
        while LeftColorSensor.reflection() >= black_level:
            gyro_error = 0-gyro.angle()
            robot.drive(speed, 3*gyro_error)
        robot.stop(Stop.BRAKE)
    if port == 4:
        white_level = (p4w - p4b)*white_level+p4b
        black_level = (p4w - p4b)*black_level+p4b
        while RightColorSensor.reflection() <= white_level:
            gyro_error = 0-gyro.angle()
            robot.drive(speed, 3*gyro_error)
        while RightColorSensor.reflection() >= black_level:
            gyro_error = 0-gyro.angle()
            robot.drive(speed, 3*gyro_error)
        robot.stop(Stop.BRAKE)
    
def accDecDrive(rotations,speed):
    gyro.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    target_deg=rotations*360
    target_speed = speed

    #Initialize local variables
    speed = 60
    cruising_point = 0
    dec_point = target_deg

    while True:
        if abs(left_motor.angle())>=target_deg:
            robot.stop(Stop.BRAKE)
            break
        else:
            gyro_error = 0-(gyro.angle())
            if speed < 0:
                gyro_error *= -1
            robot.drive(speed, 2*gyro_error)     
            if (abs(left_motor.angle())>=dec_point):
                #Decelarate from deceleration point
                speed -= 5
                ev3.screen.print(str(speed))
            elif (speed < target_speed):
                # Accelerate till cruising pont where the top speed = target_speed
                speed += 5
                if speed >= target_speed:
                    cruising_point = abs(left_motor.angle())
                    if target_deg > (2 * cruising_point):
                        dec_point = target_deg - cruising_point
                    else:
                        dec_point = cruising_point
                        ev3.screen.print("*** Target distance too small!")
                        ev3.speaker.beep()
                ev3.screen.print(str(speed))

def gems(rotations,speed):
    gyro.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    target_deg=rotations*360
    
    while True:
        ev3.screen.print("Checking gyro")
        if abs(left_motor.angle())>=target_deg:
            robot.stop(Stop.BRAKE)
            break
        else:
            gyro_error = 0-(gyro.angle())
            ev3.screen.print(str(gyro_error))
            if speed<0:
                gyro_error *= -1
            robot.drive(speed, 3*gyro_error)

def aligntoblack():
    BeamDone = 0

    while not BeamDone == 1:
        if LeftColorSensor.reflection() > p1_50:
            left_motor.run(50)
        else: 
            left_motor.run(-50)
        if RightColorSensor.reflection() > p4_50:
            right_motor.run(50)
        else:
            right_motor.run(-50)
        if LeftColorSensor.reflection() > p1_45 and LeftColorSensor.reflection() < p1_55:
            if RightColorSensor.reflection() > p4_45 and RightColorSensor.reflection() < p4_55:
                BeamDone = 1
    robot.stop()
    s = str(LeftColorSensor.reflection()) + " " + str(RightColorSensor.reflection())
    ev3.screen.print(s)

def gyrospinturn(angle,speed):
    x=speed
    nx=-1*speed
    gyro.reset_angle(0)
    if angle < 0:
        left_motor.run(nx)
        right_motor.run(x)
    else:
        left_motor.run(x)
        right_motor.run(nx)
    while True:
        if abs(gyro.angle()) >= abs(angle):
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            break

def show_menu():
    global current_row
    button_pressed=False
    ev3.screen.clear()
    ev3.screen.print("Run 1 ***")
    ev3.screen.print("Run 2")
    ev3.screen.print("Run 3")
    ev3.screen.print("Exit")
    while True:
        if Button.CENTER in ev3.buttons.pressed():
            wait(150)
            return current_row
        elif Button.UP in ev3.buttons.pressed():
            wait(150)
            if current_row == 1:
                current_row = 4
            else:
                current_row -= 1
            button_pressed=True
        elif Button.DOWN in ev3.buttons.pressed():
            wait(150)
            if current_row == 4:
                current_row = 1
            else:
                current_row += 1
            button_pressed=True
        if button_pressed:
            if current_row == 1:
                ev3.screen.clear()
                ev3.screen.print("Run 1 ***")
                ev3.screen.print("Run 2")
                ev3.screen.print("Run 3")
                ev3.screen.print("Exit")
            elif current_row == 2:
                ev3.screen.clear()
                ev3.screen.print("Run 1")
                ev3.screen.print("Run 2 ***")
                ev3.screen.print("Run 3")
                ev3.screen.print("Exit")
            elif current_row == 3:
                ev3.screen.clear()
                ev3.screen.print("Run 1")
                ev3.screen.print("Run 2")
                ev3.screen.print("Run 3 ***")
                ev3.screen.print("Exit")
            elif current_row == 4:
                ev3.screen.clear()
                ev3.screen.print("Run 1")
                ev3.screen.print("Run 2")
                ev3.screen.print("Run 3")
                ev3.screen.print("Exit ***")
            button_pressed = False

def run1():
    gyrospinturn(-31,200)
    accDecDrive(1.15,200)
    gyrospinturn(-42,200)
    accDecDrive(3.35,200)
    gyrospinturn(45,200)
    accDecDrive(1,200)
    gyrospinturn(85,200)
    # distance too small
    accDecDrive(0.7,200)
    gyrospinturn(-60,200)
    accDecDrive(3,200)
    gyrospinturn(22,200)
    accDecDrive(5.5,300)
    
# Main program starts here
current_row = 1
while True:
    key = show_menu()
    ev3.screen.clear()
    if key == 1:
        wait(150)
        ev3.screen.print("Run 1")
        run1()
        show_menu()
    elif key == 2:
        ev3.screen.print("Run 2")
        show_menu()
    elif key == 3:
        ev3.screen.print("Run 3")
        show_menu()
    elif key == 4:
        exit

    
