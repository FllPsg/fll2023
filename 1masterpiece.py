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

NEED_LOGGING = False
if NEED_LOGGING:
    data_log = DataLog('LogMessage')
    def LogMsg(msg):
        data_log.log(msg)
else:
    def LogMsg(msg):
        pass

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Instantiate a timer
robot_clock = StopWatch()

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

# Light reading value normalization to 0-100%
# Light value raw value to % mapping (for white (100%), blacn (0%) 45%, 55% and 50%)
try:
    infile = io.open("lval.txt",'r')
except:
    ev3.screen.print("lval.txt does not \nexist!. \nPlease do light \nreading!")
    ev3.speaker.beep()
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

def gyro_soft_calib():
    """ Gyro sensor run time calibration by mode switching ("GYRO-CAL" --> "GYRO-ANG")
    """
    #print("robot: gyro_calib")
    ev3.light.on(Color.RED)
    wait(200)
    calib_gyro = Ev3devSensor(Port.S2)

    for i in range(3):
        calib_gyro.read("GYRO-CAL")
        wait(200)
        angle = int(calib_gyro.read("GYRO-ANG")[0]) 
        if angle == 0:
            print("gyro calib done!", i)
            break
    wait(200)
    gyro.reset_angle(0)
    #ev3.speaker.beep()
    ev3.light.on(Color.GREEN)

def robot_stop(mode=1):
    robot.stop()
    if mode == 1: 
        left_motor.brake()
        right_motor.brake()
    elif mode == 2:
        left_motor.stop()
        right_motor.stop()
    elif mode == 3: 
        left_motor.hold()
        right_motor.hold()

def gyro_read_angle():
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
        robot_stop()
    if port == 4:
        white_level = (p4w - p4b)*white_level+p4b
        black_level = (p4w - p4b)*black_level+p4b
        while RightColorSensor.reflection() <= white_level:
            gyro_error = 0-gyro.angle()
            robot.drive(speed, 3*gyro_error)
        while RightColorSensor.reflection() >= black_level:
            gyro_error = 0-gyro.angle()
            robot.drive(speed, 3*gyro_error)
        robot_stop()

def calc_var_acc_speed(cdist, axdist, dxdist, sspeed, tspeed):
    """ Compute the speed based on current position of the robot
    Paramters:
        cdist = Current distance in % of total distance
        axdist = Acceleration distance in % of total distance
        dxdist = Deceleration distance in  % of total distance
        sspeed = Starting speed
        tspeeed = Top speed
    returns:
        drspeed = drive speed in whatever unit given.
    """
    # If the current position of the robot is before the distance of the cruising point
    # calculate speed depending on the current location. The additional speed added to the
    # minimum starting speed is proportional to the distance travelled
    # The idea: Slope equation of a line ( y = mx + c)
    if cdist >= 0 and cdist <= axdist:
         drspeed = (tspeed-sspeed)/(axdist)*cdist+sspeed
   
    # If the current position of robot is in the distance between the end of
    # acceleration and start of decleration (curising distance), set the speed to
    # top speed
    elif cdist >= axdist and cdist <= 1 - dxdist:
        drspeed = tspeed
   
   # If the current position of the robot is in the deccelration zone calculate the speed
    # as per the current location. As the robot goes closer and closer to the destination,
    # amount of deceleration is also reduced. By the time the robot reaches the destination,
    # the speed expected to be same as the starting speed.
    # The idea: Slope equation of a line (y = -mx + c)
    
    elif cdist >= 1-dxdist and cdist <= 1:
        drspeed = (tspeed-sspeed)/(-dxdist)*(cdist-(1-dxdist))+tspeed
    
    # setting the power to min before the drive/turn and after
    else:
        drspeed = sspeed

    # return the computed speed
    return drspeed

def accDecGems(total_dist, start_speed = 30, top_speed = 300, acc_dist=0.2, dec_dist=0.2):
    """ Acceleration/Deceleration Drive with variable acceleration
        Parameters:
            total_dist = The total target distance. Given in number of rotations.
                        Negative distance is for backward direction
            start_speed = Starting speed
            top_speed = The top speed (curising speed after raising acceleration)
            acc_dist = The distance from the starting posion, how far the acceleration is done
                    At the end of this distance, the cruising will start (no acceleration)
                    This is distance from start as % of total_distance
            dec_dist = Deceleration distance. The distance from the end how far deceleration should be one
                    This is given % of total_distance.
    """
    #Proportional factor for PID
    # I and D are not used.
    kp = 1.0

    gyro.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    #Convert distance in rotation to distance in angle and set the direction
    direction = 1
    if total_dist < 0:
        direction = -1
    total_dist = abs(total_dist*360)

    while True:
        current_dist = abs(left_motor.angle())
        if current_dist >= total_dist:
            robot_stop()
            break
        else:
            drive_speed = calc_var_acc_speed(current_dist/total_dist, acc_dist, dec_dist, start_speed, top_speed)
            gyro_error = 0-(gyro.angle())
            robot.drive(drive_speed*direction, kp*gyro_error)
            LogMsg("Speed: {}, Distance: {}, Error: {}".format(drive_speed, current_dist, gyro_error))
        wait(10)

def gems(rotations,speed):
    gyro.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    target_deg=rotations*360
    
    while True:
        ev3.screen.print("Checking gyro")
        if abs(left_motor.angle())>=target_deg:
            robot_stop()
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
    robot_stop()

def gyrospinturn(angle,speed):
    x=speed
    nx=-1*speed
    abs_angle = abs(angle)
    gyro.reset_angle(0)
    if angle < 0:
        left_motor.run(nx)
        right_motor.run(x)
    else:
        left_motor.run(x)
        right_motor.run(nx)
    while True:
        ga = gyro.angle()
        if abs(ga) >= abs_angle:
            robot_stop()
            break

def simplemovestraight(distance_rotation, speed):
    """ converting rotations into mm
    1 wheel diameter = 56 mm
    1 rotation = 56 * pi = 175.84 mm """
    distance_mm = 175.84 * distance_rotation
    robot.settings(straight_speed=speed)
    robot.straight(distance_mm) 
    robot_stop()

def show_menu(row):
    ev3.screen.clear()
    if row == 1:
        ev3.screen.print("Run 1 ___")
        ev3.screen.print("Run 2.pink")
        ev3.screen.print("Run 2.orange")
        ev3.screen.print("Run 3")
        ev3.screen.print("Run 4")
    elif row == 2:
        ev3.screen.print("Run 1")
        ev3.screen.print("Run 2.pink ___")
        ev3.screen.print("Run 2.orange")
        ev3.screen.print("Run 3")
        ev3.screen.print("Run 4")
    elif row == 3:
        ev3.screen.print("Run 1")
        ev3.screen.print("Run 2.pink")
        ev3.screen.print("Run 2.orange ___")
        ev3.screen.print("Run 3")
        ev3.screen.print("Run 4")
    elif row == 4:       
        ev3.screen.print("Run 1")
        ev3.screen.print("Run 2.pink")
        ev3.screen.print("Run 2.orange")
        ev3.screen.print("Run 3 ___")
        ev3.screen.print("Run 4")
    elif row == 5:
        ev3.screen.print("Run 1")
        ev3.screen.print("Run 2.pink")
        ev3.screen.print("Run 2.orange")
        ev3.screen.print("Run 3")
        ev3.screen.print("Run 4 ___")
     
def get_menu_selection():
    global current_row
    show_menu(current_row)       
    while True:
        buttons_pressed = ev3.buttons.pressed()
        wait(150)
        center_pressed = Button.CENTER in buttons_pressed
        up_pressed = Button.UP in buttons_pressed
        down_pressed = Button.DOWN in buttons_pressed
        if center_pressed:
            return current_row
        elif up_pressed:
            if current_row == 1:
                current_row = 5
            else:
                current_row -= 1
            show_menu(current_row)
        elif down_pressed:
            if current_row == 5:
                current_row = 1
            else:
                current_row += 1
            show_menu(current_row)

#**************************************
# Helper functions
#**************************************
def resetleftmediummotor():
    left_medium_motor.reset_angle(0)
    left_medium_motor.run_time(350,1500, Stop.BRAKE, False)

def resetrightmediummotor():
    right_medium_motor.reset_angle(0)
    right_medium_motor.run_time(350,1500, Stop.BRAKE, False)

#*************************************
# Run 1 
#*************************************
def run1():
    def movefrombase():
        simplemovestraight(0.75,120)

    def moveto3dexp():
        simplemovestraight(1.1,120)
    
    def lamovedown_3dc():
        left_medium_motor.reset_angle(0)
        left_medium_motor.run_target(200,-68, Stop.BRAKE)

    #Initialize devices initial positions
    gyro_soft_calib()
    left_medium_motor.stop()
    right_medium_motor.stop()

    # *** Mission: 3D Cenima ***
    run_parallel(resetleftmediummotor, resetrightmediummotor, movefrombase)
    gyrospinturn(-100, 200)
    run_parallel(moveto3dexp,lamovedown_3dc)
    #Align to model (3D cenima)
    gyrospinturn(-11, 150)
    
    simplemovestraight(0.2,100)
    # Rotate right to over dragon to complete 3D Cinema mission
    gyrospinturn(10,150)
    
    # *** Mission: Audience Delivery - 1 (Destination: 3D Cenima) ***
    def lamoveup_ad1():
        left_medium_motor.reset_angle(0)
        left_medium_motor.run_time(350,1000, Stop.BRAKE, False)

    def movefwd_ad1():  
        simplemovestraight(0.6,100) 

    run_parallel(lamoveup_ad1, movefwd_ad1)
    simplemovestraight(-0.6, 120)
    
    # *** Mission: Audience Delivery - 2 (Destination: Skateboard area) ***
    gyrospinturn(36, 150)
    # Move towards the skateboard area (the audience id moved to the skateboard area)
    accDecGems(3.4,30,300,0.3,0.3)
    # Move backward slightly to clear way for the audience to be delivered in the next step
    simplemovestraight(-0.4,100)
    gyrospinturn(-20, 150)
    simplemovestraight(-0.25, 100)
    gyrospinturn(25,250)
    # Move back straignt to return to left launch area.
    simplemovestraight(-4.3, 300)

    #********* End of Run 1 *********

#***********************************************
# Run 2 
# ***********************************************
def run2(color = 1):
    #Initialization
    gyro_soft_calib()
    left_medium_motor.stop()
    right_medium_motor.stop()
    
    # Start from the base
    run_parallel(resetleftmediummotor, resetrightmediummotor)
    # Rotate towards the aligning line near theater scene change mission
    gyrospinturn(-70, 150)
    # Move straight to the aligning line
    accDecGems(2.75,30,300,0.3,0.3)
    gems2blackfwd(0.95,0.05, 100, 1)
    # Prepare to aligning to black line
    gyrospinturn(-40,150)
    simplemovestraight(0.25,100)
    # Alighn to black line (near theater scene change mission)
    aligntoblack()

    # *** Mission Expert delivery - 1 (Stage manager collection) ***
    # Move back a little to give room for the right arm when brought down
    simplemovestraight(-0.25,100)
    def rotatetostagemanager():
        gyrospinturn(10,150)
    def ramovedowntoexpert():
        right_medium_motor.reset_angle(0)
        right_medium_motor.run_target(200, -75, Stop.BRAKE, False)
    # Rotate the robot to position aligning towards the stage manager and bring the right arm down
    # simultaneously
    run_parallel(rotatetostagemanager, ramovedowntoexpert)
    # Move straight into the loop of the stage manager
    simplemovestraight(0.5,100)
    def movebackfromstagemanager():
        simplemovestraight(-0.25,150)
    def liftupthestagemanager():
        right_medium_motor.reset_angle(0)
        right_medium_motor.run_time(150,1500, Stop.BRAKE, False)
    # Lift up the stage manager (collect stage manager) and move backwards slightly to prepare to 
    # activate organge lever to change scene to pink (this is default always)
    run_parallel(movebackfromstagemanager, liftupthestagemanager)

    # *** Mission: Theater Scene Change ***
    # Adjust the angle of the lancer to activate the organge lever
    gyrospinturn(16,150)
    # Move forward and backward to activate the orange lever to change the scene once (to change to pink)
    simplemovestraight(0.3,150)
    simplemovestraight(-0.3,150)

    # If Orange is chosen, activate the orange lever one more time to change the scene to orange.
    if color==2:
        simplemovestraight(0.3,150)
        simplemovestraight(-0.3,150)

    # Prepare for align to black again to reset the errors created due to shacking of the robot while 
    # performing theater scene change mission
    gyrospinturn(-8,150)
    # Align to black. This positions the robot in the same spot with same orientation after completinng 
    # theater scene change mission, irrespective of pink or orange scene change. 
    aligntoblack()

    # *** Mission: Immersive Experience ***
    # Spin rotate towards the immersive experience mission such that the left arm is in line with
    # the orange lever
    gyrospinturn(117, 150)
    #Move forward to the immersive experience mission
    accDecGems(1.4,30,250,0.3,0.3)
    # Lower the left arm fast till it touches the orange lever
    left_medium_motor.reset_angle(0)
    left_medium_motor.run_target(350,-90, Stop.BRAKE, True)
    # Lower the left arm again fast so that the entire force is used to lower the lever
    left_medium_motor.run_time(-350,1000, Stop.BRAKE, True)
    # Raise the left arm to complete the mission.
    left_medium_motor.run_time(350,1000, Stop.BRAKE, True)

    # *** Expert delivery - 2 (Collecting sound engineer)
    # Move forward little bit so that robot will not hit sound mixer in the rotation done below
    simplemovestraight(0.4,150)
    # Rotate towards the sound engineer. The right arm is in line with the loop of the sound engineer
    gyrospinturn(98,150)
    # Bring the right arm to pickup the sound engineer
    ramovedowntoexpert()
    # Move into the loop of the sound engineer
    simplemovestraight(0.5,150)
    # Pick up sound engineer by lifting the right arm
    right_medium_motor.reset_angle(0)
    right_medium_motor.run_target(100, 65, Stop.BRAKE)
    
    # *** Mission: Augmented reality ***
    # Re-init the gyro to reset the accumulated error (Make sure that the robot is still here)
    gyro_soft_calib()
    # Rotate towards the AR mission
    gyrospinturn(-90,150)
    # Move towards the AR mission
    accDecGems(2,30,300,0.2,0.2)
    # Rotate left slightly to position the robot arm to the left of the orange activator
    gyrospinturn(-15,150)
    # Move forward to keep the orange activator at the robot arm lenght
    simplemovestraight(0.5,150)
    # Lower the rigt arm. The right arm is touching the floor just left to the orange activator
    left_medium_motor.reset_angle(0)
    left_medium_motor.run_time(-350,500, Stop.BRAKE, True)
    # Rotate clockwise to activate the orange lever
    gyrospinturn(25,200)
    # Continue activating the orange lever by pushing foward from this point onwards
    simplemovestraight(0.6,150)
    # Complete the activating the orange lever by pushing sideways left
    gyrospinturn(-10,150)
    
    # ***Mission: Craft Creater - 1 (Opening the Printer)***
    # Rotate towards VR craft createor mission
    gyrospinturn(50,151)
    # Bring up the right arm to level with the organge lid of the 3D printer
    left_medium_motor.reset_angle(0)
    left_medium_motor.run_target(350,38, Stop.BRAKE, True)
    # Move the robot right arm under the 3D printer's orange lid
    simplemovestraight(0.55,150)
    """left_medium_motor.reset_angle(0)
    left_medium_motor.run_time(250,500, Stop.BRAKE, True)"""

#************************************************
# Run 3 
#************************************************
def run3():
    pass 

#************************************************
# Run 4
#************************************************
def run4():
    gyro_soft_calib() 
    gems2blackfwd(0.95,0.05, 100, 1)

#****************************************************
# Main program loop starts here
#****************************************************
current_row = 1
while True:
    key = get_menu_selection()
    ev3.screen.clear()
    if key == 1:
        ev3.screen.print("Run 1")
        run1()
    elif key == 2:
        ev3.screen.print("Run 2.pink")
        run2(1)
    elif key == 3:
        ev3.screen.print("Run 2.orange")
        run2(2)
    elif key == 4:
        ev3.screen.print("Run 3")
        run3()
    elif key == 5:
        ev3.screen.print("Run 4")
        run4()