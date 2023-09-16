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
from pybricks.tools import wait, DataLog, StopWatch
import io
import uerrno
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

try:
    infile = io.open("lval.txt",'r')
except:
    ev3.screen.print("lval.txt does not \nexist!. \nPlease do light \nreading!")
    ev3.speaker.beep()
    wait(10000)
    exit

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

# Light value raw value to % mapping (for white (100%), blacn (0%) 45%, 55% and 50%)
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

def accDecDrive(total_dist, start_speed = 30, top_speed = 300, acc_dist=0.2, dec_dist=0.2):
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
    s = str(LeftColorSensor.reflection()) + " " + str(RightColorSensor.reflection())
    ev3.screen.print(s)

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
            left_motor.brake()
            right_motor.brake()
            break

def gyrospinturntime(angle, speed, time):
    x=speed
    nx=-1*speed
    ctime = robot_clock.time()
    abs_angle = abs(angle)
    gyro.reset_angle(0)
    if angle < 0:
        left_motor.run(nx)
        right_motor.run(x)
    else:
        left_motor.run(x)
        right_motor.run(nx)
    while True:
        if (robot_clock.time() >= ctime+time) or (abs(gyro.angle()) >= abs_angle):
            left_motor.brake()
            right_motor.brake()
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
        ev3.screen.print("Run 1.pink ***")
        ev3.screen.print("Run 1.orange")
        ev3.screen.print("Run 2")
    elif row == 2:
        ev3.screen.print("Run 1.pink")
        ev3.screen.print("Run 1.orange ***")
        ev3.screen.print("Run 2")
    elif row == 3:
        ev3.screen.print("Run 1.pink")
        ev3.screen.print("Run 1.orange")
        ev3.screen.print("Run 2 ***")

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
                current_row = 3
            else:
                current_row -= 1
            show_menu(current_row)
        elif down_pressed:
            if current_row == 3:
                current_row = 1
            else:
                current_row += 1
            show_menu(current_row)


def run1(color=1):

    def resetleftmediummotor():
        left_medium_motor.reset_angle(0)
        left_medium_motor.run_time(350,1500, Stop.BRAKE, False)

    def resetrightmediummotor():
        right_medium_motor.reset_angle(0)
        right_medium_motor.run_time(350,1500, Stop.BRAKE, False)
    
    def movefrombase():
        simplemovestraight(0.75,120)

    def moveto3dexp():
        simplemovestraight(1.1,120)
    
    def lamovedown_3dc():
        left_medium_motor.reset_angle(0)
        left_medium_motor.run_target(200,-67, Stop.BRAKE)

    # Mission: 3D Cenima
    run_parallel(resetleftmediummotor, resetrightmediummotor, movefrombase)
    gyrospinturn(-100, 200)
    run_parallel(moveto3dexp,lamovedown_3dc)
    #Align to model (3D cenima)
    gyrospinturn(-11, 150)
    
    simplemovestraight(0.2,100)
    # Rotate right to over dragon to complete 3D Cinema mission
    gyrospinturn(11,150)
    
    # Mission: Audience Delivery - 1 (Destination: 3D Cenima)
    def lamoveup_ad1():
        left_medium_motor.reset_angle(0)
        left_medium_motor.run_time(350,1000, Stop.BRAKE, False)

    def movefwd_ad1():  
        simplemovestraight(0.6,100) 

    run_parallel(lamoveup_ad1, movefwd_ad1)
    simplemovestraight(-0.6, 120)
    
    # Mission: Audience Delivery - 2 (Destination: Skateboard area)
    gyrospinturn(36, 150)
    # Move towards teh skateboard area (the audience id moved to the skateboard area)
    accDecDrive(3.3,30,300,0.3,0.3)
    # Move backward slightly to clear way for the audience to be delivered in the next step
    simplemovestraight(-0.5,100)
    # Rotate left to deliver the audience at the same align the left arm correctly towards the 
    # Stage manager loop
    gyrospinturn(-68,200)

    #Mission: Expert delivery - 1 (Stage Manager collection)
    def ramovedowntostagemanager():
        right_medium_motor.reset_angle(0)
        right_medium_motor.run_target(200, -70, Stop.BRAKE, False)

    left_medium_motor.reset_angle(0)
    left_medium_motor.run_time(-300,500, Stop.BRAKE)
    # Align to Theater scene change model
    gyrospinturn(-10, 150)
    simplemovestraight(-0.5,100)
    run_parallel(lamoveup_ad1, ramovedowntostagemanager)
    # Pick up stage manager 
    simplemovestraight(0.75,100)

    def liftstagemanager():
        right_medium_motor.reset_angle(0)
        right_medium_motor.run_time(150, 1000, Stop.BRAKE, False)

    def spintoscenechange():
        gyrospinturn(0.25,100)

    # Mission Theater Scene Change
    run_parallel(liftstagemanager,spintoscenechange)

    simplemovestraight(0.45,100)
    simplemovestraight(-0.45,100)
    # If Orange color, change scene one more time 
    if color == 2: 
        simplemovestraight(0.46,100)
        simplemovestraight(-0.53,100)

    # Mission Virtual Reality 
    gyrospinturn(67, 200)
    # Move backward little bit so that when we rotate towards VR, the audience 
    # in the skateboard area is not disturbed.  
    simplemovestraight(-0.5,100)
    # Turn to align to the mission
    gyrospinturn(57,200)
    #Move forward to the mission VR
    accDecDrive(1.05,30,250,0.3,0.3)
    # Lower the left arm fast till it touches the orange lever
    left_medium_motor.reset_angle(0)
    left_medium_motor.run_target(350,-90, Stop.BRAKE, True)
    # We rotate slightly right to align to orange lever
    gyrospinturn(1,50)
    # Lower the left arm again fast so that the entire force is used to lower the lever
    left_medium_motor.run_time(-350,1500, Stop.BRAKE, True)
    # Raise the left arm to complete the mission.
    left_medium_motor.run_time(350,1000, Stop.BRAKE, True)

    """# Mission Expert Delivery - 2 (Collecting Sound Engineer)
    #align robot at sound mixer
    gyrospinturn(21,200)
    simplemovestraight(0.5,150)
    gyrospinturn(62,200) 
    simplemovestraight(-0.15,150)
    right_medium_motor.run_time(-200,375, Stop.BRAKE, True) 
    # A lign Robot
    gyrospinturn(18,150)
    gyrospinturn(-5,150)
    simplemovestraight(0.25,100) 
    # Pick up Sound engineer
    right_medium_motor.run_time(150,500, Stop.BRAKE, True) """

def run2():
    pass
def run3():
    pass
    

#---------------------------------------  
# Main program loop starts here
#----------------------------------------
current_row = 1
while True:
    key = get_menu_selection()
    ev3.screen.clear()
    if key == 1:
        ev3.screen.print("Run 1.pink")
        run1(1)
        robot_stop(2)
    elif key == 2:
        ev3.screen.print("Run 1.orange")
        run1(2)
    elif key == 3:
        ev3.screen.print("Run 2")
        run3()



    
