#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import struct, time

# Scaling function for converting stick values (0,255) to (-100,100)
def scale(val, src, dst):
    return (float(val-src[0]) / (src[1]-src[0])) * (dst[1]-dst[0])+dst[0]

# Initialise brick and motors
ev3 = EV3Brick()
ev3.speaker.beep()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Declare variables
drive = 0
turn = 0

# Setup for reading controller input
infile_path = "/dev/input/event4"
in_file = open(infile_path, "rb")
FORMAT = 'llHHI'
EVENT_SIZE = struct.calcsize(FORMAT)
event = in_file.read(EVENT_SIZE)

# Main loop
while event:
    
    # Read incoming event
    (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(FORMAT, event)

    # Joystick moved
    if ev_type == 3:

        if code == 0: # Left stick x
            turn = scale(value, (0,255), (-100,100))
            
        if code == 1: # Left stick y
            drive = scale(value, (0,255), (-100,100))

        if code == 3: # Right stick x
            pass

        if code == 4: # Right stick y
            pass
    
    # Button pressed
    if ev_type == 1:

        if code == 304: # X button
            pass

        if code == 305: # O button
            pass
        
        if code == 307: # △ button
            pass

        if code == 308: # □ button
            pass

    # Set speeds/directions and drive motors
    left_speed = -drive + turn
    right_speed = drive + turn

    left_motor.dc(left_speed)
    right_motor.dc(right_speed)

    # Read new event
    event = in_file.read(EVENT_SIZE)

in_file.close()
