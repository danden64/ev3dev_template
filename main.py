#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import struct, time
from threading import Thread

global lx, ly, rx, ry, bX, bTriangle, bSquare, bCircle
global RUNNING
RUNNING = True

# Scaling function for converting stick values (0,255) to (-100,100)
def scale(val, src, dst):
    return (float(val-src[0]) / (src[1]-src[0])) * (dst[1]-dst[0])+dst[0]

# Initialise brick
ev3 = EV3Brick()
ev3.speaker.beep()

# Initialise motors
b = Motor(Port.B)
c = Motor(Port.C)

# Autonomous
def Auto():
    ev3.say("Autonomous")

start_time = time.time()
while start_time < start_time + 15:
    Auto()

# Drive motors
def mainThread():
    global RUNNING
    global lx, ly, rx, ry, bX, bTriangle, bSquare, bCircle

    while RUNNING:

        b_speed = ly - rx
        c_speed = ly + rx

        b.dc(b_speed)
        c.dc(c_speed)
        
thread = Thread(target=mainThread)
thread.start()

# Setup for reading controller input
infile_path = "/dev/input/event4"
with open(infile_path, "rb") as in_file:
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
                lx = scale(value, (0,255), (-100,100))
                
            if code == 1: # Left stick y
                ly = scale(value, (0,255), (-100,100))

            if code == 3: # Right stick x
                rx = scale(value, (0,255), (-100,100))

            if code == 4: # Right stick y
                ry = scale(value, (0,255), (-100,100))

        # Button pressed
		if ev_type == 1:

			if code == 304: # X button
				if value == 1:
                    bX = True
                elif value == 0:
                    bX = False

			if code == 305: # O button
				if value == 1:
                    bCircle = True
                elif value == 0:
                    bCircle = False
			
			if code == 307: # △ button
				if value == 1:
                    bTriangle = True
                elif value == 0:
                    bTriangle = False

			if code == 308: # □ button
				if value == 1:
                    bSquare = True
                elif value == 0:
                    bSquare = False

        # Read new event
        event = in_file.read(EVENT_SIZE)

RUNNING = False
ev3.speaker.beep()
