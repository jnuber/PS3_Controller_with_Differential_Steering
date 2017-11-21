#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#  PiPlateMotor_Driver.py
#  
#  Copyright 2017 John Nuber <pi@raspberrypi>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
import time
from time import sleep
import pygame
import piplates.MOTORplate as MOTOR          #import the MOTORplate module

# Define Motors
FL = 1				# Define Motor 1 = Front Left   - FL
FR = 2  			# Define Motor 2 = Fright Right - FR
RL = 3  			# Defome Motor 3 = Rear Left    - RL
RR = 4  			# Define Motor 4 = Rear Right   - RR

# Define inital motor parameters
speed = 0			# Initalize motor speed 0 - 100
rate  = 0.0			# Initalize aceleration rate 0 - 100

# Function to set all drives off
def MotorOff():
    #print("Stopping Motor")
    motor.dcSTOP(ctl,FL)
    motor.dcSTOP(ctl,FR)
    motor.dcSTOP(ctl,RL)
    motor.dcSTOP(ctl,RR)
        
    status = "stopped"

def resetCtl():
	MOTOR.clrLED(ctl)
	sleep(1)
	MOTOR.RESET(ctl)
	return

def initMotor(FLdir,FRdir,RLdir,RRdir):
	#print("Setting: ",rotation)
	motor.dcCONFIG(ctl,FL,FLdir,0.0,0.0)
	motor.dcCONFIG(ctl,FR,FRdir,0.0,0.0)
	motor.dcCONFIG(ctl,RL,RLdir,0.0,0.0)
	motor.dcCONFIG(ctl,RR,RRdir,0.0,0.0)
	
	motor.dcSTART(ctl,FL)
	motor.dcSTART(ctl,FR)
	motor.dcSTART(ctl,RL)
	motor.dcSTART(ctl,RR)
	
	if   (FLdir == FRdir == RLdir == RRdir  == "cw") : 
		status = "forward"
		return
	elif (FLdir == FRdir == RLdir == RRdir  == "ccw"): 
		status = "backward" 
		return
	elif (FLdir == RLdir == "cw"  and RLdir == RRdir == "ccw"):
		status = "left"
	elif (FLdir == RLdir == "ccw" and RLdir == RRdir == "cw"):
		status = "right"
	else:
		 status = None
	
	return status 

def fwd():
	# print("Forward motion called. CTL: ",ctl)
	motor.dcCONFIG(ctl,FL,"cw",50,100.0)
	motor.dcCONFIG(ctl,FR,"cw",50,100.0)
	motor.dcCONFIG(ctl,RL,"cw",50,100.0)
	motor.dcCONFIG(ctl,RR,"cw",50,100.0)
	
	motor.dcSTART(ctl,FL)
	motor.dcSTART(ctl,FR)
	motor.dcSTART(ctl,RL)
	motor.dcSTART(ctl,RR)
	
	status = "forward"
	return status

def speed(FLspeed,FRspeed,RLspeed,RRspeed):
	#print("{0}: {1}: {2}: {3}".format(round(LFspeed,2),round(RFspeed,2), round(LRspeed,2),round(RRspeed,2)))
	motor.dcSPEED(ctl,FL,FLspeed)
	motor.dcSPEED(ctl,FR,FRspeed)
	motor.dcSPEED(ctl,RL,RLspeed)
	motor.dcSPEED(ctl,RR,RRspeed)

#Function to handle pygame events
def PygameHandler(events):
    # Variables accessible outside this function
    global hadEvent
    global moveUp
    global moveDown
    global moveLeft
    global moveRight
    global moveQuit
    global button12
    global nJoyX
    global nJoyY
    
    # Handle each event individually
    for event in events:
        if event.type == pygame.QUIT:
            # User exit
            hadEvent = True
            moveQuit = True
        elif event.type == pygame.KEYDOWN:
            # A key has been pressed, see if it is one we want
            hadEvent = True
            if event.key == pygame.K_ESCAPE:
                moveQuit = True
        elif event.type == pygame.KEYUP:
            # A key has been released, see if it is one we want
            hadEvent = True
            if event.key == pygame.K_ESCAPE:
                moveQuit = False
        elif event.type == pygame.JOYAXISMOTION:
            # A joystick has been moved, read axis positions (-1 to +1)
            hadEvent = True
            nJoyX    = float(joystick.get_axis(axisUpDown))
            nJoyY    = float(joystick.get_axis(axisLeftRight))
            button12 = joystick.get_button(12)
            # Invert any axes which are incorrect
            if axisUpDownInverted:
                nJoyX = -nJoyX
            if axisLeftRightInverted:
                nJoyY = -nJoyY
            # Determine Up / Down values
            if nJoyX < -0.1:
                moveUp   = True
                moveDown = False
            elif nJoyX > 0.1:
                moveUp   = False
                moveDown = True
            else:
                moveUp   = False
                moveDown = False
            # Determine Left / Right values
            if nJoyY < -0.1:
                moveLeft  = True
                moveRight = False
            elif nJoyY > 0.1:
                moveLeft  = False
                moveRight = True
            else:
                moveLeft  = False
                moveRight = False      

# Settings for JoyStick
leftDrive  = FL                         # Drive number for left motor
rightDrive = FR                         # Drive number for right motor
axisUpDown = 3                          # Joystick axis to read for up / down position
axisUpDownInverted = False              # Set this to True if up and down appear to be swapped
axisLeftRight = 2                       # Joystick axis to read for left / right position
axisLeftRightInverted = False           # Set this to True if left and right appear to be swapped
interval = 0.25                         # Time between keyboard updates in seconds, smaller responds faster but uses more processor time

# Setup pygame and key states
global hadEvent
global moveUp
global moveDown
global moveLeft
global moveRight
global moveQuit
global button12
global rotation 
global direction
global status

direction = "stopped" # Forward / Backwards / Left / Right

# OUTPUTS
nMotMixL = 0.0          # Motor (left)  mixed output           (-100..+100)
nMotMixR = 0.0          # Motor (right) mixed output           (-100..+100)

# CONFIG
# - fPivYLimt  : The threshold at which the pivot action starts
#                This threshold is measured in units on the Y-axis
#                away from the X-axis (Y=0). A greater value will assign
#                more of the joystick's range to pivot actions.
#                Allowable range: (0..+99)
fPivYLimit = 15

			
# TEMP VARIABLES
nMotPremixL = 0.0    # Motor (left)  premixed output        (-100..+99)
nMotPremixR = 0.0    # Motor (right) premixed output        (-100..+99)
nPivSpeed   = 0      # Pivot Speed                          (-100..+99)
fPivScale   = 0.0    # Balance scale b/w drive and pivot    (   0..1   )

hadEvent  = True
moveUp    = False
moveDown  = False
moveLeft  = False
moveRight = False
moveQuit  = False
button12  = False
rotation  = 'cw'

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
screen = pygame.display.set_mode([300,300])
pygame.display.set_caption("JoyBorg - Press [ESC] to quit")

motor = MOTOR
global ctl
ctl = 1

print("Reset and initalize controller {0} and motors".format(ctl))

resetCtl()
status = initMotor('ccw','ccw','ccw','ccw')
print(status)
sleep(2)

#
try:
    print('Press [ESC] to quit')
    # Loop indefinitely
    while True:
        # Get the currently pressed keys on the keyboard
        PygameHandler(pygame.event.get())
        if button12:
            print("Exiting Motor Drivers") 
            status = MotorOff()
            break

        # Calculate Drive Turn output due to Joystick X input
        if (nJoyY >= 0): 
        # Forward
            if (nJoyX >=0): nMotPremixL = (99.0)
            else: nMotPremixL = (99 + nJoyX)           
            if (nJoyX >=0): nMotPremixR = (99.0 -nJoyX)
            else: nMotPremixR = 99.0
        else: 
        # Reverse
           if (nJoyX >=0):
               nMotPremixL = (99.0 -nJoyX)
           else:
                nMotPremixLR = 99.0            
           if (nJoyX >=0):
               nMotPremixR = (99.0)
           else:
                nMotPremixR = (99 + nJoyX)                       

        # Scale Drive output due to Joystick Y input (throttle)
        nMotPremixL = nMotPremixL * nJoyY/100.0
        nMotPremixR = nMotPremixR * nJoyY/100.0

        # Now calculate pivot amount
        # - Strength of pivot (nPivSpeed) based on Joystick X input
        # - Blending of pivot vs drive (fPivScale) based on Joystick Y input
        nPivSpeed = nJoyX
        fPivScale = (abs(nJoyY)>fPivYLimit) if  0.00 else (1.00 - abs(nJoyY)/fPivYLimit)

        # Calculate final mix of Drive and Pivot
        nMotMixL = (1.0-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed)
        nMotMixR = (1.0-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed)
                        
        if((nMotMixL >= -99 or nMotMixL <= -.02 )and nMotMixR > 0.2 ):
            #status = fwd()
            if(status != 'forward'):
                status = initMotor('ccw','ccw','ccw','ccw')
                motor.dcCONFIG(ctl,FL,"ccw",nMotMixL*-100,100.0)
                motor.dcCONFIG(ctl,FR,"ccw",nMotMixR* 100,100.0)
                motor.dcCONFIG(ctl,RL,"ccw",nMotMixL*-100,100.0)
                motor.dcCONFIG(ctl,RR,"ccw",nMotMixR* 100,100.0)
                motor.dcSTART(ctl, FL)
                motor.dcSTART(ctl, FR)
                status='forward'
            speed(nMotMixL*-100,nMotMixL*-100,nMotMixR*100 ,nMotMixR*100 )
            status = 'forward'
            
        elif(nMotMixL > 0.2 and nMotMixR >= -99):
            if(status != 'backwards'):
                status = initMotor('cw','cw','cw','cw')
                motor.dcCONFIG(ctl,FL,'cw',nMotMixL* 100,100.0)
                motor.dcCONFIG(ctl,FR,'cw',nMotMixR*-100,100.0)
                motor.dcCONFIG(ctl,RL,'cw',nMotMixL* 100,100.0)
                motor.dcCONFIG(ctl,RR,'cw',nMotMixR*-100,100.0)
                motor.dcSTART(ctl,FL)
                motor.dcSTART(ctl,FR)
                status='backwards'
            speed(nMotMixL*100,nMotMixL*100,nMotMixR*-100 ,nMotMixR*-100 )
            status = 'backwards'
        
        elif((nMotMixL > -6.6 and nMotMixL < 0) and (nMotMixR > -6.6 and nMotMixR < 0)):
            if(status != 'left'):
                status = initMotor('cw','ccw','cw','ccw')
                motor.dcCONFIG(ctl,FL,'cw' ,nMotMixL* -1000,100.0)
                motor.dcCONFIG(ctl,FR,'ccw',nMotMixR* -1000,100.0)
                motor.dcCONFIG(ctl,RL,'cw' ,nMotMixL* -1000,100.0)
                motor.dcCONFIG(ctl,RR,'ccw',nMotMixR* -1000,100.0)
                motor.dcSTART(ctl,FL)
                motor.dcSTART(ctl,FR)
                motor.dcSTART(ctl,RL)
                motor.dcSTART(ctl,RR)
                status = 'left'
            speed(nMotMixL*-1000,nMotMixL*-1000,nMotMixR*-1000 ,nMotMixR*-1000 )
            status = 'left'
                
        elif((nMotMixL < 6.6 and nMotMixL > 0) and (nMotMixR < 6.6 and nMotMixR > 0)):
            if(status != 'right'):
                status = initMotor('ccw','cw','ccw','cw')
                motor.dcCONFIG(ctl,FL,'ccw',nMotMixL* 1000,100.0)
                motor.dcCONFIG(ctl,FR,'cw' ,nMotMixR* 1000,100.0)
                motor.dcCONFIG(ctl,RL,'ccw',nMotMixL* 1000,100.0)
                motor.dcCONFIG(ctl,RR,'cw' ,nMotMixR* 1000,100.0)
                motor.dcSTART(ctl,FL)
                motor.dcSTART(ctl,FR)
                motor.dcSTART(ctl,RL)
                motor.dcSTART(ctl,RR)
                status = 'right'
            speed(nMotMixL*1000,nMotMixL*1000,nMotMixR*1000 ,nMotMixR*1000 )
            status = 'right'


            status = 'right'

        elif(nMotMixL == 0.0 and nMotMixR == 0.0 and status != None ):
           status = MotorOff()
        
        if(status != None):                        
            print("{4} : MotMixL: {0}    MotMixR: {1}  PivSpeed: {2} PivScale: {3}".format(round(nMotMixL*100,2),round(nMotMixR*100,2),round(abs(nPivSpeed)*100,2),round(fPivScale,2),status))    
        
        time.sleep(interval)
                
    
    # Disable all drives
    #status = MotorOff()
except KeyboardInterrupt:
    # CTRL+C exit, disable all drives
    status = MotorOff()
