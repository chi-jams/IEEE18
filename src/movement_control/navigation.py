#!/usr/bin/python3
from featureDetect import *
from gyro import Gyro
from navToDrive import NavToDrive
import time as t

DIST_THRESH  = 0.25
ANGLE_THRESH = 3.0
PIXEL_TO_INCH = 1/160.0

def readInstructionFile():
    file = open("instructions.txt", "r")
    instructList = []
    for line in file:
        instructList.append(line.split())
    return instructList

def alignToCross(targ_pos):
    #get camera-aligned errors
    errors = fd.checkCross()
    errors = pixelsToInches(errors)
    if withinThreshold(errors):
        return True #aligned

    #correct to global pos
    targ_rot = targ_pos[2]
    if targ_rot == 0:
        errors = [-errors[1], errors[0], errors[2]]
    elif targ_rot == 90:
        pass
    elif targ_rot == 180:
        errors = [errors[1], -errors[0], errors[2]]
    elif targ_rot == 270:
        errors = [-errors[0], -errors[1], errors[2]]
    else:
        raise ValueError("unrecognized orientation for cross")

    cur_pos = [targ_pos[i] + errors[i] for i in range(3)]

    #send current position and wait for navduino to execute
    navPi.sendRotation(g.get_z_rotation())
    navPi.sendCurrentPosition(cur_pos)
    while not navPi.checkDone():
        navPi.sendRotation(g.get_z_rotation())
    return False #need to check again

def alignToFork(targ_pos, orientation):
    #get camera-aligned errors
    errors = fd.checkFork(orientation)
    errors = pixelsToInches(errors)
    if withinThreshold(errors):
        return True #aligned

    #TODO correct to global pos
    if   orientation == 'up':
        pass
    elif orientation == 'down':
        pass
    elif orientation == 'up-left':
        pass
    elif orientation == 'up-right':
        pass
    elif orientation == 'down-left':
        pass
    elif orientation == 'down-right':
        pass
    else:
        raise ValueError("unrecognized orientation for fork")
    
    
    cur_pos = [targ_pos[i] + errors[i] for i in range(3)]

    #send current position and wait for navduino to execute
    navPi.sendRotation(g.get_z_rotation())
    navPi.sendCurrentPosition(cur_pos)
    while not navPi.checkDone():
        navPi.sendRotation(g.get_z_rotation())
    return False #need to check again


def alignToLine(targ_pos, orientation):
    errors = fd.checkLine(orientation)
    errors = pixelsToInches(errors)

    #correct to global pos
    targ_rot = targ_pos[2]
    if targ_rot == 0:
        errors = [0, -errors[1], errors[2]]        
    elif targ_rot == 90:
        pass
    elif targ_rot == 180:
        pass
    elif targ_rot == 270:
        errors = [0, -errors[1], errors[2]]        
    else:
        raise ValueError("unrecognized orientation for line")

    cur_pos = [targ_pos[i] + errors[i] for i in range(3)]

    #send current position and wait for navduino to execute
    navPi.sendRotation(g.get_z_rotation())
    navPi.sendCurrentPosition(cur_pos)
    while not navPi.checkDone():
        navPi.sendRotation(g.get_z_rotation())
    return False #need to check again

def alignToL(targ_pos):
    errors = fd.checkL()
    errors = pixelsToInches(errors)

    cur_pos = [targ_pos[i] + errors[i] for i in range(3)]

    #send current position and wait for navduino to execute
    navPi.sendRotation(g.get_z_rotation())
    navPi.sendCurrentPosition(cur_pos)
    while not navPi.checkDone():
        navPi.sendRotation(g.get_z_rotation())
    return False #need to check again

def pixelsToInches(errors):
    return [errors[i] * PIXEL_TO_INCH for i in range(2)] + [errors[2]]

def withinThreshold(errors):
    return abs(errors[0]) <  DIST_THRESH and\
           abs(errors[1]) <  DIST_THRESH and\
           abs(errors[2]) < ANGLE_THRESH

def executeInstuctions(instructList):
    for instruct in instructList:
        
        #get instruction type
        name = instruct[0]

        #dump
        if name == "Dump":
            color = instruct[1]
            #TODO add dump instructions
        
        #drive
        else:
            target_pos = [instruct[i] for i in range(1:4)]
            #send target to navduino and wait for completion
            navPi.sendRotation(g.get_z_rotation())
            navPi.sendTargetPosition(target_pos)
            while not navPi.checkDone():
                t.sleep(0.05)
                rot = g.get_z_rotation()
                print(rot)
                navPi.sendRotation(g.get_z_rotation())
            '''
            #if instruction includes camera check
            if len(instruct) > 4:
                check = instruct[4]
                d = 0
                if check == "toCross":
                    while not alignToCross():
                        navPi.sendRotation(g.get_z_rotation())
                elif check == "toFork":
                    orientation = instruct[5]
                    while not alignToFork(orientation):
                        navPi.sendRotation(g.get_z_rotation())
                elif check == "toLine":
                    orientation = instruct[5]
                    while not alignToLine(orientation):
                        navPi.sendRotation(g.get_z_rotation())
                elif check == "toL":
                    while not alignToL():
                        navPi.sendRotation(g.get_z_rotation())
                else:
                    raise ValueError("Unknown check instruction")
            '''
            print ("FINISHED: " + " ".join(instruct))

if __name__ == "__main__":
    fd = FeatureDetector(30,debug=True) #get camera features
    g  = Gyro(0x68)                     #get rotation feedback
    navPi = NavToDrive(0x42)            #send instructions to navduino
    inst = readInstructionFile()        #read instructions.txt
    executeInstructions(inst)           #do instructions
