#!/usr/bin/python3
from featureDetect import *
from gyro import Gyro

def readInstructionFile():
    file = open("instructions.txt", "r")
    instructList = []
    for line in file:
        instructList.append(line.split())
    return instructList

def sendTargetPosition(x,y,r,d):
    pass
    #TODO actually send this stuff over

def sendCurrentPosition(x,y,r):
    pass
    #TODO actually send this stuff over

def isAtGoal(g, c):
    pass
    #TODO check if current state is within bounds of goal state

def executeInstuctions(instructList):
    for instuct in instructList:
        name = instruct[0]
        if name == "Dump":
            color = instruct[1]
            #TODO add dump instructions
        else:
            x,y,r = instruct[1], instruct[2], instruct[3]
            if name == "DRB":
                d = -1
            else:
                d = 1
            sendTargetPosition(x,y,r,d)
            #TODO wait for arduino
            if len(instruct) > 4:
                atGoal = False
                check = instruct[4]
                d = 0
                while not atGoal:
                    if check == "toCross":
                        #TODO do the toCross checker
                        x_c, y_c, r_c = featureDectector.checkCross()
                    elif check == "toFork":
                        refAngle = instruct[5]
                        #TODO do the toFork checker
                    elif check == "toLine":
                        #TODO do the toLine checker
                        refAngle = instruct[5]
                    else:
                        pass
                        #TODO do the toL checker
                    
                    #TODO adjust feature detection values to adjust for
                    #     orientation
                        
                    sendCurrentPosition(x_c,y_c,r_c)#wait

                    atGoal = isAtGoal((x,y,r), (x_c, y_c, r_c))

if __name__ == "__main__":
    fd = FeatureDetector(30,debug=True)
    g  = Gyro(0x68)
    
    while True:
        fd.crossDetect()
        #fd.lineDetect("horizontal")
        #fd.LDetect()
