#!/usr/bin/python3
import smbus
import struct
from gyro import Gyro
import time as t

class NavToDrive:
    BUS_NUM = 1

    SEND_TARG_POS = 0
    SEND_CUR_POS = 1
    SEND_GYRO_ROT = 2

    def __init__(self, addr):
        self.addr = addr
        self.bus = smbus.SMBus(NavToDrive.BUS_NUM)

    def sendTargPos(self, pos):
        NavToDrive.validatePos(pos)
        pos = [int(pos[i] * 100) for i in range(3)]

        msg = NavToDrive.serializeMsg(list(pos))
        self.bus.write_block_data(self.addr, NavToDrive.SEND_TARG_POS, msg)

    def sendCurPos(self, pos):
        NavToDrive.validatePos(pos)
        pos = [int(pos[i] * 100) for i in range(3)]

        msg = NavToDrive.serializeMsg(list(pos))
        self.bus.write_block_data(self.addr, NavToDrive.SEND_CUR_POS, msg)

    def sendRotation(self, rot):
        rot = int(round(rot*100))
        msg = NavToDrive.serializeMsg([rot])
        self.bus.write_block_data(self.addr, NavToDrive.SEND_GYRO_ROT, msg)

    def checkDone(self):
        msg = self.bus.read_byte(self.addr) 
        return (msg == 1 or msg == 129)
        
    def validatePos(pos):
        if type(pos) not in [tuple, list] or len(pos) != 3:
            raise ValueError("A position must be a 3-element list or tuple")
        
    def serializeMsg(msg):
        return [b for num in msg for b in struct.pack('>h', num)]


# Use this to debug this connection
if __name__ == "__main__":
    to_drive = NavToDrive(0x42)
    g = Gyro(0x68)
    try:
        while True:
            try:
                to_send = int(input("update target or pos? (0, 1): "))
                raw_vals = input("values to send: ").split(" ")

                if to_send == NavToDrive.SEND_TARG_POS:
                    *pos, d = [int(i) for i in raw_vals]
                    to_drive.sendTargPos(pos, d)
                elif to_send == NavToDrive.SEND_CUR_POS:
                    pos = [int(i) for i in raw_vals]
                    to_drive.sendCurPos(pos)
                else:
                    raise ValueError("Invalid number of args") 

                while not to_drive.checkDone():
                    try:
                        t.sleep(0.05)
                        rot = g.get_z_rotation()
                        print(rot)
                        to_drive.sendRotation(rot)
                    except IOError as e:
                        print (e)
                
                print("DONE WITH MOVE!")

            except ValueError as e:
                print(e)
                print("Please enter 3-4 integers")
                continue

            print(to_drive.checkDone())

    except KeyboardInterrupt:
        pass

