#!/usr/bin/python3
import smbus
import struct

class NavToDrive:
    BUS_NUM = 1

    SEND_TARG_POS = 0
    SEND_CUR_POS = 1
    CHECK_DONE = 2

    def __init__(self, addr):
        self.addr = addr
        self.bus = smbus.SMBus(NavToDrive.BUS_NUM)

    def sendTargPos(self, pos, direction):
        NavToDrive.validatePos(pos)

        if type(direction) != int:
            raise TypeError("direction must be an integer")
        elif direction not in [0,1]:
            raise ValueError("Direction must be a 1 or 0")

        msg = NavToDrive.serializeMsg(list(pos) + [direction])
        self.bus.write_block_data(self.addr, NavToDrive.SEND_TARG_POS, msg)

    def sendCurPos(self, pos):
        NavToDrive.validatePos(pos)

        msg = NavToDrive.serializeMsg(list(pos))
        self.bus.write_block_data(self.addr, NavToDrive.SEND_CUR_POS, msg)

    def checkDone(self):
        msg = self.bus.read_byte(self.addr) 
        return msg == 0
        
    def validatePos(pos):
        if type(pos) not in [tuple, list] or len(pos) != 3:
            raise ValueError("A position must be a 3-element list or tuple")
        
    def serializeMsg(msg):
        return [b for num in msg for b in struct.pack('>h', num)]


# Use this to debug this connection
if __name__ == "__main__":
    to_drive = NavToDrive(0x42)
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

            except ValueError as e:
                print(e)
                print("Please enter 3-4 integers")
                continue

            print(to_drive.checkDone())

    except KeyboardInterrupt:
        pass

