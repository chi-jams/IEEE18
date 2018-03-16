#!/usr/bin/python3
import smbus

class NavToDrive:
    BUS_NUM = 1

    def __init__(self, addr):
        self.addr = addr
        self.bus = smbus.SMBus(NavToDrive.BUS_NUM)

    def sendTargPos(self, pos, direction):
        msg = [*pos, direction];
        print("Sending %s..." % msg)
        self.bus.write_block_data(self.addr, 0, msg)    

    def sendCurPos(self, pos):
        print("Dude I don't know")

# Use this to debug this connection
if __name__ == "__main__":
    to_drive = NavToDrive(0x42)
    
    try:
        while True:
            raw_vals = input("values to send (pick 4): ").split(" ")
            if len(raw_vals) != 4:
                print("Please input 4 numbers")
                continue

            try:
                *pos, d = [int(i) for i in raw_vals]

                to_drive.sendTargPos(pos, d)
            except ValueError:
                print("Please enter 4 integers")
                continue

    except KeyboardInterrupt:
        pass

