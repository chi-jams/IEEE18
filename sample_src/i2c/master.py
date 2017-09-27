#!/usr/bin/python3

import smbus

MOTOR_BUS = 1
MOTOR_ADDR = 0x28

bus = smbus.SMBus(MOTOR_BUS)

msg = 'a'
while (msg != '0'):
    msg = input("What do?")
    print(type(msg))
    bus.write_byte(MOTOR_ADDR, int(msg))

