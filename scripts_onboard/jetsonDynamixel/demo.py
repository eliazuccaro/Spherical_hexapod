#! /usr/bin/env python3

import os
from sys import argv, path, exit
from time import sleep
from random import randint

# path.append("../ax12")
from ax12 import Ax12


N_MOTORS = 1


# check user privileges
userId = os.getuid()
if userId != 0:
    exit("You aren't root, I need superuser privileges to interface with GPIOs")


# convert from [-150; +150]deg to 0-1023
def deg2bin(angle_deg):
    angle_bin = int(angle_deg * 1024.0 / 300.0) + 512
    if angle_bin < 0:
        return 0
    if angle_bin > 1023:
        return 1023
    return angle_bin


# convert from 0-1023 to [-150; +150]deg
def bin2deg(angle_bin):
    return angle_bin * 300.0 / 1024.0 - 150


if __name__ == "__main__":
    mServos = Ax12()

    for motorId in range(1, N_MOTORS+1):
        # print("Actual position: " + str(mServos.readPosition(motorId)))
        print("I: [ID " + str(motorId) + "] Setting motor velocity to a quarter")

        try:
            mServos.moveSpeed(motorId, 512, 256)
        except Ax12.timeoutError as e:
            print("W: Motor " + str(motorId) + " seems to be unreachable")
            pass

    while 1:
        for motorId in range(1, N_MOTORS+1):
            try:
                if mServos.readMovingStatus(motorId) != 1:
                    next_pos_deg = randint(-150, 150)  # positive counterclockwise
                    next_pos_bin = deg2bin(next_pos_deg)

                    print("I: [ID " + str(motorId) + "] Going at " \
                        + str(next_pos_deg) + " degrees")

                    mServos.move(motorId, next_pos_bin)
                    mServos.setLedStatus(motorId, 1)
                else:
                    mServos.setLedStatus(motorId, 0)
            except Ax12.timeoutError as e:
                # something as gone wrong reading the reply,
                # think about resending the command
                print(e)
                pass

            sleep(0.05)  # seconds

    # set ID
    # if len(argv) > 1:
    #     motorId = argv[1]
    #     for i in range(254):
    #         sleep(100)
    #         try:
    #             mServos.setID(i, motorId)
    #         except:
    #             print("error resetting id from "+str(i)+" to "+str(motorId))
    #         else:
    #             print("set id to "+str(motorId))
    #             exit(1)
    # else:
    #     print("ERROR")
    #     print("usage: python setMotorId motorNumber")
