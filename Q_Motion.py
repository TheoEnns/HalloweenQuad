import os
import dynamixel
import sys
import subprocess
import math
import time
from serial_tools import serial_ports

class Cerebellum():
    def __init__(self):
        self.sIDMap =   {"ForeLeftA":9,   "ForeLeftB":2,   "ForeLeftR":4, \
                         "ForeRightA":6,  "ForeRightB":16, "ForeRightR":10, \
                         "BackLeftA":13,  "BackLeftB":5,   "BackLeftR":17, \
                         "BackRightA":15, "BackRightB":7,  "BackRightR":12}
        self.sNameMap = {v:k for k,v in self.sIDMap.iteritems()}
        self.sPol =     {"ForeLeftA":-1,  "ForeLeftB":1,   "ForeLeftR":-1, \
                         "ForeRightA":-1, "ForeRightB":1,  "ForeRightR":-1, \
                         "BackLeftA":-1,  "BackLeftB":1,   "BackLeftR":1, \
                         "BackRightA":-1, "BackRightB":1,  "BackRightR":1}
        self.actuatorMap = {}
    def setup(self, scan=False):
        settings = {}
        settings['port'] = serial_ports()[0]
        settings['baudRate'] = 1000000

        serial = dynamixel.SerialStream(port=settings['port'],
                                        baudrate=settings['baudRate'],
                                        timeout=1)
        # Instantiate our network object
        net = dynamixel.DynamixelNetwork(serial)
        settings['servoIds'] = self.sIDMap.values()

        # Populate our network with dynamixel objects
        count = 0
        if scan:
            for servoId in settings['servoIds']:
                newDynamixel = dynamixel.Dynamixel(servoId, net)
                net._dynamixel_map[servoId] = newDynamixel
                print "Found ", servoId, " successfully!"
                if servoId in self.sIDMap.values():
                    count += 1

            if not net.get_dynamixels():
                print 'No Dynamixels Found!'
                sys.exit(0)
            else:
                print "...Done"
        else:
            count = len(self.sIDMap)

        # Prompt the user to move servos.
        for actuator in net.get_dynamixels():
            actuator.moving_speed = 50
            actuator.torque_enable = True
            actuator.torque_limit = 800
            actuator.max_torque = 800
            actuator.goal_position = 512
            self.actuatorMap[actuator.id] = actuator

        # Send all the commands to the servos.
        net.synchronize()
        self.net = net

        if count == len(self.sIDMap):
            print("Mecha-Mercy Ready To Romp!")
            return True
        else:
            print("Mecha-Mercy still has a sleepy limb!")
            return False

    def move_all_motors_degrees(self, degrees, speed, wait = False):
        self.move_degrees(self.sIDMap.values(),degrees, speed, wait)

    def move_all_motors_radians(self, radians, speed, wait = False):
        self.move_radians(self.sIDMap.values(),radians, speed, wait)

    def wait_for_all_stop(self):
        time.sleep(0.01)
        stopped = False
        while(not stopped):
            stopped = True
            for actuator in self.net.get_dynamixels():
                stopped = stopped and not actuator.moving
            time.sleep(0.01)

    def move_degrees(self, ids, degrees, speed=50, wait = False):
        self.move_radians(ids, math.radians(degrees), speed, wait)

    def move_radians(self, ids, radians, speed=50, wait = False):
        angle = (180/150.0)*radians/math.pi*512
        for id in ids:
            actuator = self.actuatorMap[id]
            actuator.goal_position = 512 + self.sPol[self.sNameMap[id]]* (int)(angle)
            actuator.moving_speed = speed
        self.net.synchronize()
        if wait:
            self.wait_stop(ids)

    def multi_move_degrees(self, ids, moves, speed=50, wait = False):
        moves = {k, math.radians}
        self.multi_move_radians(ids, moves, speed, wait)

    def multi_move_radians(self, moves, speed=50, wait = False):
        for id in moves.keys():
            angle = (180/150.0)*moves[id]/math.pi*512
            actuator = self.actuatorMap[id]
            actuator.goal_position = 512 + self.sPol[self.sNameMap[id]]* (int)(angle)
            actuator.moving_speed = speed
        self.net.synchronize()
        if wait:
            self.wait_stop(moves.keys())

    def wait_stop(self, ids):
        time.sleep(0.01)
        stopped = False
        while(not stopped):
            stopped = True
            for id in ids:
                actuator = self.actuatorMap[id]
                stopped = stopped and not actuator.moving
            time.sleep(0.001)

    def twitch(self,ids):
        self.move_degrees(ids,15, 200, True)
        self.move_degrees(ids,0, 200, True)

if __name__ == "__main__":
    cb = Cerebellum()
    if(cb.setup(True)):
        cb.wait_for_all_stop()
        print "Move +15 Degrees"
        cb.move_all_motors_degrees(15, 50, True)
        print "Move -15 Degrees"
        cb.move_all_motors_degrees(-15, 50, True)
        print "Move 0 Degrees"
        cb.move_all_motors_degrees(0, 50, True)

        macro_opts = ["Fore","Back","Right","Left","All"]
        options = cb.sIDMap.values() + cb.sIDMap.keys() + macro_opts
        while(True):
            answer = raw_input(\
                """Press: 
                    - Q to quit 
                    - on of these motors to twitch: """ + str(cb.sIDMap.values()) + """
                                                    """ + str(cb.sIDMap.keys()) + """
                                                    """ + str(macro_opts) + """
                    """)
            try:
                if answer == "Q":
                    break
                elif str.isdigit(answer) and int(answer) in cb.sIDMap.values():
                    print "Move #", answer, " Twitch"
                    cb.twitch([int(answer)])
                elif answer in cb.sIDMap.keys():
                    print "Move ", answer, " Twitch"
                    cb.twitch([cb.sIDMap[answer]])
                elif answer in macro_opts:
                    if answer == "All":
                        filtered_list = cb.sIDMap.values()
                    else:
                        filtered_list = {cb.sIDMap[k] for k in cb.sIDMap.keys() if answer in k}
                    cb.twitch(filtered_list)
            except:
                pass
