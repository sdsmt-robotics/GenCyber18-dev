from __future__ import print_function, division
from struct import pack, unpack
from time import sleep as wait

import serial
import serial.tools.list_ports

ROBOT_VID = 0x0403
ROBOT_PID = 0x6001
ROBOT_PID2 = 0x6015

HANDSHAKE  = 0x77
END_FLAG   = 0x33

DRIVE_FLAG = 0x45
LEFT_FLAG  = 0x36
RIGHT_FLAG = 0x35
LIGHTS_FLAG = 0x30

IR_READ_FLAG = 0x27
IR_POS_FLAG  = 0x28

class Robot(object):
    """
        A Robot class to interact with a Trossen Robotics Geekbot.
    """
    def __init__(self, baud, file=None):
        """
            Connect and initialize a Geekbot. Connects to a Geekbot via serial using the given
            baudrate. If the serial port is not given, attempt to find it.
        """
        self.port = serial.Serial()
        if file is not None:
            self.location = file
        else:
            self.location = self.find_robot()
        self.port.baudrate = baud
        self.port.port = self.location
        self.port.timeout = 1
        self.port.dtr = 1
        self.connected = False
        if self.location is not None:
            self.port.open()
            wait(2)
            self.port.write(chr(HANDSHAKE).encode())
            wait(0.5)
            while self.port.read() != chr(HANDSHAKE).encode():
                print("Waiting for handshake")
                # wait(0.5)
            self.connected = True

    @staticmethod
    def find_robot():
        """
            Attempts to find a serial port with a Geekbot connected.
        """
        print("Finding Geekbot's USB port...")
        port_list = serial.tools.list_ports.comports()
        for i in port_list:
            print(i.device)
            if (i.vid == ROBOT_VID) and (i.pid == ROBOT_PID or i.pid == ROBOT_PID2):
                print("Geekbot found: " + i.device)
                return str(i.device)
        print("No Geekbot found!")
        return None

    def shutdown(self):
        """
            Stops all motion and disconnects from the Geekbot.
        """
        self.halt()
        self.port.close()
        self.connected = False

    @staticmethod
    def map_short(num):
        """
            Maps the given number, within 0 and 100, to an int16_t.

            Example:

            >>> from geekbot import Robot
            >>> Robot.map_short(100)
            32767
            >>> Robot.map_short(0)
            0
            >>> Robot.map_short(-100)
            -32767
        """
        temp = (num * 32767) // 100
        if temp > 32767:
            return 32767
        elif temp < -32767:
            return -32767
        return int(temp)

    @staticmethod
    def pack_short(num):
        """
            Packs the given int16_t into a struct for use with a serial object.
        """
        return pack("h", int(num))

    def send_cmd(self,flag, data):
        self.port.write(chr(flag).encode())
        self.port.write(self.pack_short(self.map_short(data)))

    def lights_on(self):
        self.send_cmd(LIGHTS_FLAG, 0x01)

    def lights_off(self):
        self.send_cmd(LIGHTS_FLAG, 0x00)

    def halt(self):
        self.send_cmd(DRIVE_FLAG, 0)

    def turn(self, speed, seconds=None):
        self.send_cmd(LEFT_FLAG, -speed)
        self.send_cmd(RIGHT_FLAG, speed)
        if seconds != None:
            wait(seconds)
            self.halt()
        return

    def drive_forward(self, speed, adjust=None, seconds=None):
        if adjust == None:
            self.send_cmd(DRIVE_FLAG, speed)
        else:
            self.drive_left_wheel(speed)
            adjusted = speed+adjust
            if adjusted > 100:
                self.drive_right_wheel(100)
            elif adjusted < 0:
                self.drive_right_wheel(0)
            else:
                self.drive_right_wheel(adjusted)
        if seconds == None:
            return
        wait(seconds)
        self.halt()


    def drive_backward(self, speed, adjust=None, seconds=None):
        if adjust == None:
            self.send_cmd(DRIVE_FLAG, -speed)
        else:
            self.drive_left_wheel(-speed)
            adjusted = speed+adjust
            if   adjusted > 100:
                self.drive_right_wheel(-100)
            elif adjusted < 0:
                self.drive_right_wheel(0)
            else:
                self.drive_right_wheel(-(adjusted))
        if seconds == None:
            return
        wait(seconds)
        self.halt()

    def drive_right_wheel(self, speed):
        self.send_cmd(RIGHT_FLAG, -speed)

    def drive_left_wheel(self, speed):
        self.send_cmd(LEFT_FLAG, -speed)

    def get_ir_distance(self):
        self.send_cmd(IR_READ_FLAG, 1)
        data = self.port.read(2)
        dist = unpack(">H", data)
        return dist[0]

    def set_ir_position(self, angle):
        self.send_cmd(IR_POS_FLAG, angle)
