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

            TODO: What is the Arduino code the Geekbot is running? Is it custom? Default provided
            by Trossen? This needs to be documented.
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
        # An int16_t in native byte order (little-endian).
        return pack("h", int(num))

    def send_cmd(self, flag, data):
        """
            Send a generic command to the connected Geekbot.

            data - The command value to pass to the Geekbot. Specified by each command flag.
            flag - The command flag to send. One of the following:

                DRIVE_FLAG   - Drive both wheels with equal direction and speed
                LEFT_FLAG    - Control the left wheel
                RIGHT_FLAG   - Control the right wheel

                data - An integer value between -100 and 100 whose sign indicates direction, and
                        whose magnitude indicates speed.

                LIGHTS_FLAG  - Control the robot's LED
                data - True: Light on. False: Light off.

                IR_POS_FLAG  - Set the angle of the IR distance sensor
                TODO: Verify
                data - An angle between -360 and 360. Angle is measured from the front of the robot,
                        angles moving right, and negative angles moving left.
                TODO: Verify that 0x1 is a placeholder and doesn't have any special meaning.
                IR_READ_FLAG - Request an update from the IR distance sensor
                data - A placeholder (0x1) should be given.
        """
        self.port.write(chr(flag).encode())
        self.port.write(self.pack_short(self.map_short(data)))

    def lights_on(self):
        """Turn on the Robot's LED"""
        self.send_cmd(LIGHTS_FLAG, 0x01)

    def lights_off(self):
        """Turn off the Robot's LED"""
        self.send_cmd(LIGHTS_FLAG, 0x00)

    def halt(self):
        """Stop the Robot in its tracks."""
        self.send_cmd(DRIVE_FLAG, 0)

    def turn(self, speed, seconds=None):
        """
            Turns the Robot.

            TODO: Make note of special Robotics terminology for this type of locomotion.

            speed - The speed at which to turn the robot. May be between -100 and 100.
            seconds - The amount of time the turn should take. May be fractional.
        """
        self.send_cmd(LEFT_FLAG, -speed)
        self.send_cmd(RIGHT_FLAG, speed)

        if seconds is not None:
            wait(seconds)
            self.halt()

    def drive_forward(self, speed, adjust=0, seconds=None):
        """
            Drives the Robot forward.

            TODO: Make note of special Robotics terminology for this type of locomotion.

            speed   - The speed at which the Robot should drive. An integer between -100 and 100.
            adjust  - The speed by which the right wheel should differ from the left wheel. Optional
            seconds - The amount of time you wish the Robot to drive forward. Optional. If not
                      given, the Robot will drive forward until instructed otherwise.
        """
        self.drive_left_wheel(speed)
        # self.send_cmd() already handles range overflows, so just add the adjustment.
        self.drive_right_wheel(speed + adjust)

        if seconds is not None:
            wait(seconds)
            self.halt()

    def drive_backward(self, speed, adjust=0, seconds=None):
        """
            Drives the Robot backward.

            TODO: Make note of special Robotics terminology for this type of locomotion.

            speed   - The speed at which the Robot should drive. An integer between -100 and 100.
            adjust  - The speed by which the right wheel should differ from the left wheel. Optional
            seconds - The amount of time you wish the Robot to drive backward. Optional. If not
                      given, the Robot will drive backward until instructed otherwise.
        """
        # Negating both speed and adjust is equivalent to adding, and then negating the result.
        self.drive_forward(-speed, -adjust, seconds)

    def drive_right_wheel(self, speed):
        """Drives the right wheel indefinitely at the given speed."""
        self.send_cmd(RIGHT_FLAG, -speed)

    def drive_left_wheel(self, speed):
        """Drives the left wheel indefinitely at the given speed."""
        self.send_cmd(LEFT_FLAG, -speed)

    def get_ir_distance(self):
        """Gets the distance measured by the Robot's IR sensor."""
        self.send_cmd(IR_READ_FLAG, 1)
        data = self.port.read(2)
        # A uint16_t, in big-endian byte order.
        dist = unpack(">H", data)
        return dist[0]

    def set_ir_position(self, angle):
        """Sets the angle (in degrees) the Robot's IR sensor is pointing."""
        self.send_cmd(IR_POS_FLAG, angle)
