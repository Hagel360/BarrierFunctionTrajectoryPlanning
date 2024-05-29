"""
A class allows for two different communication modes
1. Allows communication between a virtual crazyradio and the crazyflies
2. Allows communication between crazyflies
The class doesn't differentiate between the two modes, it just sends and receives packets
as the different modes are handled on the crazyradio side. (i.e. the crazyflies don't know)
"""

from controller import Emitter
from controller import Receiver
import json
import numpy as np

class radio_controller:
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.emitter = robot.getDevice("emitter")
        self.receiver = robot.getDevice("receiver")
        self.receiver.enable(self.timestep)

    def send_packet(self, name, packettype, packet):
        # check if packet is a numpy array, convert to list
        if type(packet) == type(np.array([])):
            packet = packet.tolist()
        packet_data = {
            'name': name,
            'type': packettype,
            'data': packet
        }
        packet_str = json.dumps(packet_data)
        self.emitter.send(packet_str)

    def receive_packet(self):
        received_packet_str = self.receiver.getString()
        received_packet = json.loads(received_packet_str)
        name = received_packet['name']
        packetType = received_packet['type']
        packet_data = received_packet['data']
        
        # throw current packet to trash
        Receiver.nextPacket(self.receiver)

        # drone position
        if packetType == "POS":
            x, y, z = packet_data
            return name, "POS", [x, y, z]
        # drone is active
        if packetType == "ACK":
            return name, "ACK", packet_data
        # drone is inactive
        if packetType == "NACK":
            return name, "NACK", packet_data
        # drone trajectory
        if packetType == "TRAJ":
            return name, "TRAJ", packet_data