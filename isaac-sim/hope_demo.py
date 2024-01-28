# Configuration
USDPath = "/home/tshumay/Dev/mediapipe-robot-arm-controller/isaac-sim/Collected_HOPE1_02/HOPE1_02.usd"


#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import omni
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.dynamic_control import _dynamic_control

import numpy as np

import socket
import selectors
import types
import struct

# Set up the socket for comms to the MediaPipe process
sel = selectors.DefaultSelector()

def accept_wrapper(sock):
    conn, addr = sock.accept()  # Should be ready to read
    print("accepted connection from", addr)
    conn.setblocking(False)
    data = types.SimpleNamespace(addr=addr, inb=b"", outb=b"")
    events = selectors.EVENT_READ | selectors.EVENT_WRITE
    sel.register(conn, events, data=data)

receive_buffer = bytearray()
def service_connection(key, mask):
    sock = key.fileobj
    data = key.data
    if mask & selectors.EVENT_READ:
        try:
            recv_data = sock.recv(1024)  # Should be ready to read
            
            if recv_data:
                # Commands are packets of 23 angles, each angle is a byte
                #print('received', len(recv_data), 'bytes from', data.addr)
                #print(recv_data)
                receive_buffer.extend(recv_data)
                while len(receive_buffer) >= 23:
                    # Send the first 23 bytes to the unpacker
                    unpackAngleDataToSim(receive_buffer[:23])
                    del receive_buffer[:23]
                #data.outb += recv_data
            else:
                print('closing connection to', data.addr)
                sel.unregister(sock)
                sock.close()
        except ConnectionResetError:
            print('closing connection to', data.addr)
            sel.unregister(sock)
            sock.close()

    if mask & selectors.EVENT_WRITE:
        if data.outb:
            print('echoing', repr(data.outb), 'to', data.addr)
            sent = sock.send(data.outb)  # Should be ready to write
            data.outb = data.outb[sent:]

# Adapter that takes byte data from the socket and converts it to angles for the simulator controls
def unpackAngleDataToSim(angleBytes):
    global dof_angles

    #print('Unpacking angle data:', angleBytes)
    unpacked = np.array(struct.unpack('23B', angleBytes))

    # This is a little messy, because we need to go through a remapping process to convert the angles
    # from the values output for the hardware into the values for the joint angles in the simulator.
    # In the future, once proper controls are established, these should be unified and this step
    # will not be necessary.

    # Shoulder (pitch, yaw, roll = angles 19,20,21)
    dof_angles[0] = -np.deg2rad(unpacked[19]) # Revolute 1
    dof_angles[1] = -np.deg2rad(unpacked[20]) # Revolute 2
    dof_angles[2] = np.deg2rad(180-unpacked[21]) # Revolute 3

    # Elbow
    dof_angles[3] = np.deg2rad(unpacked[22]) # Revolute 4

    # Wrist (pitch, yaw roll = angles 16,17,18)
    dof_angles[4] = np.deg2rad(unpacked[18]-180)  # Revolute 5
    dof_angles[5] = np.deg2rad(unpacked[16]-90)  # Revolute 6
    dof_angles[6] = np.deg2rad(unpacked[17]-90)  # Revolute 7
    
    # dof_angles[7] is the fixed wrist joint, and shouldn't be animated

    # dof_angles[8] is base of thumb
    dof_angles[8] = np.deg2rad(unpacked[13])  # Revolute 9
    
    # TBD - The math coming out of the demo controller for these joints 
    # is wrong. They are disabled for now.
    # 9,10,11,12 are the base of the fingers
    # dof_angles[9] = np.deg2rad(unpacked[1]-90) # Revolute 14
    # dof_angles[10] = np.deg2rad(unpacked[4]-90) # Revolute 18
    # dof_angles[11] = np.deg2rad(90-unpacked[7]) # Revolute 22
    # dof_angles[12] = np.deg2rad(90-unpacked[10]) # Revolute 26
    
    # dof_angles[13] is thumb roll
    dof_angles[13] = np.deg2rad(unpacked[13])  # Revolute 10

    # dof_angles[14-17] are first knuckle joints
    dof_angles[14] = np.deg2rad(180-unpacked[0])    # Revolute 15
    dof_angles[15] = np.deg2rad(180-unpacked[3])    # Revolute 19
    dof_angles[16] = np.deg2rad(180-unpacked[6])    # Revolute 23
    dof_angles[17] = np.deg2rad(180-unpacked[9])     # Revolute 27

    # dof_angles[18] is thumb first joint
    dof_angles[18] = np.deg2rad(180-unpacked[12])  # Revolute 11

    # dof_angles[19-22] are second knuckle joints
    dof_angles[19] = np.deg2rad(180-unpacked[2])    # Revolute 16
    dof_angles[20] = np.deg2rad(180-unpacked[5])    # Revolute 20
    dof_angles[21] = np.deg2rad(180-unpacked[8])    # Revolute 24
    dof_angles[22] = np.deg2rad(unpacked[11]-180)     # Revolute 28 - Angular inversion in the model for some reason?

    # dof_angles[23] is thumb second joint
    dof_angles[23] = np.deg2rad(180-unpacked[14])  # Revolute 12
    
    # dof_angles[24-27] are finger tips - driven from same angles as second knuckles
    dof_angles[24] = np.deg2rad(180-unpacked[2])    # Revolute 17
    dof_angles[25] = np.deg2rad(180-unpacked[5])    # Revolute 21
    dof_angles[26] = np.deg2rad(180-unpacked[8])    # Revolute 25
    dof_angles[27] = np.deg2rad(unpacked[11]-180)     # Revolute 29 - Angular inversion in the model for some reason?

    # dof_angles[28] is thumb tip - driven from same angle as thumb first joint
    dof_angles[28] = np.deg2rad(180-unpacked[14])  # Revolute 13

 
host = '127.0.0.1'
port = 65432
lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
lsock.bind((host, port))
lsock.listen()
print('listening on', (host, port))
lsock.setblocking(False)
sel.register(lsock, selectors.EVENT_READ, data=None)

# Load the HOPE stage
open_stage(USDPath)
stage=omni.usd.get_context().get_stage()

# Set up the world
world = World()
# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()

# Locate the robot
rootPath = '/World/Complete_Arm_20/Complete_Arm_20'
root = stage.GetObjectAtPath(rootPath)

# TS - I am not certain of this strategy as this was a Bing code suggestion
# and not something I could find reference to on the Isaac Sim documentation.
# But it seems to work, so I am rolling with it for now.
dc = _dynamic_control.acquire_dynamic_control_interface()
articulation = dc.get_articulation(rootPath) # Replace Robot with your robot name
dc.wake_up_articulation(articulation)

# Get information about the structure of the articulation
num_joints = dc.get_articulation_joint_count(articulation)
num_dofs = dc.get_articulation_dof_count(articulation)
num_bodies = dc.get_articulation_body_count(articulation)

print("Number of joints: ", num_joints)
print("Number of DOFs: ", num_dofs)
print("Number of bodies: ", num_bodies)

# Initial state
dof_angles = np.zeros(num_dofs).astype(np.float32)

ticker = -2*np.pi
angle = 13

while True:
    
    # Socket comms
    events = sel.select(timeout=0)
    for key, mask in events:
        if key.data is None:
            accept_wrapper(key.fileobj)
        else:
            service_connection(key, mask)

    # Useful for testing DOF ranges
    #ticker += 0.15
    #if ticker > 2*np.pi:
    #    ticker = -2*np.pi
    #    angle += 1
    #    if angle > 28:
    #        angle = 13
    #    print('Angle', angle)
    #dof_angles[angle] = ticker
    

    # Update pose
    dc.set_articulation_dof_position_targets(articulation, dof_angles)

    # Step the simulation
    # we have control over stepping physics and rendering in this workflow
    # things run in sync
    world.step(render=True) # execute one physics step and one rendering step

simulation_app.close() # close Isaac Sim