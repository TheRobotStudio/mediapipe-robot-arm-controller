import cv2
import mediapipe as mp
import numpy as np
from copy import deepcopy
import argparse
import opencv_cam
import depthai_cam
import struct
import time
import socket
import selectors
import types


mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose
mp_hand = mp.solutions.hands
mp_holistic = mp.solutions.holistic


# Socket comms handlers for connection to Isaac Sim
sel = selectors.DefaultSelector()

def start_connection(host, port):
    addr = (host, port)
    print('attempting socket connection to', addr)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
      sock.connect(addr)
      print("Socket connected to HOPE Demo")
    except ConnectionRefusedError:
      print('ERROR: Socket connection to demo refused. Ensure HOPE Demo is running.')
      print('Disabling socket comms for this session.')
      args.disable_socket = True
    

    sock.setblocking(False)
    events = selectors.EVENT_READ | selectors.EVENT_WRITE
    data = types.SimpleNamespace(addr=addr, inb=b'', outb=b'')
    sel.register(sock, events, data=data)

def service_connection(key, mask):
    sock = key.fileobj
    data = key.data

    if mask & selectors.EVENT_READ:
        recv_data = sock.recv(1024)  # Should be ready to read
        if recv_data:
            print('received', repr(recv_data), 'from connection', data.addr)
            data.outb += recv_data
        else:
            print('closing connection to', data.addr)
            sel.unregister(sock)
            sock.close()
    if mask & selectors.EVENT_WRITE:
        if data.outb:
            print('sending', repr(data.outb), 'to connection', data.addr)
            sent = sock.send(data.outb)  # Should be ready to write
            data.outb = data.outb[sent:]



def visibilityToColour(vis):
  if (vis < 0.5):
    return (0, 0, 255)  # Red - low visibility
  elif (vis < 0.75):
    return (0,255,255)  # Yellow - medium visibility
  else:
    return (0, 255, 0)  # Green - high visibility
  
def angle(a, b, c):
    # https://stackoverflow.com/questions/35176451/python-code-to-calculate-angle-between-three-point
    # -using-their-3d-coordinates
    # a, b and c : points as np.array([x, y, z]) 
    ba = a - b
    bc = c - b
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)

    return np.degrees(angle)

def landmark_to_np(landmark):
  # Create a numpy array of the landmark positions
  landmark_np = np.array([landmark.x, landmark.y, landmark.z])
  return landmark_np

# Calculate a rotation matrix that will take a vector and rotate it so that Y points up
def calculate_y_up_matrix(v):
   # Normalize the vector
    v = v / np.linalg.norm(v)

    # Compute the rotation axis
    axis = np.cross(v, np.array([0.0, 1.0, 0.0]))
    axis = axis / np.linalg.norm(axis)

    # Compute the rotation angle
    angle = -np.arccos(np.dot(v, np.array([0.0, 1.0, 0.0])))
    
    # Compute the rotation matrix using the axis-angle representation
    axis_crossproduct_matrix = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])

    # Compute final rotation matrix
    rotation_matrix = (
        np.eye(3) +
        np.sin(angle) * axis_crossproduct_matrix +
        (1 - np.cos(angle)) * np.dot(axis_crossproduct_matrix, axis_crossproduct_matrix)
    )

    return rotation_matrix


  
def calculate_pose_angles(pose_world_landmarks, arm=0):
  # Grab our points of interest for easy access
  if arm == 0:
    elbow = pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW]
    wrist = pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]
    shoulder = pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    opp_shoulder = pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
    hip = pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP]
  else:
    elbow = pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW]
    wrist = pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST]
    shoulder = pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
    opp_shoulder = pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    hip = pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP]

  # Calculate the angle of the right elbow joint in the Z=0 plane (Front View)
  elbow_angle = angle(
     np.array([shoulder.x, shoulder.y]), 
     np.array([elbow.x, elbow.y]),
     np.array([wrist.x, wrist.y]))
  
  # Invert angle to match robot arm 170 by trial and error
  elbow_angle = 170.0 - elbow_angle

  #Calculate pitch in the X=0 plane (side view)
  shoulder_pitch = angle(np.array([hip.z, hip.y]), np.array([shoulder.z, shoulder.y]),
                               np.array([elbow.z, elbow.y]))
  # Calculate shoulder Yaw in 3d
  shoulder_yaw = angle(np.array([opp_shoulder.x, opp_shoulder.y, opp_shoulder.z]), np.array([shoulder.x,
                shoulder.y, shoulder.z]), np.array([elbow.x, elbow.y, elbow.z])) - 90

  # Calculate roll by difference between yaw and angle to wrist from shoulders
  shoulder_roll = angle(np.array([opp_shoulder.x, opp_shoulder.y, opp_shoulder.z]),
                                                   np.array([shoulder.x, shoulder.y, shoulder.z]),
                                                   np.array([wrist.x, wrist.y, wrist.z])) -\
                        angle(np.array([opp_shoulder.x, opp_shoulder.y, opp_shoulder.z]),
                              np.array([shoulder.x, shoulder.y, shoulder.z]),
                              np.array([elbow.x, elbow.y, elbow.z])) + shoulder_pitch / 3

  return elbow_angle,shoulder_yaw,shoulder_pitch,shoulder_roll

def calculate_finger_angles(joint_angles, joint_xyz):

  # First finger, fore or index
  # Angles calculated correspond to knuckle flex, knuckle yaw and long tendon length for all fingers,
  # note difference in knuckle yaw for little
  joint_angles[0] = angle(joint_xyz[0], joint_xyz[5], joint_xyz[8])
  joint_angles[1] = angle(joint_xyz[9], joint_xyz[5], joint_xyz[6])
  joint_angles[2] = angle(joint_xyz[5], joint_xyz[6], joint_xyz[7])
  #print(int(joint_angles[0]), int(joint_angles[1]), int(joint_angles[2]))

  # Second finger, middle
  joint_angles[3] = angle(joint_xyz[0], joint_xyz[9], joint_xyz[12])
  joint_angles[4] = angle(joint_xyz[13], joint_xyz[9], joint_xyz[10])
  joint_angles[5] = angle(joint_xyz[9], joint_xyz[10], joint_xyz[11])
  #print(joint_angles[3], joint_angles[4], joint_angles[5])

  # Third finger, ring
  joint_angles[6] = angle(joint_xyz[0], joint_xyz[13], joint_xyz[16])
  joint_angles[7] = angle(joint_xyz[9], joint_xyz[13], joint_xyz[14])
  joint_angles[8] = angle(joint_xyz[13], joint_xyz[14], joint_xyz[15])
  #print(joint_angles[6], joint_angles[7], joint_angles[8])

  # Fourth finger, pinky
  joint_angles[9] = angle(joint_xyz[0], joint_xyz[17], joint_xyz[20])
  joint_angles[10] = angle(joint_xyz[13], joint_xyz[17], joint_xyz[18])
  joint_angles[11] = angle(joint_xyz[17], joint_xyz[18], joint_xyz[19])
  #print(joint_angles[9], joint_angles[10], joint_angles[11])

  # Thumb, bit of a guess for basal rotation might be better automatic
  joint_angles[12] = angle(joint_xyz[1], joint_xyz[2], joint_xyz[3])
  joint_angles[13] = angle(joint_xyz[2], joint_xyz[1], joint_xyz[5])
  joint_angles[14] = angle(joint_xyz[2], joint_xyz[3], joint_xyz[4])
  joint_angles[15] = angle(joint_xyz[9], joint_xyz[5], joint_xyz[2])
  #print(joint_angles[12], joint_angles[13], joint_angles[14], joint_angles[15])

  return joint_angles


def drawDebugViews(results, hand_points, hcp, hncp, hand_points_norm):
  
  # Create images for the 3 planar projection views
  window_size = 256
  xaxis = np.zeros((window_size, window_size, 3), np.uint8)
  xaxis[:] = (0, 0, 64)
  yaxis = np.zeros((window_size, window_size, 3), np.uint8)
  yaxis[:] = (0, 64, 0)
  zaxis = np.zeros((window_size, window_size, 3), np.uint8)
  zaxis[:] = (64, 0, 0)

  # Draw planar projection views for debugging
  if results.pose_world_landmarks is not None:
      last = None
      names = ['RWri','RElb','RSho','RHip', 'LHip', 'LSho', 'LElb', 'LWri']
      joints = [
        mp_pose.PoseLandmark.RIGHT_WRIST, 
        mp_pose.PoseLandmark.RIGHT_ELBOW, 
        mp_pose.PoseLandmark.RIGHT_SHOULDER, 
        mp_pose.PoseLandmark.RIGHT_HIP, 
        mp_pose.PoseLandmark.LEFT_HIP, 
        mp_pose.PoseLandmark.LEFT_SHOULDER, 
        mp_pose.PoseLandmark.LEFT_ELBOW,
        mp_pose.PoseLandmark.LEFT_WRIST]

      # Put all the world landmark positions for the joints into numpi array
      world_landmarks = np.array([[results.pose_world_landmarks.landmark[i].x, results.pose_world_landmarks.landmark[i].y, results.pose_world_landmarks.landmark[i].z] for i in joints])

      # Center the landmarks in the window
      world_landmarks += 0.5

      # Scale the landmarks to fit in the window
      world_landmarks *= window_size

      # Estimate center of torso
      cp = (world_landmarks[2]+world_landmarks[4])/2.0

      # Compute the normal to the center of the torso
      normal = np.cross(world_landmarks[3]-world_landmarks[2],world_landmarks[4]-world_landmarks[2])
      normal /= np.linalg.norm(normal)
      ncp = cp+(normal*20.0)

      # To bump the rendering down a bit to use the window better
      yoffset = int(window_size*.25)

      # To integers
      world_landmarks = world_landmarks.astype(int)
      cp = cp.astype(int)
      ncp = ncp.astype(int)

      for idx in range(len(world_landmarks)):
          landmark = world_landmarks[idx]
          #print(i, landmark)
          cv2.circle(zaxis, (landmark[0], landmark[1]+yoffset), 2, (255, 255, 255), -1)
          cv2.circle(yaxis, (landmark[0], landmark[2]+yoffset), 2, (255, 255, 255), -1)
          cv2.circle(xaxis, (landmark[2], landmark[1]+yoffset), 2, (255, 255, 255), -1)
          cv2.putText(zaxis, names[idx], (landmark[0], landmark[1]+yoffset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
          cv2.putText(yaxis, names[idx], (landmark[0], landmark[2]+yoffset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
          cv2.putText(xaxis, names[idx], (landmark[2], landmark[1]+yoffset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
          if last is not None:
              cv2.line(zaxis, (landmark[0], landmark[1]+yoffset), (last[0], last[1]+yoffset), (255, 255, 255), 1)
              cv2.line(yaxis, (landmark[0], landmark[2]+yoffset), (last[0], last[2]+yoffset), (255, 255, 255), 1)
              cv2.line(xaxis, (landmark[2], landmark[1]+yoffset), (last[2], last[1]+yoffset), (255, 255, 255), 1)
          last = landmark

          # Draw debug info for the shoulder pitch
          cv2.line(xaxis, (world_landmarks[2][2], world_landmarks[2][1]+yoffset), (world_landmarks[1][2], world_landmarks[1][1]+yoffset), (0, 255, 255), 2)

          # Draw a yellow line between the sholder and the hip
          cv2.line(xaxis, (world_landmarks[2][2], world_landmarks[2][1]+yoffset), (world_landmarks[3][2], world_landmarks[3][1]+yoffset), (0, 255, 255), 2)

          # Draw debug info for the elbow angle
          # Draw a cyan line between the shoulder and the elbow
          cv2.line(zaxis, (world_landmarks[2][0]+2, world_landmarks[2][1]+yoffset+2), (world_landmarks[1][0]+2, world_landmarks[1][1]+yoffset+2), (255, 255, 0), 2)
          # Draw a cyan line between the elbow and the wrist
          cv2.line(zaxis, (world_landmarks[1][0], world_landmarks[1][1]+yoffset), (world_landmarks[0][0], world_landmarks[0][1]+yoffset), (255, 255, 0), 2)

          # Draw debug info for the shoulder yaw
          # Draw a magenta line between the shoulder and the hip
          cv2.line(zaxis, (world_landmarks[2][0], world_landmarks[2][1]+yoffset), (world_landmarks[3][0], world_landmarks[3][1]+yoffset), (255, 0, 255), 2)
          # Draw a magenta line between the shoulder and the elbow
          cv2.line(zaxis, (world_landmarks[2][0], world_landmarks[2][1]+yoffset), (world_landmarks[1][0], world_landmarks[1][1]+yoffset), (255, 0, 255), 2)

          # Draw torso center in each view
          cv2.circle(zaxis, (cp[0], cp[1]+yoffset), 2, (255, 255, 0), -1)
          cv2.circle(yaxis, (cp[0], cp[2]+yoffset), 2, (255, 255, 0), -1)
          cv2.circle(xaxis, (cp[2], cp[1]+yoffset), 2, (255, 255, 0), -1)

          # Draw normal line
          cv2.line(zaxis, (cp[0], cp[1]+yoffset), (ncp[0], ncp[1]+yoffset), (255, 255, 0), 2)
          cv2.line(yaxis, (cp[0], cp[2]+yoffset), (ncp[0], ncp[2]+yoffset), (255, 255, 0), 2)
          cv2.line(xaxis, (cp[2], cp[1]+yoffset), (ncp[2], ncp[1]+yoffset), (255, 255, 0), 2)


      if results.right_hand_landmarks is not None:

        # Translate the points for rendering in center of screen
        hand_points += 0.5
        hcp += 0.5
        hncp += 0.5
        hand_points_norm *= 0.5 # Scale down the normalized points
        hand_points_norm += 0.5

        # Scale the landmarks to fit in the window
        hand_points *= window_size
        hcp *= window_size
        hncp *= window_size
        hand_points_norm *= window_size

        # To integers for OpenCV drawing
        hand_points = hand_points.astype(int)
        hncp = hncp.astype(int)
        hcp = hcp.astype(int)
        hand_points_norm = hand_points_norm.astype(int)

        # Draw hand points in each view, with unrotated hand in lower right of window
        for i in range(21):
          cv2.circle(zaxis, (hand_points[i][0], hand_points[i][1]+yoffset), 2, (255, 255, 255), -1)
          cv2.circle(yaxis, (hand_points[i][0], hand_points[i][2]+yoffset), 2, (255, 255, 255), -1)
          cv2.circle(xaxis, (hand_points[i][2], hand_points[i][1]+yoffset), 2, (255, 255, 255), -1)

          cv2.circle(zaxis, (hand_points_norm[i][0]+100, hand_points_norm[i][1]+100), 2, (0, 255, 255), -1)
          cv2.circle(yaxis, (hand_points_norm[i][0]+100, hand_points_norm[i][2]+100), 2, (0, 255, 255), -1)
          cv2.circle(xaxis, (hand_points_norm[i][2]+100, hand_points_norm[i][1]+100), 2, (0, 255, 255), -1)

        # Draw hand center in each view
        cv2.circle(zaxis, (hcp[0], hcp[1]+yoffset), 2, (255, 255, 0), -1)
        cv2.circle(yaxis, (hcp[0], hcp[2]+yoffset), 2, (255, 255, 0), -1)
        cv2.circle(xaxis, (hcp[2], hcp[1]+yoffset), 2, (255, 255, 0), -1)

        # Draw coordinate system for hand center
        cols = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]
        for i,pt in enumerate(hncp):
            cv2.line(zaxis, (hcp[0], hcp[1]+yoffset), (pt[0], pt[1]+yoffset), cols[i], 2)
            cv2.line(yaxis, (hcp[0], hcp[2]+yoffset), (pt[0], pt[2]+yoffset), cols[i], 2)
            cv2.line(xaxis, (hcp[2], hcp[1]+yoffset), (pt[2], pt[1]+yoffset), cols[i], 2)


  # Show the debug views
  cv2.imshow('YZ Plane (Side View)',xaxis)
  cv2.imshow('XZ Plane (Top View)',yaxis)
  cv2.imshow('XY Plane (Front View)',zaxis)


  # Move the windows over to the left side of the main window and stack them vertically
  # Only on the first apparence of the debug views, in case the user overrides the positions
  if not hasattr(drawDebugViews, "views_moved"):
    cv2.moveWindow('YZ Plane (Side View)', 0, 512)
    cv2.moveWindow('XZ Plane (Top View)', 0, 256)
    cv2.moveWindow('XY Plane (Front View)', 0, 0)
    drawDebugViews.views_moved = True


def transmit_angles_serial(ser,arm,joint_angles):

  # This code is basically verbatim from the original demo

  # Generate checksum
  sum = np.sum(joint_angles)
  sum = sum & 0x000000FF
  t_xchecksum = 255 - sum
  #print(sum, t_xchecksum)

  # Initialise bus with two A
  command = B'\xFE'
  ser.write(command)
  ser.write(command)

  # Write a 0 for right arm, and 0x80 for left arm
  if (arm == 0):
    ser.write(B'\x00')
  else:
    ser.write(B'\x80')
  
  packed_data = struct.pack('23B', *joint_angles)
  ser.write(packed_data)
  #packed_data = struct.pack('B', t_xchecksum)
  #ser.write(packed_data)
  #ser.flushOutput()

  #print(ser.out_waiting)

  #time.sleep(delay)



  #packed_data = struct.pack('2B', joint_angles[0], joint_angles[1])
  #ser.write(packed_data)
  #ser.flushOutput()

  #time.sleep(delay)

  #packed_data = struct.pack('2B', joint_angles[2], joint_angles[3])
  #ser.write(packed_data)
  #ser.flushOutput()
  #time.sleep(delay)

  #packed_data = struct.pack('2B', joint_angles[4], joint_angles[5])
  #ser.write(packed_data)
  #ser.flushOutput()
  #time.sleep(delay)

  #packed_data = struct.pack('2B', joint_angles[6], joint_angles[7])
  #ser.write(packed_data)
  #ser.flushOutput()
  #time.sleep(delay)

  #packed_data = struct.pack('2B', joint_angles[8], joint_angles[9])
  #ser.write(packed_data)
  #ser.flushOutput()
  #time.sleep(delay)

  #packed_data = struct.pack('2B', joint_angles[10], joint_angles[11])
  #ser.write(packed_data)
  #ser.flushOutput()
  #time.sleep(delay)

  #packed_data = struct.pack('2B', joint_angles[12], joint_angles[13])
  #ser.write(packed_data)
  #ser.flushOutput()
  #time.sleep(delay)

  #packed_data = struct.pack('2B', joint_angles[14], joint_angles[15])
  #ser.write(packed_data)
  #ser.flushOutput()
  #time.sleep(delay)



  # End bus with check sum and two B
  ser.write(struct.pack('B', t_xchecksum))
  command = B'\xFD'
  ser.write(command)
  ser.write(command)

  #time.sleep(delay)

  ser.flushOutput()


# Initialize the timestamp with current time
serial_timestamp = time.time()
serial_muted = False
hold_arm_angles = False

# Periodic serial transmit function - maintains a maximum transmit rate
# specified in arguments to the program
xmit_arm = 0  # We alternate between arms on serial transmissions
def serial_timer_transmit(fps, ser, joint_angles):
  global xmit_arm
  global serial_timestamp
  global serial_muted

  serial_period = 1.0/fps    

  if (time.time() - serial_timestamp) > serial_period:
  
    xmit_angles = np.clip(joint_angles[xmit_arm].astype(int), 0, 255) # Clip to 8 bit values

    if (not args.disable_serial and not serial_muted):
      transmit_angles_serial(ser,xmit_arm,xmit_angles)

    print("Arm: ", xmit_arm)
    print(xmit_angles)
    
    print("Serial FPS: ", 1.0/(time.time()-serial_timestamp))

    # Reset timer    
    serial_timestamp = time.time()

    xmit_arm = 1 - xmit_arm # Toggle between arms


# Periodic serial transmit function - maintains a maximum transmit rate
# specified in arguments to the program
def socket_timer_transmit(joint_angles):
  # Make sure socket comms are enabled
  if (not args.disable_socket):

    joint_angles = joint_angles.astype(int)
    joint_angles = np.clip(joint_angles, 0, 255) # Clip to 8 bit values
    
    
    events = sel.select(timeout=1)
    if events:
      for key, mask in events:

          data = key.data

          # Pack the angles into a byte array          
          data.outb += struct.pack('23B', *joint_angles)
          
          service_connection(key, mask)
    
          print(joint_angles)
    # Check for a socket being monitored to continue.
    #if not sel.get_map():
    #  break

  
# Read command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--nodebug', action='store_true', help='Disable debug views')
parser.add_argument('--force-webcam', action='store_true', help='Force webcam input even if OAKD device is present')
parser.add_argument('--oakd-capture-width', type=int, default=3840, help='Set OAKD capture width (default=3840)')
parser.add_argument('--oakd-capture-height', type=int, default=2160, help='Set OAKD capture height (default=2160)')
parser.add_argument('--webcam-capture-width', type=int, default=1920, help='Set webcam capture width (default=1920)')
parser.add_argument('--webcam-capture-height', type=int, default=1080, help='Set webcam capture height (default=1080)')
parser.add_argument('--preview-width', type=int, default=1280, help='Set preview width (default=1280)')
parser.add_argument('--preview-height', type=int, default=720, help='Set preview height (default=720)')
parser.add_argument('--disable-serial', action='store_true', help='Disable serial port output')
parser.add_argument('--serial-port', type=str, default='COM4', help='Set serial port (default=COM4)')
parser.add_argument('--serial-fps', type=int, default=20, help='Set serial port output frequency (default=20)')
parser.add_argument('--disable-socket', action='store_true', help='Disable socket output')
parser.add_argument('--lpf-value', type=float, default=0.25, help='Low pass filter value (default=0.25). 1.0 = no filtering')
args = parser.parse_args()
show_debug_views = not args.nodebug


# Start serial port if requested
if not args.disable_serial:
  import serial
  ser = serial.Serial(
        port=args.serial_port,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
        timeout=1
    )
else:
  ser = None

# Start socket if requested
if not args.disable_socket:
   host = '127.0.0.1'
   port = 65432
   start_connection(host, port)



# For the camera, we look to see if there is a DepthAI device connected (OAK-D camera) and prefer that by default
# If not, we fall through to webcam

# Start with DepthAI camera
cvcam = depthai_cam.DepthAICam(width=args.oakd_capture_width,height=args.oakd_capture_height) # Default to 4K OAK-D camera
if (parser.parse_args().force_webcam or cvcam.is_depthai_device_available() is False):
   # Fall back to default webcam
   print("No DepthAI device available, falling back to webcam.")
   cvcam = opencv_cam.OpenCVCam(width=args.webcam_capture_width,height=args.webcam_capture_height)

# Start the video strema
if cvcam.start() is False:
  print("Failed to start video capture - exiting.")
  exit()

# Create 2D array with enough space for all calculated angles for left, right arm
joint_angles = np.zeros([2,23])
is_valid_frame = False
data_updated = False    

# Process the video stream
with mp_holistic.Holistic(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as holistic:
  while cvcam.is_opened():

    # Start a frame time counter
    frame_time = cv2.getTickCount()

    success, image = cvcam.read_frame()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # See if it's time to send serial/socket data - Note: We do this in two places to try avoid
    # beat pattern of computation because of the FPS. In the future, this should
    # be threaded or done in a separate process.
    if (data_updated):
      serial_timer_transmit(args.serial_fps, ser, joint_angles)
      socket_timer_transmit(joint_angles[0])
      data_updated = False

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = holistic.process(image)

    # Draw the pose annotation on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    mp_drawing.draw_landmarks(
        image,
        results.pose_landmarks,
        mp_holistic.POSE_CONNECTIONS,
        landmark_drawing_spec=mp_drawing_styles
        .get_default_pose_landmarks_style())
    mp_drawing.draw_landmarks(
        image,
        results.right_hand_landmarks,
        mp_holistic.HAND_CONNECTIONS,
        landmark_drawing_spec=mp_drawing_styles
        .get_default_hand_landmarks_style())
    mp_drawing.draw_landmarks(
        image,
        results.left_hand_landmarks,
        mp_holistic.HAND_CONNECTIONS,
        landmark_drawing_spec=mp_drawing_styles
        .get_default_hand_landmarks_style())


    # Once mediapipe has processed the frame, we can scale it down for display
    image = cv2.resize(image, (args.preview_width,args.preview_height))


    # Calculate a point to approximate the center of the torso at the midpoint between the left shoulder and right hip
    if results.pose_landmarks is not None:
        right_hip = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP]
        left_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
        torso_center = np.array([right_hip.x+left_shoulder.x, right_hip.y+left_shoulder.y, right_hip.z+left_shoulder.z])/2.0

        torso_render = torso_center*np.array([image.shape[1], image.shape[0], 0.0])
        torso_render = torso_render.astype(int)

        cv2.circle(image, (torso_render[0],torso_render[1]), 4, (255, 255, 0), -1)


    # Flip the image horizontally for a selfie-view display.
    flipped_image = cv2.flip(image, 1)
    
    # Stash last set of joint angles for filtering
    prev_joint_angles = joint_angles.astype(np.float32)


    # Process right side, and then left side
    data_updated = False
    for arm in range(2):
       
      hand_points = None
      wrist_rotation = 0.0

      hand_landmarks = None

      if (arm == 0):
        hand_landmarks = results.right_hand_landmarks
      else:
        hand_landmarks = results.left_hand_landmarks
      
      if hand_landmarks is not None:
        data_updated = True
        
        # Create a numpy array of the hand landmarks
        hand_points = np.array([[hand_landmarks.landmark[i].x, hand_landmarks.landmark[i].y, hand_landmarks.landmark[i].z] for i in range(21)])

        # The idea here is to rotate the hand so that the middle finger points up to make it more consistent to pick off
        # angles including the rotation angle around the wrist which isn't easily obtained otherwise.

        # Make a copy of the array for the normalized positio:ns
        hand_points_norm = deepcopy(hand_points)
        hand_points_norm -= hand_points_norm[0] # Move all points relative to the wrist

        # If this is the left hand, flip the X axis so that the hand is oriented the same as the right hand
        if (arm == 1):
          hand_points_norm[:,0] = -hand_points_norm[:,0]

        # Compute up vector for the hand
        normalized_up = hand_points_norm[mp_hand.HandLandmark.WRIST] - hand_points_norm[mp_hand.HandLandmark.MIDDLE_FINGER_MCP]
        normalized_up /= np.linalg.norm(normalized_up)

        # Compute matrix to rotate the hand so that the middle finger points up
        hand_rotation_matrix = calculate_y_up_matrix(normalized_up)

        # Transform the hand points to the new coordinate system
        hand_points_norm = np.matmul(hand_points_norm, hand_rotation_matrix)

        # Use normalized points to calculate hand rotation in the Y=0 plane
        index = hand_points_norm[mp_hand.HandLandmark.INDEX_FINGER_MCP]
        ring = hand_points_norm[mp_hand.HandLandmark.RING_FINGER_MCP]
        xaxis = hand_points_norm[mp_hand.HandLandmark.PINKY_MCP] + np.array([1.0,0.0,0.0])
        rel = index - ring

        # Arm direciton - uses non-normalized points
        arm_direction = hand_points[mp_hand.HandLandmark.INDEX_FINGER_MCP] - hand_points[mp_hand.HandLandmark.WRIST]

        # Depending on which side of the hand the thumb is on, the angle will be positive or negative
        wrist_rotation = angle(
          np.array([index[0], index[2]]),
          np.array([ring[0], ring[2]]),
          np.array([xaxis[0], xaxis[2]]))

        if (rel[2] < 0): # Look at Z axis direction beteen index finger mcp and pinky to determine direction of hand
          
          # This is a bit of a cheat to prevent some bad angles occuring at extreme angles when the 
          # arm is pointed toward the left side of the body
          
          if (arm_direction[0] <= 0): # If hand is pointed to the right, allow full range of motion
            wrist_rotation = -wrist_rotation
          else: # If hand is pointed to the left, avoid weird angles if wrist is over rotated
            if (wrist_rotation < 90.0):
              wrist_rotation = -wrist_rotation
            else:
              wrist_rotation = 179.9  # Clamp

        wrist_rotation = 180+wrist_rotation # Move values to usable positive range

        # Scale degress (0-360) to 8-bit range (0-255)
        wrist_rotation = wrist_rotation/360.0*255.0

        # Calculate finger joint angles
        hand_angles = calculate_finger_angles(joint_angles[arm], hand_points_norm)

        # Calculate wrist angles

        # Model does not seem to be in the same origin as the pose, so we need to translate
        # the hand points to the pose frame of reference if we want to compare them
        if (arm == 0):
          pose_wrist = landmark_to_np(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST])
        else:
          pose_wrist = landmark_to_np(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST])
        
        delta = pose_wrist - hand_points[0]
        hand_points += delta

        # Estimate the center of the palm
        hcp = (hand_points[0]+hand_points[5]+hand_points[17])/3.0

        # Compute the normal to the center of the palm
        hup = hand_points[9]-hand_points[0]
        hup /= np.linalg.norm(hup)

        # Compute the right vector - invert for left hand
        if (arm == 0):
          hright = hand_points[5]-hand_points[17]
        else:
          hright = hand_points[17]-hand_points[5]
        hright /= np.linalg.norm(hright)

        hand_normal = np.cross(hright, hup)
        hand_normal /= np.linalg.norm(hand_normal)

        hncp = np.array([hcp+hright*0.2, hcp+hup*0.2, hcp+hand_normal*0.2])

        # Wrist pitch
        if (arm == 0):
          elbow = landmark_to_np(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW])
        else: 
          elbow = landmark_to_np(results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW])
        
        fk = hand_points[0]+hand_normal
        joint_angles[arm][16] = angle(fk, hand_points[0], elbow)-30.0  # The 30.0 is an empirical fudge factor - I don't know why this angle is offset

        # Use Middle finger calculate wrist yaw
        if (arm == 0):
          wrist_yaw = angle(hand_points[mp_hand.HandLandmark.MIDDLE_FINGER_MCP], hand_points[mp_hand.HandLandmark.WRIST], np.array([1.0,0,0]))
        else:
          wrist_yaw = angle(hand_points[mp_hand.HandLandmark.MIDDLE_FINGER_MCP], hand_points[mp_hand.HandLandmark.WRIST], np.array([-1.0,0,0]))

        joint_angles[arm][17] = 180.0-wrist_yaw

        # Wrist roll
        joint_angles[arm][18] = wrist_rotation
        #print(int(joint_angles[16]), int(joint_angles[17]), int(joint_angles[18]))
      
      # Calculate pose angles
      if results.pose_world_landmarks is not None:
          # Grab our points of interest for easy access
          elbow_angle,shoulder_yaw,shoulder_pitch,shoulder_roll = calculate_pose_angles(results.pose_world_landmarks,arm)

          if not hold_arm_angles:
            joint_angles[arm][19] = shoulder_pitch
            joint_angles[arm][20] = shoulder_yaw
            joint_angles[arm][21] = shoulder_roll + 180
            joint_angles[arm][22] = elbow_angle
      
      # Check to see if the data frame is valid
      is_valid_frame = results.pose_landmarks is not None and hand_landmarks is not None

      # Add annotations after flipping the image
      if is_valid_frame and show_debug_views:

          # Screen points for drawing text
          if arm == 0:
            screen_elbow = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW]
            screen_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
            screen_wrist = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]
          else:
            screen_elbow = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW]
            screen_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
            screen_wrist = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST] 

          # Elbow
          cv2.rectangle(flipped_image, (int(image.shape[1] - screen_elbow.x * image.shape[1]) + 5, int(screen_elbow.y * image.shape[0]) - 15), (int(image.shape[1] - screen_elbow.x * image.shape[1]) + 100, int(screen_elbow.y * image.shape[0]) + 5), (0, 0, 0), -1)
          cv2.putText(flipped_image, "Elb: {:.2f}".format(elbow_angle), (5+int(image.shape[1] - screen_elbow.x * image.shape[1]), int(screen_elbow.y * image.shape[0])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, visibilityToColour(screen_elbow.visibility), 1, cv2.LINE_AA)

          # Wrist
          scaled = wrist_rotation*360.0/255.0
          cv2.rectangle(flipped_image, (int(image.shape[1] - screen_wrist.x * image.shape[1]) + 5, int(screen_wrist.y * image.shape[0]) - 15), (int(image.shape[1] - screen_wrist.x * image.shape[1]) + 200, int(screen_wrist.y * image.shape[0]) + 5), (0, 0, 0), -1)
          cv2.putText(flipped_image, "Wri: {:.2f} Deg: {:.2f}".format(wrist_rotation,scaled), (5+int(image.shape[1] - screen_wrist.x * image.shape[1]), int(screen_wrist.y * image.shape[0])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, visibilityToColour(screen_wrist.visibility), 1, cv2.LINE_AA)
          
          # Shoulder
          cv2.rectangle(flipped_image, (int(image.shape[1] - screen_shoulder.x * image.shape[1]) + 5, int(screen_shoulder.y * image.shape[0]) - 15), (int(image.shape[1] - screen_shoulder.x * image.shape[1]) + 250, int(screen_shoulder.y * image.shape[0]) + 5), (0, 0, 0), -1)
          cv2.putText(flipped_image, "Sh Yaw:{:.2f} Pit:{:.2f} Roll:{:.2f}".format(shoulder_yaw, shoulder_pitch, shoulder_roll), (int(image.shape[1] - screen_shoulder.x * image.shape[1]), int(screen_shoulder.y * image.shape[0])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, visibilityToColour(screen_shoulder.visibility), 1, cv2.LINE_AA)

      # Debug output
      if (is_valid_frame and show_debug_views):
        drawDebugViews(results, hand_points, hcp, hncp, hand_points_norm)

    # New data?
    if (data_updated):
      
      # Apply low pass filter
      joint_angles = (1.0-args.lpf_value)*prev_joint_angles + args.lpf_value*joint_angles

      # Send updated serial data
      serial_timer_transmit(args.serial_fps, ser, joint_angles)

      # Isaac sim only has right arm
      socket_timer_transmit(joint_angles[0])

      data_updated = False


      

    # Show serial status
    if (serial_muted):
      # Draw message at top right of screen to indicate serial is off
      cv2.rectangle(flipped_image, (image.shape[1]-200, 0), (image.shape[1], 40), (0, 0, 0), -1)
      cv2.putText(flipped_image, "Serial: OFF", (image.shape[1]-200, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)

    if (hold_arm_angles):
      # Draw message at top right of screen to indicate serial is off
      cv2.rectangle(flipped_image, (image.shape[1]-200, 0), (image.shape[1], 40), (0, 0, 0), -1)
      cv2.putText(flipped_image, "Arm: Hold", (image.shape[1]-200, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 1, cv2.LINE_AA)


    
    # Calculate the frame rate
    frame_rate = cv2.getTickFrequency() / (cv2.getTickCount() - frame_time)

    # Display frame rate on frame
    cv2.rectangle(flipped_image, (0, 0), (200, 40), (0, 0, 0), -1)
    cv2.putText(flipped_image, "FPS: {:.2f}".format(frame_rate), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)

    # Render view
    cv2.imshow('MediaPipe Pose', flipped_image)


    # Keyboard input
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
      break
    elif key == ord('d'):
      show_debug_views = not show_debug_views
      if (not show_debug_views):
        cv2.destroyWindow('YZ Plane (Side View)')
        cv2.destroyWindow('XZ Plane (Top View)')
        cv2.destroyWindow('XY Plane (Front View)')
    # check for space bar to toggle serial mute
    elif key == 32:
       serial_muted = not serial_muted
    # hold arm angles?
    elif key == ord('a'):
      hold_arm_angles = not hold_arm_angles
       
# Clean up camera and windows
cvcam.stop()
cv2.destroyAllWindows()