# mediapipe-robot-arm-controller
A demonstration of using MediaPipe Holistic to create a controller for a robot arm from the shoulder to fingers. 

# Installation

I would recommend creating a Python virtual environment first:

### Mac and Linux
```
$ python3 -m venv mp_env && source mp_env/bin/activate
```
Then, install the requirements.txt as follows:
```
$ pip install -r requirements.txt
```

### Windows
```
python -m venv mp_env
.\mp_env\Scripts\activate
```
Then, install the requirements.txt as follows:
```
$ pip install -r requirements.txt
```



# Running the Demo

To launch the demo script:
```
$ python controller.py
```

To see command line options:
```
$ python controller.py help
```

## Camera Selection and Preview Window
The default behaviour is to look at your machine to see if there is a DepthAI powered device (OAK-D Camera) first and to try initialize capture at 4K resolution. If an OAK-D is not detected, the script will fall back to using the first webcam on the system detected by OpenCV at a default 1080P resolution 

The preview window defaults to a 1280x720 resolution regardless of the resolution of the input camera, so don't use this as a reference of the capture size. This is to make it a bit easier to manage on your desktop.

These values can all be overriden via command-line parameters as follows:

- **oakd-capture-width** (3840, 1920) - Sets the horizontal resolution of the capture (note this must match the vertical, either 4K or 1080P)
- **oakd-capture-height** (2160, 1080) - Sets the vertical resolution of the capture (note this must match the horizontal, either 4K or 1080P)
- **webcam-capture-width** (Any valid value for your system)
- **webcam-capture-height** (Any valid value for your system)
- **preview-width** (Width of the preview window on the desktop)
- **preview-height** (Height of the preview window on the desktop)
- **force-webcam** (Ignore any DepthAI devices present and use the system webcam)

Examples:

To launch DepthAI in 1080P mode instead of 4K
```
$ python controller.py --oakd-capture-width 1920 --oakd-capture-height 1080
```

To force webcam use at 720P
```
$ python controller.py --force-webcam --webcam-capture-width 1280 --webcam-capture-height 720
```

## Serial Communication
The demo contains the code from the original demo which packs the 23 DOF angles into an integer array and transmits over a serial port of choice. Because the demo won't run without a proper serial port, this function can be disabled while running.

- **disable-serial** - Disables the serial comms
- **serial-port** - Sets the serial port (defaults to COM15 as per original demo)

The demo defaults to a serial transmission rate of 10hz (fps). This can be adjusted as follows:

- **serial-fps XX** - Sets the transmission rate in frames per second

Example:

To launch with serial enabled:
```
$ python controller.py --enable-serial --serial-port COM5
```

### Serial enumeration utility
There is a python script provided for enumerating serial devices on the system. To see available devices:
```
$ python list_serial_ports.py
```

This will display a result similar to this
```
COM4 - Standard Serial over Bluetooth link (COM4)
COM3 - Standard Serial over Bluetooth link (COM3)
```

## Socket Communication to Isaac Sim
This demo can also stream angles to an demo script running in Isaac Sim on the same machine via a local TCP socket (defaults to using localhost / 127.0.0.1, port 65432). The script will attempt a socket connection when it is executed. If the demo is running on Isaac a connection will be established, otherwise the comms will be disabled automatically. So, you should be able to run with or without the companion Isaac demo. 

However, if you wish to disable this socket connection attempt, you can turn it off with the following flag:

- **disable-socket** - Disables the socket comms



## Low Pass Filter
By default, a low pass filter is applied to the data read from the skeleton joints to reduce jitter. The amount of filtering can be overriden with a command line parameter:

- **lpf-value 0.XX** - Adjusts the LPF Coefficient

LPF can be entirely disabled by setting this value to 1.0. Any value less than that will blend the current input frame with previous in the ratio specified. For example 0.25 = 25% current frame, 75% historial frame. 



# What am I seeing in the Demo Visualization
The demo will open with a video view (note this is mirrored so that you can move more organically) which displays computed values for the skeletal pose (right shoulder down to right hand currently). 

It will also open 3 planar projection views for X=0 (Side View), Y=0 (Top View), and Z=0 (Front View).

In those views, the joints being used for the calculations will be highlighted:

- Cyan (Elbow Calculation)
- Magenata (Shoulder Yaw Calculation)
- Yellow (Shoulder Pitch Calculation)

## Runtime hotkeys
With the demo active, you can press the following keys to perform actions:

- **ESC** - Quit the demo
- **d** - Toggle debug views on and off


# Isaac Sim Script and Integration (Experimental)
**The instructions below are my best current understanding of how Isaac Sim works and how projects are structured. I am certain this is probably wrong in many ways, and we'll discover better ways to package and distribute packages, but this should at least get the demo running for exploration**

The instructions below assume running Isaac Sim on Windows. I will update Linux instructions down the road. 

## Step 1: Identify where Isaac Sim is installed

Installing and launching the App will require the path to the Isaac Sim installation on your system. In a typical install, this will be something like this:

### Windows
```
C:\Users\tshum\AppData\Local\ov\pkg\isaac_sim-2022.2.1
```

### Linux
```
~/.local/share/ov/pkg/isaac_sim-2022.2.1
```

## Step 2: Using python.bat to run Isaac scripts

### Windows
Isaac Sim has it's own Python interpreter that needs to be used to launch scripts interacting with the Sim. This file is located in the root of the Isaac Sim folder. For windows, it is called **python.bat**. You will run this from your Isaac Sim root folder:
```
cd C:\Users\tshum\AppData\Local\ov\pkg\isaac_sim-2022.2.1
python.bat <python script>
```

### Linux
```
cd ~/.local/share/ov/pkg/isaac_sim-2022.2.1/
./python.sh <python script>
```


## Step 3: Symbolic link to the demo script

The easiest way to keep the script in sync with changes from GitHub is to create a symbolic link of the script into the ```extension_examples\user_examples``` folder of your Isaac install. There are likely better ways to package this up in the future, but for now, we just install it as an example. 

You will want to symlink the ```isaac-sim\hope_demo.py``` script from your GitHub repo into this folder. 

### Windows

This can be done on Windows as follows (from the root of your Isaac Sim install folder as above). **Replace the folder in [] with your local github folder.**
```
C:\Users\tshum\AppData\Local\ov\pkg\isaac_sim-2022.2.1>
mklink /H extension_examples\user_examples\hope_demo.py [c:\dev\mediapipe-robot-arm-controller]\isaac-sim\hope_demo.py
```
If the link is successful, windows should output something like this, but with your paths:
```
Hardlink created for extension_examples\user_examples\hope_demo.py <<===>> c:\dev\mediapipe-robot-arm-controller-trs\isaac-sim\hope_demo.py
```

### Linux
**Replace the folder in [] with your local github folder.**
```
ln -s [~/Dev/mediapipe-robot-arm-controller/isaac-sim/hope_demo.py] extension_examples/user_examples/hope_demo.py
```

## Step 4: Edit the script to point at your data
The top of the hope_demo.py file contains the path to the HOPE USD file - you will need to edit this to match where your GitHub project lives on your local filesystem:

### Windows
Edit the first part of the path below to point to the location you cloned the GitHub Project to set the USDPath variable to match your data file and save the script.

```
# Configuration
USDPath = "C:/Dev/mediapipe-robot-arm-controller/isaac-sim/Collected_HOPE1_02/HOPE1_02.usd"
```

### Linux
Edit the first part of the path below (inside the []) to point to the location you cloned the GitHub Project to
```
# Configuration
USDPath = "[/home/tshumay]/mediapipe-robot-arm-controller/isaac-sim/Collected_HOPE1_02/HOPE1_02.usd"
```


## Step 5: Run the demo

Now, from your root Isaac Sim folder, you can run the demo script as follows:

### Windows
```
C:\Users\tshum\AppData\Local\ov\pkg\isaac_sim-2022.2.1>python.bat extension_examples/user_examples/hope_demo.py
```

### Linux
```
./python.sh extension_examples/user_examples/hope_demo.py
```

This should open an Isaac Sim window. After that, you can launch the MediaPipe demo (**controller.py**) just as you normally do, but it should indicate that it has established a socket connection when it runs. **You should use your regular python venv for the MediaPipe demo to run it and not python.bat as it requires the dependencies from that venv, so they will be run in separate command prompts**

Note that if you don't have a physical robot connection, you should specify **--disable-serial** to avoid having the program hang waiting on serial.
```
(mp_env) C:\Dev\mediapipe-robot-arm-controller-trs>python controller.py [--disable-serial]

attempting socket connection to ('127.0.0.1', 65432)
Socket connected to HOPE Demo
```


