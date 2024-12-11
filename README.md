# 890-ros-f24-project-articubot
Repository for F24 ROS Articubot project

# Resources

## Articulated Robotics Project Description

This project is based on the Articulated Robotics (AR) YouTube series
[Building a mobile robot](https://youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&si=RugUUmOhQfiXqiuk)

Additional resources include:
* The corresponding [blog
  posts](https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros)
  on the AR website


This project makes the following types of adaptations from the
original description
* some small changes are needed to get the project to work with Humble
  distribution of ROS2 instead of the original Foxy.
* a number of things had to be modified to use the new version of the
  Gazebo simulator (Ignition Fortress -- which is the version recommended for
  Humble) instead of Gazebo Classic.
* minor changes were made to the hardware elements

## Parts List

The AR Bill of Materials is provide on the [AR
website](https://articulatedrobotics.xyz/tutorials/mobile-robot/project-overview#bill-of-materials).

(For 890 version, place spreadsheet in git repo and insert link)

# Charging the Battery

Plug in the battery to the charger terminals before providing power to prevent sparks.

![Lipo Wiring Image](/ReadMe_Pictures/Lipo_Wire.png)

![Lipo Charge Settings Image](/ReadMe_Pictures/LipoCharge.png)

LiPo Charge - 5.2 amp 11.1v (3S) 
- Long press on Enter.
- Enter again.


# Setup Instructions

1. Install Ubuntu 22.04

    - On a Virutal Machine or Personal Computer (64bit).
    - And on a Raspberry Pi (Still Ubuntu 22.04, use the 64bit version from the raspberry pi imager utility).

2. In both machine, run the following commands in command prompt:

    ```
    sudo apt update && sudo apt upgrade -y
    ```

3. Run the command on the pi:

    ```
    sudo usermod -a -G dialout jill
    ```
    
    - NOTE: jill is the username we have on *our* pi, yours may be different.

    ```
    sudo usermod -a -G video jill
    ```

    - NOTE: jill is the username we have on *our* pi, yours may be different.

4. Reboot both of the systems.

5. Follow the setup instructions to install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) on both.

6. Do the following commands on both:
    ```
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```

7. Do a recursive clone of this repository on each machine. 
    ```
    git clone --recursive https://github.com/santoslab/890-ros-f24-project-articubot.git
    ```

## Pi Only

8. On the ***pi***, run the command:

    ```
    sudo apt install openssh-server raspi-config python3-colcon-common-extensions -y
    ```

    - Then run,

    ```
    sudo raspi-config
    ```

    - Do the following inside of the raspi-config:

      - 3 Interface Options --> Legacy Camera --> Yes
      - 3 Interface Options --> SPI --> Yes
      - 3 Interface Options --> I2C --> Yes
      - 3 Interface Options --> SSH --> Yes
    
    - Then, press tab and right arrow to click finish, exiting the program.

9. On the ***pi*** run the command:

    ```
    sudo apt install v4l-utils ros-humble-v4l-camera ros-humble-ros2-control ros-humble-ros2-controllers libserial-dev ros-humble-xacro -y
    ```

10. cd into articubot_ws in your copy of this repo on your pi, then run.

    ```
    colcon build 
    source install/setup.bash
    ```

11. On the pi, remove all usb devices expect for arduino. (ssh into your pi in order to provide it commands)

12. run the command on the pi making sure you are in the articubot_ws folder of the repo's directory:

    ```
    ls /dev/serial/by-path/
    ```

    - copy the resulting string from the ls command. 

    ```
    nano src/articubot_description/urdf/ros2_control.xacro 
    ```

    - in the line containing /dev/serial/by-path/. . . , replace . . . with your copied string:

    ![First Replacement Picture](/ReadMe_Pictures/FirstReplace.png)

    ```
    It will resemble the following:
    /dev/serial/by-path/COPIED_STRING
    ```

13. Plug in the USB hub to the pi with the lidar.

14. On the pi:

    ```
    ls /dev/serial/by-path/
    ```

    - copy the new string in the result of ls.

    ```
    nano src/articubot_description/launch/real_robot.launch.py
    ```

    - on the lidar node's parameter's serial port entry, make it resemble the following like step #10.

    ![Second Replacement Picture](/ReadMe_Pictures/SecondReplace.png)

    ```
    /dev/serial/by-path/COPIED_STRING
    ```

15. Run the following command on the within the articubot_ws folder of the repo:

    ```
    colcon build
    source install/setup.bash
    ros2 launch articubot_description real_robot.launch.py
    ```

## VM/Personal Only

16. Run the following install command:

    ```
    sudo apt install ros-humble-twist-mux rviz ros-humble-slam-toolbox joystick -y
    ```

NOTE: When running the system, there is a race condition that exists within the real_robot.launch.py file. If the race condition causes an unsuccessful launch (the launch exits when ran by itself), then try again. At the moment, the system waits 3 seconds before trying to run the second part of the race condition, but this does not always catch it. 

# Workplan

The work was carried out in a progression similar the one outlined on
the AR blog.

## Initial Design

* URDF/XACRO specification of the robot, with assumed dimensions for
  chassis, wheels, etc.  This enables the robot structure to be
  visualized and provides the basis for a Gazebo simulation (next
  step).  The dimensions where modified slightly once the robot was
  actually assembled.
* Initial Simulation in Gazebo  

These steps have some extensions later in the project.  For example,
when camera and lidar are added the URDF and Gazebo simulations are
extended to include these additional components.



## Motor Prototyping

ToDo:

* Add 1 paragraph overview description of the goals of this activity and what product (part of the robot) is the result
* Add link to Articulated robotics video that you followed
* Add list of parts to be utilized in this activity (Arduino nano, L298N controller, motor(s), power sources, laptop or PI to interact over serial)
* Add instructions for how to download and install sketch on Arduino (any notes on modifications to the sketch code)
* Add instructions for commands to use to communicate over serial to arduino
* Add description of basic tests/steps to calibrate encoder counts, etc.

### Open Loop Control

#### Arduino Nano Initial Setup

* Download (e.g., as a zip file) the contents of Josh's ROS Arduino Bridge repo https://github.com/joshnewans/ros_arduino_bridge
* Open the file `ROSArduinoBridge.ino` in the Arduino IDE, compile it, and upload to the Arduino

#### Prepare Infrastructure to Send Commands Over Serial 

If you are using the Arduino IDE, you can send/receive serial messages over the Serial Monitor tool.  Otherwise, if you want to do communicate using the terminal, install MiniTerm.   Here are the instructions for Ubuntu (e.g., if you are setting up to communicate from the Raspberry Pi)
```
sudo apt install -y python3-serial 
```

On Mac OS using Homebrew, equivalent functionality is provided by `minicom`.
```
brew install minicom
``` 

#### Complete Minimal Wiring/Power Circuit Needed for Interacting with Motors

The following items are needed:
* Arduino nano
* L298N controller
* motor
* 5V power source
* 12 power source
* Ground comes from


##### Connect Motor Controls to Driver Board

The red & white leads from the motors connect to the two sets of blue side terminals on the Driver Board
* Left Motor White -> Driver Board Out1
* Left Motor Red -> Driver Board Out2
* Right Motor White -> Driver Board Out3
* Right Motor Red -> Driver Board Out4


##### Connect Driver Board to Arduino

Using female->male connectors connect Driver Board pins to Arduino

Driver Board In1|Arduino Pin D6
Driver Board In2|Arduino Pin D10
Driver Board In3|Arduino Pin D9
Driver Board In4|Arduino Pin D5

##### Buck Converter




##### Open Loop Control

```
ls /dev | grep ttyUSB
pyserial-miniterm -e /dev/ttyUSB0 57600
```
Or using the macOS minicom, 
```
-b, --baudrate         : set baudrate (ignore the value from config)
-D, --device           : set device name (ignore the value from config)
tty.usbserial-B003PZ4Z
```



This brings up the mini-term interface.  To test the open loop control, you can do something like the following...
```
o 100 100
```
Perhaps it is a good idea to spin one wheel at a time to make sure that the wheel wiring is not reversed.

### Closed Loop Control (adding encoder functionality)

##### Connect Encoder Sensors from Each Motor to Arduino

Right Motor
* Right Motor Blue|Arduino 5V pin
* Right Motor Green|Arduino pin A4
* Right Motor Yellow|Arduino pin A5
* Right Motor Black|Arduino Ground pin

Left Motor
* Left Motor Blue|Arduino 5v pin
* Left Motor Green|Arduino pin D2
* Left Motor Yellow|Arduino pin D3
* Left Motor Black|Arduino Ground pin

##### Callibrating Encoders

To test to see if encoder sensors are working, do something like the following sequence over the serial connection...

```
m 100 100    // spin motors
e            // read encoder values (should return 2 values)
r            // reset encoder values
e            // read encoder values (should return 2 0's)  
```

Now go through the process of doing getting encoders per revolution...

* Make an indicator point on each wheel/shaft, e.g., using a piece of tape
* Move the shaft manually in the direction of rotation to a reference position (e.g., tape pointing up)
* reset the encoders with `r` and confirm that counts are 0 using `e`.
* using managed rotation (e.g., such as 'm 100 100'), repeatedly give this command until you have approximate 22 rotations have occured (the exact # doesn't matter).  Manually rotate shaft in direction of rotation until reference position is reached.   
* Read number of encoder counts using `e`.  For each wheel, Calculate the number of encoder steps per revolution by dividing total number of counter by revolution


### Motor Wiring Chart
**Start source**|**Destination source**
:-----:|:-----:
Left Motor White|Driver Board Out1
Left Motor Blue|Arduino 5v pin
Left Motor Green|Arduino pin D2
Left Motor Yellow|Arduino pin D3
Left Motor Black|Arduino Ground pin
Left Motor Red|Driver Board Out2
Right Motor White|Driver board Out3
Right Motor Blue|Arduino 5V pin
Right Motor Green|Arduino pin A4
Right Motor Yellow|Arduino pin A5
Right Motor Black|Arduino Ground pin
Right Motor Red|Driver Board Out4
Driver Board In1|Arduino Pin D6
Driver Board In2|Arduino Pin D10
Driver Board In3|Arduino Pin D9
Driver Board In4|Arduino Pin D5
Driver Board +5V|Arduino 5V pin
Driver Board +12V|Power supply +12V
Driver Board Ground|Power Supply Ground


* Phase 1 Deliverables (Target Date: Week of Sept 23) - Initial interaction with motors
  - Circuit diagrams (or photos), parts descriptions, and notes for basic non-ROS-control interfacing with motors from Raspberry PI
  - Link to source code to run on PI to do basic interface
  - Description of set up, running, and issues to address.  Identify any particularly challenging aspects that future projects should be aware of 

* Phase 2 Deliverables - Motor Interface via ROS Control (Target Date: Week of Sept 30)



## Design Phase - URDF

The original AR URDF/Xacro files at the completion of the design phase
are provided in [this
branch](https://github.com/joshnewans/articubot_one/tree/d5aa5e9bc9039073c0d8fd7fe426e170be79c087)
of the AR github repo.

For the 890 project, here are the following differences...
(TBD)


