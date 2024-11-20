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
Foo bar
```

#### Complete Minimal Wiring/Power Circuit Needed for Interacting with Motors

The following items are needed:
* Arduino nano
* L298N controller
* motor
* 5V power source
* 12 power source
* Ground comes from









### Motor Wiring Chart
**Start source**|**Destination source**
:-----:|:-----:
Left Motor White|Driver Board Out1
Left Motor Blue|Arduino 5v pin
Left Motor Green|Arduino pin D2
Left Motor Yellow|Arduino pin D3
Left Motor Black|Arduino Ground pin
Left Motor Red|Driver Board Out2
Right Motor White|Driver board Out4
Right Motor Blue|Arduino 5V pin
Right Motor Green|Arduino pin A4
Right Motor Yellow|Arduino pin A5
Right Motor Black|Arduino Ground pin
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


