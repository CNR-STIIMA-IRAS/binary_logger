# Binary Logger Package by CNR-ITIA (www.itia.cnr.it)

The repository contains the implementation of the ROS Binary Logger developed by the Institute of Industrial Technologies and Automation, of the National Research Council of Italy (CNR-ITIA).


# Overview

The package provides all the functionalities to acquire ROS topics and store the data into a binary file. 

The goal is to reduce the dimensions of the stored file (if compared to a .bag file) for long data acquisition and speed up the log file analysis (e.g. MATLAB spend ~0.1s to unpack 300MB of binary file).

The message type that can be recorded (at the moment) are:

- 'JointState'

- 'Imu'

- 'PoseStamped'

- 'WrenchStamped'

- 'Float64MultiArray'


# Installation and supported ROS versions

Download and compile the package (there are no particular dependencies, only the ROS standard installation is required).

The package has been tested with ROS 'Jade' and 'Kinetic'.


# Usage

To acquire a new topic it is required to:

1) define the topics characteristics and the binary file name into the 'cfg/binary_logger_cfg.yaml' (further information are reported inside the file);

2) launch the file 'launch/binary_logger.launch' to start to record the data (NOTE: if the topic is not available, the logger will wait for the topic);


# MATLAB  

The package provides also scripts to decode the binary files in MATLAB.
More details are reported in **scripts** ([README](scripts/README.md)).


# Add new logger functionalities

To package is an opensource project and the users are encouraged to add and share new functionalities introducing new message type. 

To add new message type it is required to:

1) add a new header file for the new message type to 'include/binary_logger/' folder (use an existing file as example);

2) add the correspondent .cpp file to 'src/binary_logger/' foder (use an existing file as example. The main changes that need to be done is in the class method that handles the callback). 

NOTE: the new .cpp need to be added to the CMakeLists.txt;

3) add the new message type to the 'binary_logger_plugins.xml';


# Developers Contacts

**Authors:** 

- Manuel Beschi (manuel.beschi@itia.cnr.it)
- Enrico Villagrossi (enrico.villagrossi@itia.cnr.it)
 
Software License Agreement (BSD License) Copyright (c) 2016, National Research Council of Italy, Institute of Industrial Technologies and Automation. All rights reserved.