youBot API
------------

[API documentation](http://youbot.github.com/youbot_driver)

The basic idea of the youBot API is to represent a robot system as a combination of decoupled functional sub-systems. That is, this API represents manipulator arm and base platform as the combination of several joints. At the same time each joint is defined as a combination of a motor and a gearbox.

There are three main classes in the youBot API that a user should be aware of.

* YouBotManipulator class that represents youBot arm as the aggregation of several joints and a gripper
* YouBotBase class that represents youBot base platform
* YouBotJoint class that represents joints which make up both the manipulator and the base


Installation
------------

### System requirements: 
* Linux 
* Ethernet adapter 
* Root access to the Ethernet adapter 

These libraries are required by the youBot API: 

* [Simple Open EtherCAT master](http://soem.berlios.de)
* [Boost C++ Libraries](http://www.boost.org)

You can fetch, compile and install these library by hand or you can use robotpkg a software packaging tool to do this automatically. 

### Installation with robotpkg:
If you want to use robotpkg please visit this [site](https://github.com/youbot/youbot_packages/wiki) for more details.


### Installation with rosmake:
Install a minimal installation of ROS. (see ros.org)

Clone the youBot API sources:
    
    git clone git://github.com/youbot/youbot_driver.git

Clone additional ros packages which include the SOEM (Simple Open EtherCAT master):

    git clone git://github.com/janpaulus/brics-external-packages-ros.git

Add both repository folders to the ROS_PACKAGE_PATH environment variable.

Compile the youbot driver by typing:

    rosmake youbot_driver --rosdep-install


### Manual installation:
First, make sure that you have the gnu-make software available on your system (version 3.81 or later required), as well as a working C compiler chain.
The console commands below are exemplary for a Ubuntu Linux.
You will need a git version control software:

    sudo apt-get install git git-core

Cmake a cross platform make is also necessary:

    sudo apt-get install cmake

To download the youBot API sources type:

    git clone git://github.com/youbot/youbot_driver.git

Download the Simple Open EtherCAT master (SOEM) software from the [website](http://soem.berlios.de)

Before you compile the SOEM software you have to apply two patches, which can be downloaded [here](https://github.com/youbot/youbot_driver/wiki/SOEMpatches.tar.gz).

After you have compiled and installed the SOEM you also have to install boost.

    sudo apt-get install libboost-all-dev

Now you should be able to compile the youBot API.

    cd <your_folder>/youbot_driver
    mkdir build
    cd build
    cmake ..
    make

The library will be generated in the folder ~/youbot_driver/lib.


If you do not want to do these steps manually, please use robotpkg.


Usage
------------

The youBot API give you complete joint level access to the youBot joints. Every youBot joint is represented as a youbot::YouBotJoint class in the API.
At this stage we make no difference if it is a base joint which powers a wheel or a manipulator joint.

By the classes youbot::YouBotBase and youbot::YouBotManipulator it is possible to get access to a youbot::YouBotJoint instance for a particular joint.

To set a setpoint or read some sensor values form the joints you have to use the youbot::JointData classes.
Which could be for instance youbot::JointVelocitySetpoint or youbot::JointSensedCurrent.

To configure parameters of a joint, you have to use the youbot::JointParameter classes.
Which could be for instance youbot::MaximumPositioningVelocity.


Example Programs
------------
You can find example programs in the [youbot_applications](https://github.com/youbot/youbot_applications) repository.


Documentation
------------
For more detailed information on class and methods please refer to the [API documentation](http://youbot.github.com/youbot_driver).

To generate the documentation from the source code type: 

    sudo apt-get install doxygen
    make doc 


Run without sudo
------------

The youBot Driver needs access to the raw ethernet device. Under Linux a normal user does not have access to the raw ethernet device. You can grand this capability to a program by the tool setcap. To install setcap use:

    sudo apt-get install libcap2-bin

To provide a program with raw access to a ethernet device use: (replace the ./YouBot_KeyboardRemoteControl with your program.)

    sudo setcap cap_net_raw+ep ./YouBot_KeyboardRemoteControl

This have to be done whenever the executable is created or replaces e.g. after building.


License
------------

This software is published under a dual-license: GNU Lesser General Public
License LGPL 2.1 and BSD license. The dual-license implies that users of this
code may choose which terms they prefer.

