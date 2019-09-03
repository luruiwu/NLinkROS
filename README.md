![Logo of the project](./images/nooploop.png)

# NLink ROS Package

The ROS driver package is developed based on NLink of Nooploop protocol series written in C++, and there are no other dependency libraries except ROS.

Mainly includes the following parts:
* [Serial library](https://github.com/wjwwood/serial):A cross-platform library for interfacing with rs-232 serial like ports written in C++
* General protocol parsing library:Data can be directly input into the parsing interface, protocol frame splicing and verification will be completed automatically, only need to inherit our protocol class to obtain parsing support.
* Product support (non Nooploop product users can also use them as examples)
  * LinkTrack: LinkTrack is a powerful local positioning system with PNTC (location, navigation, timing, commucation) based on UWB(Ultra-wide Bandwidth) technology.Supporting 1-D, 2-D, 3-D positioning, 1-D, 2-D positioning accuracy is 10cm, 3-D typical positioning accuracy is 30cm; When choose a 50Hz refresh frequency, it supports 40 Tags and 8 anchors;Supporting distributed ranging and data transmission, is easy to be used for robots formation without geographical restrictions; supporting pure data transmission mode, with 3Mbps. 
  * TOFSense: TOFSense is a laser ranging sensor based on TOF(Time of Flight) technology. The ranging range is 1 cm~5 m, range resolution is 1 mm, and the data update frequency is 10 Hz.Adjustable FOV, maximum view angle is 27 degrees; Support UART and CAN communication; Support constant frequency output or query it by yourself; Support multi-sensor cascade ranging.

## Getting started

Build:

    catkin_make

Run(For LinkTrack):

    ./build/nlink/linktrack_publisher /dev/ttyUSB0 921600

Run(For TOFSense):

    ./build/nlink/tofsense_publisher /dev/ttyUSB0 921600

Notice: please use portname and baudrate of your device instead.

Known issue
* Run failed,show "Unhandled Exception: IO Exception (13): Permision denied,file ... ",permision need to be changed.

Edit file

    sudo gedit /etc/udev/rules.d/70-ttyusb.rules
    
append

    KERNEL=="ttyUSB[0-9]*",MODE="0666"
    
Then replug the device,try again.

## Licensing

The 3-Clause BSD License

## Contact us
website: www.nooploop.com

email: samuelying.hsu@gmail.com
