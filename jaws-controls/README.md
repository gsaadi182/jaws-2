# Jaws Controls

## TODO
* Parameters
    * Yaw power reduction coefficients.
* Servo.writeMicroseconds()
* Yaw power reduction multiply not add.

## Connecting a PS3 Controller

https://help.ubuntu.com/community/Sixaxis

Connect using a USB cable, then type:

    sudo sixpair
    
Disconnect the USB cable, then type:

    sixad -s
    
Press the PS button, wait for rumbling.

## Troubleshooting

### No Bluetooth?

    sudo apt-get install bluez --reinstall
    
## Reference Links

### Linux
* [Sixaxis](https://help.ubuntu.com/community/Sixaxis) - Follow "Quick Setup Guide".
* [jstest](http://manpages.ubuntu.com/manpages/trusty/man1/jstest.1.html) - Joystick test, get it?

### ROS
* [joystick_drivers](http://wiki.ros.org/joystick_drivers?distro=indigo) - Meta package for joysticks.
* [joy](http://wiki.ros.org/joy?distro=indigo) - The package we've been learning with.
* [ps3joy](http://wiki.ros.org/ps3joy?distro=indigo) - Button/axis numbers. Do they match joy?
* [sensor_msgs](http://wiki.ros.org/sensor_msgs) - Joy message documentation.
