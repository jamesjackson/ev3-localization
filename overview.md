

##Mission #4 - Lego Mindstorms Localization



###Hardware

- Lego Mindstorms EV3 Brick (300Mhz ARM9, 64MB RAM)
- 2 Large Motors
- 2 Ultrasonic Sensors
- 2 Color Sensors
- Edimax EW-7811Un 150Mbps 11n Wi-Fi USB Adapter


###Software

- [ev3dev](http://www.ev3dev.org/) environment based on Debian Linux (3.16 kernel)
- [python-ev3](https://github.com/topikachu/python-ev3) Python development environment


###Software Installation

 1. Follow [installation instructions](http://www.ev3dev.org/docs/getting-started/) for ev3dev, using image [ev3dev-jessie-2014-10-07](https://github.com/ev3dev/ev3dev/releases/tag/ev3dev-jessie-2014-10-07)
 2. Follow [installation instructions](https://github.com/topikachu/python-ev3) for Python EV3

###Accessing the Robot Python Environment

- SSH to robot IP address as root/r00tme
 
> jamess-air-2:~ fiberhog$ ssh root@192.168.1.16

> root@192.168.1.16's password: 

> root@ev3dev:~#

- Setup Python EV3 environment:
 
> root@ev3dev:~# workon ev3_py27

> (ev3_py27)root@ev3dev:~# 


###Robot Design

The robot is a fairly compact differential drive system with 2 large motors powering the front wheels, and a caster ball in the rear. The color sensors are optimally positioned ahead of the wheels, about 0.5cm off the ground. The color sensors are only used to locate the target (a small white card on the floor) after successful global localization. The ultrasonic sensors are also positioned ahead of the wheels (allowing for quick wheel response to sensor readings). One ultrasonic sensor faces left (pi radians) and is used for wall following. The other faces forward (pi/2 radians) and is used to detect walls or obstacles ahead.

#####Front View


![front view](robot_design_front.PNG)


