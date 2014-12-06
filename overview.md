

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
>
> root@192.168.1.16's password: 
>
> root@ev3dev:~#

- Setup Python EV3 environment:
 
> root@ev3dev:~# workon ev3_py27
>
>(ev3_py27)root@ev3dev:~# 


###Robot Design

The robot is a fairly compact differential drive system with 2 large motors powering the front wheels, and a caster ball in the rear. The color sensors are optimally positioned ahead of the wheels, about 0.5cm off the ground. The color sensors are only used to locate the target after successful global localization. The ultrasonic sensors are also positioned ahead of the wheels (allowing for quick wheel response to sensor readings). One ultrasonic sensor faces left (pi radians) and is used for wall following. The other faces forward (pi/2 radians) and is used to detect walls or obstacles ahead.

####Program Overview

The program uses ultrasonic sensors to follow the left wall of a room. It uses a particle filter to perform global localization with a known map (it does not know its starting position/orientation). Once enough steps (manually configured) have been taken for successful localization, it calculates a heading and distance towards the target (white business card on the floor) and moves towards it using simple dead reckoning. Once at the estimated target location, it uses a spiral motion and 2 color sensors to locate the target.

####Program Details

######Mapping Walls and Obstacles

An occupancy grid is pre-populated with all obstacles (0 represents a free cell, 1 represents an occupied cell). The occupancy grid is used to determine whether a particle is out-of-bounds or has moved into an obstacle. Any such particle has its weight dropped to 0.

A list is pre-populated with all walls, and used to determine the closest valid wall to a particle.


####Assumptions/Limitations

- The robot is assumed to always start alongside a wall.
- The collision avoidance during wall following is limited to the basic wall following algorithm. There is no collision avoidance when looking for the target.

####Running the Program

######Running on Real Robot - Single Particle Mode (Data Gathering/Debugging)

> run_steps = 25 (set to desired number of steps)
>
> single_particle_mode = True



####Results - Real Robot Runs

####Results - Simulation Runs




