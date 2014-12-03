import time
from math import *
import random


saved_measurements = []
saved_steps=[]
saved_real_starts=[]
saved_real_ends=[]

# Sonar measurement data from real robot runs
#
# Data Set #1:
saved_steps.append(35)
saved_measurements.append([[26.7, 113.1], [26.7, 95.1], [25.7, 77.6], [25.1, 59.8], [24.1, 41.0], [23.3, 23.5], [20.9, 74.2], [22.1, 200.0], [22.7, 200.0], [23.3, 200.0], [22.0, 172.4], [200.0, 152.0], [48.1, 200.0], [22.8, 182.5], [20.4, 178.3], [19.8, 146.5], [18.4, 127.4], [17.3, 109.0], [16.6, 92.1], [14.9, 74.8], [14.3, 58.2], [13.9, 42.7], [14.9, 33.0], [32.0, 116.6], [32.1, 98.6], [32.6, 80.4], [32.0, 62.6], [29.8, 46.0], [28.1, 27.8], [24.1, 200.0], [23.9, 200.0], [28.8, 200.0], [77.2, 200.0], [37.2, 200.0], [47.3, 96.4]])
saved_real_starts.append([35.6, 7.6, pi/2])
saved_real_ends.append([287.0, 235.0, 0.314])
#
# Data Set #2:
saved_steps.append(35)
saved_measurements.append([[27.3, 113.5], [27.8, 95.6], [28.1, 77.8], [29.0, 60.3], [28.1, 42.0], [27.7, 23.6], [21.3, 71.0], [22.5, 200.0], [23.5, 200.0], [24.1, 183.5], [25.9, 165.7], [200.0, 147.8], [38.3, 200.0], [28.2, 184.3], [27.3, 167.1], [24.7, 149.9], [22.9, 130.0], [21.1, 112.3], [19.0, 94.1], [17.5, 76.5], [15.7, 59.8], [14.5, 43.9], [13.7, 34.8], [33.0, 117.1], [33.4, 99.3], [32.6, 81.4], [29.6, 64.5], [26.3, 46.2], [25.1, 29.0], [25.1, 200.0], [23.7, 200.0], [26.7, 200.0], [35.7, 200.0], [80.0, 200.0], [67.2, 103.8]])
saved_real_starts.append([35.6, 7.6, pi/2])
saved_real_ends.append([277.0, 232.0, 0.314])
#
# Data Set #3:
saved_steps.append(25)
saved_measurements.append([[25.5, 75.0], [25.9, 57.4], [25.7, 39.1], [24.9, 20.9], [18.4, 200.0], [19.5, 200.0], [32.6, 200.0], [117.5, 200.0], [40.1, 200.0], [28.9, 80.8], [26.5, 64.3], [25.1, 45.5], [22.5, 27.3], [24.3, 76.6], [23.7, 59.0], [22.5, 40.3], [21.2, 21.9], [19.7, 200.0], [20.4, 197.8], [25.1, 179.2], [36.4, 200.0], [189.5, 142.6], [46.0, 200.0], [32.1, 144.3], [33.2, 126.3]])
saved_real_starts.append([171.4, 313.0, 0.0])
saved_real_ends.append([248.9, 112.1, 5.0])


#################
# USER SETTINGS #
#################

run_steps = 25 # only applies to real robot
single_particle_mode = False # debugging mode with a single particle, uses known starting point
real_robot_mode = False # run on real robot rather than simulating with logged robot data
active_set = 3 # active data set (see above) for simulation mode
motion_noise_on = True
known_starting_orientation = True # simulate compass sensor
pf_number_particles = 1000 # number of particles in particle filter
sensor_noise_left = 10.0 # left sensor noise
sensor_noise_front = 15.0 # front sensor noise
base_steering_noise = 0.03
base_distance_noise = 1.0
base_turning_noise = 0.05
robust_likelihood_constant = 0.0 # avoids being overly aggressive on particle killing

# Use Case #1: Gather sample data from a real robot:
#
# single_particle_mode = True
# real_robot_mode = True
# motion_noise_on = False

# Use Case #2: Simulate single particle using previous measurement data:
#
# single_particle_mode = True
# real_robot_mode = False
# motion_noise_on = False

# Use Case #3: Simulate full particle filter using previous measurement data (known initial position):
#
# single_particle_mode = False
# real_robot_mode = False
# motion_noise_on = True

# Use Case #4: Simulate full particle filter using previous measurement data (unknown initial position):
#
# single_particle_mode = False
# real_robot_mode = False
# motion_noise_on = True

# Use Case #5: Run full particle filter on real robot (known initial position):
#
# single_particle_mode = False
# real_robot_mode = True
# motion_noise_on = True

# Use Case #6: Run full particle filter on real robot (unknown initial position):
#
# single_particle_mode = False
# real_robot_mode = True
# motion_noise_on = True



Kp = 0.03 # wall follower PD controller Kp constant
Kd = 0.02 # wall follower PD controller Kd constant
base_power = 30.0 # base wheel power / 10 (degrees/s)
power_multiplier = 0.20 # constrains max/min wheel power levels
target_wall_dist = 25.4 # (cm)
wheel_diameter = 6.6 # (cm)
wheel_circumference = pi * wheel_diameter # (cm)
dist_between_wheels = 11.4 # (cm)

sonar_max_distance = 200.0 # readings beyond this distance are unreliable (cm)
sonar_max_angle = 25.0 * pi / 180.0 # sonar cone: +/- 25 (deg)

dist_front_sensor_to_center = 10.0 # offset from robot center (midpoint between wheels) to front sensor (cm)
dist_left_sensor_to_center = 10.0 # offset from robot center (midpoint between wheels) to left sensor (cm)
left_sensor_orientation_offset = pi / 2.0 # left sensor offset from front (rad)

world_size_x = 361 # (cm)
world_size_y = 349 # (cm)

if motion_noise_on:
    steering_noise = base_steering_noise
    distance_noise = base_distance_noise
    turning_noise = base_turning_noise
else:
    steering_noise = 0.0 # applies to any near straight motion or slight turn motion
    distance_noise = 0.0 # only applies to true straight motion (ex. after hard left turns at 45 deg)
    turning_noise = 0.0 # applies to turning in place (hard left or hard right turns)



if real_robot_mode:
    from ev3.lego import Motor
    from ev3.lego import UltrasonicSensor
    a = Motor(port=Motor.PORT.A)
    d = Motor(port=Motor.PORT.D)
    a.reset()
    d.reset()
    a.position_mode=Motor.POSITION_MODE.RELATIVE
    d.position_mode=Motor.POSITION_MODE.RELATIVE
    sonar_left = UltrasonicSensor(port=2)
    sonar_front = UltrasonicSensor(port=3)


def power_limit(pwr):

    pwr_min = base_power - base_power * power_multiplier
    pwr_max = base_power + base_power * power_multiplier
    if pwr < pwr_min:
        return pwr_min
    if pwr > pwr_max:
        return pwr_max
    return pwr

# Occupancy grid - checks for available positions
class Grid:

    def __init__(self, rows, cols):
        self.obstacles = []
        self.grid = []
        for row in xrange(rows): self.grid += [[0]*cols]

    def add_obstacle(self, row_start, row_end, col_start, col_end):
        for row in xrange(row_start, row_end + 1):
            for col in xrange(col_start, col_end + 1):
                self.grid[row][col] = 1

    def is_available(self, x, y):

        if (0 <= x < world_size_x) and (0 <= y < world_size_y):
            if self.grid[y][x] == 0:
                return True

        return False

    def print_grid(self):
        rows = len(self.grid)
        cols = len(self.grid[0])
        print "[ ",
        for row in reversed(xrange(rows)):
            if (row >= 0): print "\n  ",
            print "[ ",
            for col in xrange(cols):
                if (col > 0): print ",",
                print str(self.grid[row][col]),
            print "]",
        print "]"


# Robot simulator
class robot_sim:

    def __init__(self):
        if single_particle_mode:
            self.x = saved_real_starts[active_set - 1][0] # initial known x position for single robot simulator testing
            self.y = saved_real_starts[active_set - 1][1] # initial known y position for single robot simulator testing
            self.orientation = saved_real_starts[active_set - 1][2] # initial known orientation for single robot simulator testing
        else:
            self.x = random.random() * (world_size_x - 1) # initial random x position for particle filter
            self.y = random.random() * (world_size_y - 1) # initial random y position for particle filter
            if known_starting_orientation:
                self.orientation = saved_real_starts[active_set - 1][2]
            else:
                self.orientation = random.choice([0.0, pi / 2, pi, 3 * pi / 2]) # initial random choice of orientation for particle filter (assume robot always starts facing N, S, W, or E)

        self.sensor_noise_left = 0.0
        self.sensor_noise_front = 0.0
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.turning_noise = 0.0

    def set(self, new_x, new_y, new_orientation):

        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_sensor_noise_left, new_sensor_noise_front, new_steering_noise, new_distance_noise, new_turning_noise):

        self.sensor_noise_left = float(new_sensor_noise_left)
        self.sensor_noise_front = float(new_sensor_noise_front)
        self.steering_noise = float(new_steering_noise)
        self.distance_noise = float(new_distance_noise)
        self.turning_noise = float(new_turning_noise)

    def find_closest_wall(self, walls, sensor):

        orientation_offset = 0.0
        sensor_dist_offset = 0.0

        if sensor == 'left':
            orientation_offset = left_sensor_orientation_offset
            sensor_dist_offset = dist_left_sensor_to_center

        elif sensor == 'front':
            sensor_dist_offset = dist_front_sensor_to_center

        close_walls = []

        for wall in walls:

            on_line_segment = False

            x1 = wall[0]
            y1 = wall[1]
            x2 = wall[2]
            y2 = wall[3]

            # angle between sensor and wall
            angle_to_wall = acos((cos(self.orientation + orientation_offset) * (y1 - y2) + sin(self.orientation + orientation_offset) * (x2 - x1)) / sqrt((y1 - y2) ** 2 + (x2 - x1) ** 2 ))

            # check that we don't exceed the sonar cone
            if abs(angle_to_wall) > sonar_max_angle:
                continue

            # accommodate differences between sensor mount positions and robot center (mid-point between wheels)
            sensor_x = self.x + sensor_dist_offset * cos(self.orientation + orientation_offset)
            sensor_y = self.y + sensor_dist_offset * sin(self.orientation + orientation_offset)

            # forward vector from sensor to wall
            dist_to_wall = ((y2 - y1) * (x1 - sensor_x) - (x2 - x1) * (y1 - sensor_y)) / ((y2 - y1) * cos(self.orientation + orientation_offset) - (x2 - x1) * sin(self.orientation + orientation_offset))

            # must be *forward* vector
            if dist_to_wall < 0:
                continue

            # if distance is beyond sonar range, ignore it
            if dist_to_wall > sonar_max_distance:
                continue

            # intercept point on wall based on following forward vector from sensor
            x_intercept_point = sensor_x + dist_to_wall * cos(self.orientation + orientation_offset)
            y_intercept_point = sensor_y + dist_to_wall * sin(self.orientation + orientation_offset)

            # check that intercept point is within the endpoints of the wall
            if (x1 - x2) == 0:
                if ((y1 <= y_intercept_point <= y2) or (y2 <= y_intercept_point <= y1)):
                    on_line_segment = True

            elif (y1 - y2) == 0:
                if ((x1 <= x_intercept_point <= x2) or (x2 <= x_intercept_point <= x1)):
                    on_line_segment = True

            elif ((x1 <= x_intercept_point <= x2) or (x2 <= x_intercept_point <= x1)) and ((y1 <= y_intercept_point <= y2) or (y2 <= y_intercept_point <= y1)):
                on_line_segment = True

            if not on_line_segment:
                continue

            # everything looks good, add wall as a candidate
            close_walls.append(dist_to_wall)

            if single_particle_mode:
                print 'Sim - Found valid wall: ', wall
                print 'Sim - Angle to wall: ', angle_to_wall
                print 'Sim - Distance to wall: ', dist_to_wall
                print 'Sim - Wall intercept point: ', x_intercept_point, y_intercept_point

        if not close_walls:
            if single_particle_mode:
                print 'Sim - Sensor dist, ', sensor, ':', sonar_max_distance
            return sonar_max_distance
        else:
            # choose the closest viable wall
            if single_particle_mode:
                print 'Sim - Sensor dist, ', sensor, ':', min(close_walls)
            return min(close_walls)


    def move_time(self, pwr_l, pwr_r, duration):

        result = robot_sim()
        result.sensor_noise_left = self.sensor_noise_left
        result.sensor_noise_front = self.sensor_noise_front
        result.steering_noise = self.steering_noise
        result.distance_noise = self.distance_noise
        result.turning_noise = self.turning_noise

        velocity_left = pwr_l * 10 * wheel_circumference / 360.0 # (cm/s)
        velocity_right = pwr_r * 10 * wheel_circumference / 360.0 # (cm/s)

        # beta - radius of arc
        # R - radius of arc
        # dist - distance driven (for true straight motion)

        x = 0.0
        y = 0.0
        orientation = 0.0

        if velocity_left == velocity_right: # going straight
            # print 'Sim - going straight...'
            orientation = (self.orientation + random.gauss(0.0, self.steering_noise)) % (2 * pi) # add steering noise to orient
            dist = velocity_right * duration / 1000.0
            x = self.x + dist * cos(orientation) + random.gauss(0.0, self.distance_noise) # add distance noise to x
            y = self.y + dist * sin(orientation) + random.gauss(0.0, self.distance_noise) # add distance noise to y

        else:
            # print 'Sim - slight turn...'
            R = dist_between_wheels * (velocity_left + velocity_right) / (2 * (velocity_right - velocity_left))
            beta = (velocity_right - velocity_left) * (duration / 1000.0) / dist_between_wheels
            orientation = (self.orientation + beta + random.gauss(0.0, self.steering_noise)) % (2 * pi) # add steering noise to orient
            cx = self.x - sin(self.orientation) * R
            cy = self.y + cos(self.orientation) * R
            x = cx + sin(self.orientation + beta) * R + random.gauss(0.0, self.distance_noise) # add distance noise to x
            y = cy - cos(self.orientation + beta) * R + random.gauss(0.0, self.distance_noise) # add distance noise to y

        result.set(x, y, orientation)

        if single_particle_mode:
            print 'Sim - straight or slight turn...'
            print 'Sim - new x: ', x
            print 'Sim - new y: ', y
            print 'Sim - new orientation (rad): ', orientation
            print 'Sim - new orientation (deg): ', orientation * 180 / pi

        return result

    def turn_in_place(self, turn_angle, pwr=150.0):

        x = 0.0
        y = 0.0
        orientation = 0.0
        result = robot_sim()
        result.sensor_noise_left = self.sensor_noise_left
        result.sensor_noise_front = self.sensor_noise_front
        result.steering_noise = self.steering_noise
        result.distance_noise = self.distance_noise
        result.turning_noise = self.turning_noise

        orientation = (self.orientation + (turn_angle * pi / 180) + random.gauss(0.0, self.turning_noise)) % (2 * pi)
        x = self.x
        y = self.y
        result.set(x, y, orientation)

        if single_particle_mode:
            print 'Sim - turn in place...'
            print 'Sim - new x: ', x
            print 'Sim - new y: ', y
            print 'Sim - new orientation (rad): ', orientation
            print 'Sim - new orientation (deg): ', orientation * 180 / pi

        return result


    def measurement_prob(self, measurements):

        predicted_measurements = []
        predicted_measurements.append(self.find_closest_wall(walls, 'left'))
        predicted_measurements.append(self.find_closest_wall(walls, 'front'))

        # compute left sensor gaussian error - use sensor_noise_left

        error_sense_dist_left = abs(measurements[0] - predicted_measurements[0])

        if single_particle_mode:
            print 'Sim - Sensor difference, left: ', error_sense_dist_left

        error_left = (exp(- (error_sense_dist_left ** 2) / (self.sensor_noise_left ** 2) / 2.0) /
                      sqrt(2.0 * pi * (self.sensor_noise_left ** 2)))

        # compute front sensor gaussian error - use sensor_noise_front

        error_sense_dist_front = abs(measurements[1] - predicted_measurements[1])

        if single_particle_mode:
            print 'Sim - Sensor difference, front: ', error_sense_dist_front

        error_front = (exp(- (error_sense_dist_front ** 2) / (self.sensor_noise_front ** 2) / 2.0) /
                      sqrt(2.0 * pi * (self.sensor_noise_front ** 2)))

        error = error_left * error_front + robust_likelihood_constant

        if single_particle_mode:
            print 'Sim - gaussian error left: ', error_left
            print 'Sim - gaussian error front: ', error_front
            print 'Sim - gaussian error total: ', error
        return error

    def __repr__(self):
        # return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))
        return '%.6s %.6s' % (str(self.x), str(self.y))


if real_robot_mode:
    class robot_real:

        def sense_front(self):

            reading1 = sonar_front.dist_cm
            reading2 = sonar_front.dist_cm
            reading3 = sonar_front.dist_cm
            result = min(reading1, reading2, reading3) / 10.0
            if result < sonar_max_distance:
                return result
            else:
                return sonar_max_distance

        def sense_left(self):

            reading1 = sonar_left.dist_cm
            reading2 = sonar_left.dist_cm
            reading3 = sonar_left.dist_cm
            result = min(reading1, reading2, reading3) / 10.0
            if result < sonar_max_distance:
                return result
            else:
                return sonar_max_distance

        def move_time(self, pwr_l, pwr_r, duration):

            a.run_time_limited(time_sp=duration, speed_sp=pwr_l * 10, regulation_mode=True, stop_mode=Motor.STOP_MODE.BRAKE)
            d.run_time_limited(time_sp=duration, speed_sp=pwr_r * 10, regulation_mode=True, stop_mode=Motor.STOP_MODE.BRAKE)
            time.sleep(duration / 1000.0)

        def turn_in_place(self, turn_angle, pwr=150.0):

            a.reset()
            d.reset()
            wheel_distance = (turn_angle / 360.0) * pi * dist_between_wheels
            wheel_angle = (wheel_distance / wheel_circumference) * 360.0
            a.run_position_limited(position_sp=-wheel_angle, speed_sp=pwr, stop_mode=Motor.STOP_MODE.BRAKE)
            d.run_position_limited(position_sp=wheel_angle, speed_sp=pwr, stop_mode=Motor.STOP_MODE.BRAKE)
            time.sleep(abs(wheel_angle/pwr))

        def stop(self):

            a.stop()
            d.stop()


class particle_filter:

    def __init__(self):

        self.p = []
        self.w = []

        if single_particle_mode:
            self.count = 1
        else:
            self.count = pf_number_particles

        for i in range(self.count):
            r = robot_sim()
            while not (mygrid.is_available(int(r.x), int(r.y))): # re-create initial particle if it lands on an unavailable spot
                r = robot_sim()
            r.set_noise(sensor_noise_left, sensor_noise_front, steering_noise, distance_noise, turning_noise)
            self.p.append(r)

    def measurement_update(self, measurements):

        for i in range(self.count):
            self.w.append(self.p[i].measurement_prob(measurements))

    def motion_update(self, motion):

        p2 = []
        for i in range(self.count):
            motion_command = motion[0]
            if motion_command == 'turn_in_place':
                turn_angle = motion[1]
                p2.append(self.p[i].turn_in_place(turn_angle))

            if motion_command == 'move_time':
                power_left = motion[1]
                power_right = motion[2]
                duration = motion[3]
                p2.append(self.p[i].move_time(power_left, power_right, duration))

        self.p = p2

    def resample(self):

        print 'PF - max w: ', max(self.w)
        print 'PF - mean w: ', float(sum(self.w)/len(self.w))

        p3 = []
        index = int(random.random() * self.count)
        beta = 0.0
        mw = max(self.w)
        for i in range(self.count):
            beta += random.random() * 2.0 * mw
            while beta > self.w[index]:
                beta -= self.w[index]
                index = (index + 1) % self.count
            p3.append(self.p[index])
        self.p = p3

    def get_position(self):
        x = 0.0
        y = 0.0
        orientation = 0.0
        for i in range(len(self.p)):
            x += self.p[i].x
            y += self.p[i].y
            orientation += (((self.p[i].orientation - self.p[0].orientation + pi) % (2.0 * pi))
                            + self.p[0].orientation - pi)
        return [x / len(self.p), y / len(self.p), orientation / len(self.p)]


if real_robot_mode:
    ev3 = robot_real() # real robot

mygrid = Grid(world_size_y, world_size_x)
mygrid.add_obstacle(130, 348, 0, 106) # bed
mygrid.add_obstacle(279, 348, 255, 360) # dresser
mygrid.add_obstacle(0, 164, 283, 360) # table

walls = []
# format of wall is [x1, y1, x2, y2]
walls.append([0, 0, 0, 130]) # wall 1
walls.append([0, 130, 106, 130]) # wall 2
walls.append([106, 130, 106, 348]) # wall 3
walls.append([106, 348, 255, 348]) # wall 4
walls.append([255, 348, 255, 279]) # wall 5
walls.append([255, 279, 360, 279]) # wall 6
walls.append([360, 279, 360, 164]) # wall 7
walls.append([360, 164, 283, 164]) # wall 8
walls.append([283, 164, 283, 0]) # wall 9
walls.append([283, 0, 0, 0]) # wall 10


if real_robot_mode:
    measurement_history = [] # store all measurements
else:
    measurement_history = saved_measurements[active_set - 1]
    run_steps = saved_steps[active_set - 1]


# Initialize particle filter
pf = particle_filter()

lastError = 0
derivative = 0

estimated_position = []

for i in range(run_steps):

    # Sense

    if real_robot_mode:
        sonar_l = ev3.sense_left()
        sonar_f = ev3.sense_front()
        measurements = [sonar_l, sonar_f]
        measurement_history.append(measurements)
    else:
        # Read from measurement history
        measurements = measurement_history[i]
        sonar_l = measurements[0]
        sonar_f = measurements[1]

    print ''
    print 'Robot - Sensor dist, left: ', sonar_l
    print 'Robot - Sensor dist, front: ', sonar_f

    print 'PF - measurement update...'
    pf.measurement_update(measurements)

    # Move

    if sonar_f < (target_wall_dist * 1.5):
        if real_robot_mode:
            ev3.turn_in_place(-90.0)
        motion = ['turn_in_place', -90.0]
        print 'PF - update motion...'
        pf.motion_update(motion)

    elif sonar_l > (target_wall_dist * 1.5):
        if real_robot_mode:
            ev3.turn_in_place(45.0)
        motion = ['turn_in_place', 45.0]
        print 'PF - update motion...'
        pf.motion_update(motion)
        if real_robot_mode:
            ev3.move_time(base_power, base_power, 1500)
        motion = ['move_time', base_power, base_power, 1500]
        print 'PF - update motion...'
        pf.motion_update(motion)

    else:
        error = sonar_l - target_wall_dist
        # print 'Robot - PD error: ', error
        derivative = error - lastError
        delta = Kp * error + Kd * derivative
        # print 'Robot - PD delta total: ', delta
        # print 'Robot - PD delta P component: ', Kp * error
        # print 'Robot - PD delta D component: ', Kd * derivative
        power_left = base_power - delta
        power_right = base_power + delta

        power_left = power_limit(power_left)
        power_right = power_limit(power_right)

        # print 'Robot - new left power: ', power_left
        # print 'Robot - new right power: ', power_right

        if real_robot_mode:
            ev3.move_time(power_left, power_right, 1000)

        motion = ['move_time', power_left, power_right, 1000]
        print 'PF - update motion...'
        pf.motion_update(motion)

        lastError = error


    print 'PF - resampling...'
    pf.resample()

    print 'PF - estimated position: '
    estimated_position = pf.get_position()
    print estimated_position

    print ''
    print 'PF: current particle set: '
    print pf.p



# print measurement history on real run
if real_robot_mode:
    print ''
    print 'Measurement history: '
    print measurement_history

# compare estimated final position with real final position (for simulated runs)
if not real_robot_mode:
    print ''
    print 'Final estimated position: ', estimated_position
    print 'Final real position: ', saved_real_ends[active_set - 1]
    deltas = [abs(estimated_position[0] - saved_real_ends[active_set - 1][0]), abs(estimated_position[1] - saved_real_ends[active_set - 1][1]), (abs(estimated_position[2] - saved_real_ends[active_set - 1][2]) + pi) % (2 * pi) - pi]
    print 'Final position deltas: ', deltas

# print final particle set (used for particle visualization)
print ''
print 'PF: final particle set: '
print pf.p

