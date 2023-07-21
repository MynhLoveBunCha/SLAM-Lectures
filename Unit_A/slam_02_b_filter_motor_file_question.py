# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records.
#
# 02_b_filter_motor_file
# Claus Brenner, 09 NOV 2012
from math import sin, cos, pi
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):
    old_x, old_y, old_theta = old_pose
    left_inc = motor_ticks[0] * ticks_to_mm
    right_inc = motor_ticks[1] * ticks_to_mm

    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        theta = old_theta
        x = old_x + left_inc * cos(old_theta)
        y = old_y + left_inc * sin(old_theta)
        # --->>> Use your previous implementation.
        # Think about if you need to modify your old code due to the
        # scanner displacement?
        
        return (x, y, theta)

    else:
        # Turn. Compute alpha, R, etc.
        alpha = (right_inc - left_inc) / robot_width
        turning_radius = left_inc / alpha

        old_x = old_x - scanner_displacement * cos(old_theta)
        old_y = old_y - scanner_displacement * sin(old_theta)

        x_center = old_x - (turning_radius + robot_width / 2) * sin(old_theta)
        y_center = old_y + (turning_radius + robot_width / 2) * cos(old_theta)
        theta = (old_theta + alpha) % (2 * pi)
        x = x_center + (turning_radius + robot_width / 2) * sin(old_theta + alpha)
        y = y_center - (turning_radius + robot_width / 2) * cos(old_theta + alpha)

        x = x + scanner_displacement * cos(old_theta + alpha)
        y = y + scanner_displacement * sin(old_theta + alpha)
        # --->>> Modify your previous implementation.
        # First modify the the old pose to get the center (because the
        #   old pose is the LiDAR's pose, not the robot's center pose).
        # Second, execute your old code, which implements the motion model
        #   for the center of the robot.
        # Third, modify the result to get back the LiDAR pose from
        #   your computed center. This is the value you have to return.

        return (x, y, theta)

if __name__ == '__main__':
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 173.0

    # Measured start position.
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("Unit_A/robot4_motors.txt")

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,
                           scanner_displacement)
        filtered.append(pose)

    # Write all filtered positions to file.
    f = open("Unit_A/poses_from_ticks.txt", "w")
    for pose in filtered:
        f.write(f"F {pose[0]} {pose[1]} {pose[2]}\n")
    f.close()
