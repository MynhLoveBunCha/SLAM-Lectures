# For each cylinder in the scan, find its cartesian coordinates,
# in the world coordinate system.
# Find the closest pairs of cylinders from the scanner and cylinders
# from the reference, and the optimal transformation which aligns them.
# Then, output the scanned cylinders, using this transform.
# 04_c_estimate_transform
# Claus Brenner, 14 NOV 2012
from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import\
     compute_scanner_cylinders, write_cylinders
from math import sqrt
import numpy as np

# Given a list of cylinders (points) and reference_cylinders:
# For every cylinder, find the closest reference_cylinder and add
# the index pair (i, j), where i is the index of the cylinder, and
# j is the index of the reference_cylinder, to the result list.
# This is the function developed in slam_04_b_find_cylinder_pairs.
def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []
    for i in range(len(cylinders)):
        x, y = cylinders[i]
        min_dist = float("inf")
        min_ref_idx = None
        for j in range(len(reference_cylinders)):
            x_ref, y_ref = reference_cylinders[j]
            dist = ((x - x_ref)**2 + (y - y_ref)**2)**0.5
            if dist < min_dist and dist < max_radius:
                min_dist = dist
                min_ref_idx = j
        if min_ref_idx is not None:
            cylinder_pairs.append((i, min_ref_idx))
    # --->>> Insert here your code from the last question,
    # slam_04_b_find_cylinder_pairs.

    return cylinder_pairs

# Given a point list, return the center of mass.
def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (float(sx) / len(point_list), float(sy) / len(point_list))

# Given a left_list of points and a right_list of points, compute
# the parameters of a similarity transform: scale, rotation, translation.
# If fix_scale is True, use the fixed scale of 1.0.
# The returned value is a tuple of:
# (scale, cos(angle), sin(angle), x_translation, y_translation)
# i.e., the rotation angle is not given in radians, but rather in terms
# of the cosine and sine.
def estimate_transform(left_list, right_list, fix_scale = False):
    # Compute left and right center.
    lc = np.array(compute_center(left_list)).reshape((1, 2))
    rc = np.array(compute_center(right_list)).reshape((1, 2))

    left_vec = np.array(left_list)
    right_vec = np.array(right_list)

    # unbiased
    if left_vec.shape[0] <= 1:
        return None
    else:
        left_prime = left_vec - lc
        right_prime = right_vec - rc

    cs = np.sum(right_prime[:, 0] * left_prime[:, 0] + right_prime[:, 1] * left_prime[:, 1])  # cos sum
    ss = np.sum(-right_prime[:, 0] * left_prime[:, 1] + right_prime[:, 1] * left_prime[:, 0])  # sin sum
    rr = np.sum(right_prime[:, 0] * right_prime[:, 0] + right_prime[:, 1] * right_prime[:, 1])  # length of right square
    ll = np.sum(left_prime[:, 0] * left_prime[:, 0] + left_prime[:, 1] * left_prime[:, 1])  # length of left square

    la = 1.0
    if not fix_scale:
        la = sqrt(rr / ll)
    c = cs / sqrt(cs**2 + ss**2)
    s = ss / sqrt(cs**2 + ss**2)
    t = rc.T - la * (np.array([[c, -s], [s, c]]) @ lc.T)
    tx, ty = np.squeeze(t[0]), np.squeeze(t[1])
    # --->>> Insert here your code to compute lambda, c, s and tx, ty.
    return la, c, s, tx, ty

# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)


if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The constants we used for the cylinder detection in our scan.    
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # The maximum distance allowed for cylinder assignment.
    max_cylinder_distance = 300.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("Unit_B/robot4_motors.txt")
    logfile.read("Unit_B/robot4_scan.txt")

    # Also read the reference cylinders (this is our map).
    logfile.read("Unit_B/robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    out_file = open("Unit_B/estimate_transform.txt", "w")
    for i in range(len(logfile.scan_data)):
        # Compute the new pose.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Extract cylinders, also convert them to world coordinates.
        cartesian_cylinders = compute_scanner_cylinders(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset)
        world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                           for c in cartesian_cylinders]

        # For every cylinder, find the closest reference cylinder.
        cylinder_pairs = find_cylinder_pairs(
            world_cylinders, reference_cylinders, max_cylinder_distance)

        # Estimate a transformation using the cylinder pairs.
        trafo = estimate_transform(
            [world_cylinders[pair[0]] for pair in cylinder_pairs],
            [reference_cylinders[pair[1]] for pair in cylinder_pairs],
            fix_scale = True)

        # Transform the cylinders using the estimated transform.
        transformed_world_cylinders = []
        if trafo:
            transformed_world_cylinders =\
                [apply_transform(trafo, c) for c in
                 [world_cylinders[pair[0]] for pair in cylinder_pairs]]            

        # Write to file.
        # The pose.
        out_file.write(f"F {pose[0]} {pose[1]} {pose[2]}\n")
        # The detected cylinders in the scanner's coordinate system.
        write_cylinders(out_file, "D C", cartesian_cylinders)
        # The detected cylinders, transformed using the estimated trafo.
        write_cylinders(out_file, "W C", transformed_world_cylinders)

    out_file.close()
