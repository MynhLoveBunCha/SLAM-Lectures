# Compute the derivative of a scan.
# 03_b_scan_derivative
# Claus Brenner, 09 NOV 2012
from pylab import *
from lego_robot import *

# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):
    jumps = [ 0 ]
    for i in range(1, len(scan) - 1):
        left_lim = scan[i - 1]
        right_lim = scan[i + 1]
        derivative = 0
        if left_lim > min_dist and right_lim > min_dist:
            derivative = (right_lim - left_lim) / 2.0
        # --->>> Insert your code here.
        # Compute derivative using formula "(f(i+1) - f(i-1)) / 2".
        # Do not use erroneous scan values, which are below min_dist.
        jumps.append(derivative) # Replace this line, append derivative instead.

    jumps.append(0)
    return jumps


if __name__ == '__main__':

    minimum_valid_distance = 20.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("Unit_A/robot4_scan.txt")

    # Pick one scan.
    scan_no = 8
    scan = logfile.scan_data[scan_no]

    # Compute derivative, (-1, 0, 1) mask.
    der = compute_derivative(scan, minimum_valid_distance)

    # Plot scan and derivative.
    title("Plot of scan %d" % scan_no)
    plot(scan)
    plot(der)
    show()
