import sys
from dg_blmc_robots.calibration import calibrate

if __name__ == "__main__":
    if len(sys.argv) == 2:
        mechanical_calibration = bool(sys.argv[1])
    else:
        print("Please indicate if you want a mechanical calibration or not.")
    calibrate(mechanical_calibration)