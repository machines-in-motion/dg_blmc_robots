import rospy
from dg_blmc_robots.srv import TeststandCalibration

def calibrate(mechanical_calibration):
    rospy.wait_for_service("dg_blmc_robots/TeststandCalibration")
    response = False
    try:
        rospy.ServiceProxy(
          "dg_blmc_robots/TeststandCalibration", TeststandCalibration)
        response = TeststandCalibration(mechanical_calibration)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return response
