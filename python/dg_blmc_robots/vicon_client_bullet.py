
#### Viconclient object for center of mass control
class ViconClientEntity(object):
    def __init__(self, clientName):
        self.clientName = clientName

    def connect_to_vicon(self, host_name):
        self.host_name = host_name

    def displaySignals(self):
        print("signals are :")

    def add_object_to_track(self, name):
        self.robot_vicon_name = "quadruped"

    def robot_wrapper(self, robot_wrapper):
        self.robot = robot_wrapper

    def signal(self, signal_name):
        if signal_name == self.robot_vicon_name + "_position":
            value = self.robot.signal_base_pos_.sout
            print(value)

        elif signal_name == self.robot_vicon_name + "_velocity_body":
		    value = self.robot.signal_base_vel_.sout

        return value
