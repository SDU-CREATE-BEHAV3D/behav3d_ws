from traj_utils import fibonacci_trajectory

class Session():
    def __init__(self, robot_IP, extruder_speed, camera_resolution):
        self.robot_IP = robot_IP
        self.extruder_speed = extruder_speed
        self.camera_resolution = camera_resolution

        self.motion = Motion()
        self.camera = Camera(self.camera_resolution)
        self.extruder = Extruder(self.extruder_speed)

class FibScanSession(Session):
    def __init__(self):
        super().__init__("192.168.1.1", 100, "1920x1080")
        self.traj = fibonacci_trajectory

    def _fibScan(self):
        for target in self.traj:
            self.motion.moveL(target)
            if userInput() == "capture":
                self.camera.capture()


class behaviorSession(Session):
    def __init__(self, robot_IP, extruder_speed, camera_resolution):
        super().__init__(robot_IP, extruder_speed, camera_resolution)
        self.targets = read_targets_file()
        self.wait = 1
        self.z_offset = 20

    def approach(self):
        ret = []
        for target in self.targets:
            offset_target = target.move_along_z(self.z_offset)
        return ret
            if userInput() == "capture":
                self.camera.capture()

    def read_targets(self):
        self.fibTargets = fibonacci_trajectory

####################################### SCRIPT

session = Session("192.168.1.1", 100, "1920x1080")

session.read_targets()

for target in session.fibTargets:
    session.robot.moveL(target)
    if userInput() == "capture":
        session.camera.capture()
