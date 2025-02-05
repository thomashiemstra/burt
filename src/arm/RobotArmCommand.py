class RobotArmCommand:

    def __init__(self, config):
        self.x = config.arm_default_x
        self.y = 0
        self.z = config.arm_default_z
        self.phi = 0
        self.griper_state = 0
