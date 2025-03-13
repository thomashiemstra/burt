class RobotArmState:

    def __init__(self, config):
        self.x = config.arm_default_x
        self.y = 0
        self.z = config.arm_default_z
        self.phi = 0
        self.griper_state = 0

        self.initial_x = config.arm_default_x
        self.initial_y = 0
        self.initial_z = config.arm_default_z
        self.initial_phi = 0
        self.initial_gripper_state = 0

    def reset(self):
        self.x = self.initial_x
        self.y = self.initial_y
        self.z = self.initial_z
        self.phi = self.initial_phi
        self.griper_state = self.initial_gripper_state
