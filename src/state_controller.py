from src.quad.State import BehaviorState


class StateController:

    def __init__(self, arm_controller, config, robot_arm, quad_robot):
        self.arm_controller = arm_controller
        self.config = config
        self.robot_arm = robot_arm
        self.quad_robot = quad_robot

        self.trot_transition_mapping = {BehaviorState.REST: BehaviorState.TROT, BehaviorState.TROT: BehaviorState.REST,
                                        BehaviorState.INSTALL: BehaviorState.REST, BehaviorState.ARM: BehaviorState.REST}
        self.activate_transition_mapping = {BehaviorState.DEACTIVATED: BehaviorState.REST,
                                            BehaviorState.REST: BehaviorState.DEACTIVATED,
                                            BehaviorState.INSTALL: BehaviorState.REST,
                                            BehaviorState.ARM: BehaviorState.REST}
        self.install_transition_mapping = {BehaviorState.REST: BehaviorState.INSTALL,
                                           BehaviorState.TROT: BehaviorState.INSTALL,
                                           BehaviorState.INSTALL: BehaviorState.REST}
        self.arm_transition_mapping = {BehaviorState.REST: BehaviorState.ARM, BehaviorState.TROT: BehaviorState.ARM,
                                       BehaviorState.ARM: BehaviorState.REST, BehaviorState.INSTALL: BehaviorState.REST}

        self.trot_transition_mapping.update({BehaviorState.DEACTIVATED: BehaviorState.DEACTIVATED})
        self.activate_transition_mapping.update({BehaviorState.TROT: BehaviorState.DEACTIVATED})
        self.install_transition_mapping.update({BehaviorState.DEACTIVATED: BehaviorState.DEACTIVATED})
        self.arm_transition_mapping.update({BehaviorState.DEACTIVATED: BehaviorState.DEACTIVATED})

    def run(self, state, arm_state, command):
        # Update operating state based on command
        if command.activate_event:
            state.behavior_state = self.activate_transition_mapping[state.behavior_state]
            print(state.behavior_state.__str__())
        elif command.trot_event:
            state.behavior_state = self.trot_transition_mapping[state.behavior_state]
            print(state.behavior_state.__str__())
        elif command.install_event:
            state.behavior_state = self.install_transition_mapping[state.behavior_state]
            print(state.behavior_state.__str__())
        elif command.robot_arm_event:
            state.behavior_state = self.arm_transition_mapping[state.behavior_state]
            print(state.behavior_state.__str__())

    def handle_state_change(self, state_command, state, arm_state):
        if state_command.robot_arm_event or state_command.trot_event:
            if state.behavior_state == BehaviorState.ARM:
                arm_angles = self.arm_controller.run_position(self.config.arm_active_position)
                self.robot_arm.activate_arm_if_not_already(arm_angles)
                arm_state.reset()
            else:
                arm_angles = self.arm_controller.run_position(self.config.arm_rest_position)
                self.robot_arm.deactivate_arm_if_not_already(arm_angles)

        if state_command.activate_event:
            if state.behavior_state == BehaviorState.DEACTIVATED:
                self.quad_robot.disable_motors()
            else:
                self.quad_robot.enable_motors()
