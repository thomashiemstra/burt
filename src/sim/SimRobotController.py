import os

import numpy as np
import pybullet as p
import pybullet_data

from src.quad.State import BehaviorState

# Leg order matches Config.LEG_ORIGINS / State.joint_angles columns:
#   0 = front-right, 1 = front-left, 2 = back-right, 3 = back-left
LEG_NAMES = ["fr", "fl", "br", "bl"]
# Axis (row) order in state.joint_angles: 0 = abduction, 1 = hip, 2 = knee
AXIS_NAMES = ["abduction", "hip", "knee"]


class SimRobotController:
    """Drop-in stand-in for QuadRobotController that drives the URDF in PyBullet
    instead of the physical STServo bus.

    It exposes the same surface the main loop / StateController rely on
    (set_actuator_positions, enable_motors, disable_motors, get_servo_list) so
    the rest of the stack does not know whether it is talking to hardware or
    the simulator.

    The IK in src/quad/Kinematics.py returns, per leg, [abduction, hip, knee]
    where every angle is measured from the (tilted) downward axis. A URDF knee
    joint, however, rotates *relative to the thigh*, so the knee command is
    knee_angle - hip_angle. abduction and hip map straight through. See the
    foot-position calibration that produced 0mm error against the IK targets.
    """

    def __init__(self, config, urdf_path, gui=True, spawn_height=0.16,
                 max_force=12.0, plane_friction=1.0, foot_friction=1.5,
                 gui_width=1920, gui_height=1080):
        self.config = config
        self.max_force = max_force
        self.spawn_height = spawn_height
        self.motors_enabled = False

        if gui:
            self.client = p.connect(p.GUI, options=f"--width={gui_width} --height={gui_height}")
        else:
            self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)

        # Floor to walk on.
        self.plane = p.loadURDF("plane.urdf", physicsClientId=self.client)
        p.changeDynamics(self.plane, -1, lateralFriction=plane_friction,
                         physicsClientId=self.client)

        # The robot. No arm is loaded - the simulated robot is the quadruped only.
        self.robot = p.loadURDF(
            os.path.abspath(urdf_path),
            [0, 0, spawn_height],
            useFixedBase=False,
            physicsClientId=self.client,
        )

        # Start the camera behind the robot (it faces +x), zoomed in and looking
        # slightly down. yaw=-90 places the eye at -x; see camera calibration.
        if gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=1,
                cameraYaw=-70,
                cameraPitch=-30,
                cameraTargetPosition=[0, -0.2, spawn_height],
                physicsClientId=self.client,
            )

        # Map (axis_index, leg_index) -> pybullet joint index, and grab foot links.
        self.joint_index = np.empty((3, 4), dtype=int)
        name_to_joint = {}
        for j in range(p.getNumJoints(self.robot, physicsClientId=self.client)):
            info = p.getJointInfo(self.robot, j, physicsClientId=self.client)
            name_to_joint[info[1].decode()] = j
            child_link = info[12].decode()
            if child_link.endswith("_foot"):
                p.changeDynamics(self.robot, j, lateralFriction=foot_friction,
                                 physicsClientId=self.client)
        for leg_index, leg in enumerate(LEG_NAMES):
            for axis_index, axis in enumerate(AXIS_NAMES):
                self.joint_index[axis_index, leg_index] = name_to_joint[f"{leg}_{axis}"]

        # Per-joint sign / offset so axis directions can be recalibrated without
        # touching the URDF (defaults verified to reproduce the IK exactly).
        self.directions = np.ones((3, 4))
        self.offsets = np.zeros((3, 4))

        # Start with the legs posed at the REST stance so the (initially limp)
        # robot looks right rather than spawning splayed out.
        self._reset_to_rest_pose()

    def _target_angles(self, joint_angles):
        """Convert IK joint angles (3,4) into URDF joint commands (3,4)."""
        targets = np.array(joint_angles, dtype=float)
        # Knee is relative to the thigh in the URDF; the IK knee angle is absolute.
        targets[2, :] = joint_angles[2, :] - joint_angles[1, :]
        return self.directions * targets + self.offsets

    def _reset_to_rest_pose(self):
        from src.quad.Kinematics import four_legs_inverse_kinematics
        rest_feet = self.config.default_stance + np.array([0, 0, self.config.default_z_ref])[:, None]
        joint_angles = four_legs_inverse_kinematics(rest_feet, self.config)
        targets = self._target_angles(joint_angles)
        for leg_index in range(4):
            for axis_index in range(3):
                p.resetJointState(
                    self.robot,
                    int(self.joint_index[axis_index, leg_index]),
                    targets[axis_index, leg_index],
                    physicsClientId=self.client,
                )

    def reset(self):
        """Teleport the robot back to its spawn pose and clear all motion.

        Used to recover after a fall/flip without restarting the sim.
        """
        p.resetBasePositionAndOrientation(
            self.robot, [0, 0, self.spawn_height], [0, 0, 0, 1],
            physicsClientId=self.client,
        )
        p.resetBaseVelocity(self.robot, [0, 0, 0], [0, 0, 0], physicsClientId=self.client)
        self._reset_to_rest_pose()

    def set_actuator_positions(self, state):
        if state.behavior_state == BehaviorState.DEACTIVATED:
            return
        targets = self._target_angles(state.joint_angles)
        force = self.max_force if self.motors_enabled else 0.0
        for leg_index in range(4):
            for axis_index in range(3):
                p.setJointMotorControl2(
                    self.robot,
                    int(self.joint_index[axis_index, leg_index]),
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=targets[axis_index, leg_index],
                    force=force,
                    physicsClientId=self.client,
                )

    def enable_motors(self):
        self.motors_enabled = True

    def disable_motors(self):
        """Let the legs go limp, mirroring torque-off on the real servos."""
        self.motors_enabled = False
        for leg_index in range(4):
            for axis_index in range(3):
                p.setJointMotorControl2(
                    self.robot,
                    int(self.joint_index[axis_index, leg_index]),
                    controlMode=p.VELOCITY_CONTROL,
                    force=0.0,
                    physicsClientId=self.client,
                )

    def step(self, n=1):
        for _ in range(n):
            p.stepSimulation(physicsClientId=self.client)

    def get_servo_list(self):
        # Present only so the (optional) servo editor in the runner has a target.
        return []

    def disconnect(self):
        if p.isConnected(self.client):
            p.disconnect(self.client)
