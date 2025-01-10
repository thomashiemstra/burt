import inspect
from threading import Thread
from time import sleep

import pybullet as p
import os

import tkinter as tk
from tkinter import ttk


class Slider(object):

    def __init__(self, root, body_id, joint_index) -> None:
        self._current_value = tk.IntVar()
        self._body_id = body_id
        self._joint_index = joint_index

        joint_info = p.getJointInfo(body_id, joint_index)
        self._joint_lower_limit = joint_info[8]
        self._joint_upper_limit = joint_info[9]

        self._slider_min = -100
        self._slider_max = 100

        self.slider = ttk.Scale(
            root,
            from_=self._slider_min,
            to=self._slider_max,
            orient='horizontal',  # vertical
            command=self._slider_changed,
            variable=self._current_value,
            length=400
        )

        self.slider.grid(
            column=0,
            row=self._joint_index,
            sticky='we'
        )

    def _slider_changed(self, event):
        target_position = self._map(self._current_value.get())
        print(target_position)
        p.setJointMotorControl2(bodyUniqueId=self._body_id,
                                jointIndex=self._joint_index,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=target_position)

    def _map(self, x):
        return (x - self._slider_min) * (self._joint_upper_limit - self._joint_lower_limit) / (self._slider_max - self._slider_min) + self._joint_lower_limit


if __name__ == '__main__':
    current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

    physics_client = p.connect(p.GUI, options="--width=1920 --height=1080")

    # p.setGravity(0, 0, -9.81, physicsClientId=physics_client)
    p.setRealTimeSimulation(1, physicsClientId=physics_client)
    p.resetDebugVisualizerCamera(1, -40, -40, cameraTargetPosition=[-0.2, -0.1, 0.5])

    body_id = p.loadURDF(current_dir + "/urdf/burt.urdf", physicsClientId=physics_client)
    # body_id = p.loadURDF(current_dir + "/abb_irb4600_support/test_abb_4600.urdf", physicsClientId=physics_client)

    number_of_joints = p.getNumJoints(body_id)

    root = tk.Tk()
    root.geometry('400x500')
    # root.resizable(False, False)
    root.title('Slider Demo')

    for joint_index in range(number_of_joints):
        Slider(root, body_id, joint_index)

    while True:
        root.update()

    # root.mainloop()