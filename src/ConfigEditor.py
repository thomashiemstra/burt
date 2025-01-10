import tkinter as tk
from tkinter import ttk


class ConfigEditor(object):

    def __init__(self, root, config_val, upper, lower, name='unknown', row=0) -> None:
        self.name = name
        self._current_value = tk.DoubleVar()

        self._slider_min = config_val - lower
        self._slider_max = config_val + upper

        self.slider = ttk.Scale(
            root,
            from_=self._slider_min,
            to=self._slider_max,
            orient='horizontal',  # vertical
            command=self._slider_changed,
            variable=self._current_value,
            length=400
        )

        self.slider.set(config_val)

        self.slider.grid(
            column=1,
            row=row,
            sticky='e'
        )

        ttk.Label(root, text=name).grid(row=row, column=0, pady=4, padx=4)

    def _slider_changed(self, event):
        new_val = self._current_value.get()
        print(self.name + ": " + str(new_val))
        self.change_val(self._current_value.get())

    def change_val(self, val):
        pass

    def report_current_value(self):
        return str(self._current_value.get())

    def _map(self, x):
        return (x - self._slider_min) * (self._joint_upper_limit - self._joint_lower_limit) / (self._slider_max - self._slider_min) + self._joint_lower_limit