import tkinter as tk
from tkinter import ttk


class ConfigEditor(object):

    def __init__(self, root, config_val, upper, lower, name='unknown', row=0) -> None:
        self.root = root
        self.row = row
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
            length=300
        )

        self.slider.set(config_val)

        self.slider.grid(
            column=2,
            row=row,
            sticky='e'
        )

        ttk.Label(root, text=name + ":").grid(row=row, column=0, pady=4, padx=2)

        self.label = ttk.Label(root, text=self._label())
        self.label.grid(row=row, column=1, pady=4, padx=20)

    def _slider_changed(self, event):
        new_val = self._current_value.get()
        print(self.name + ": " + str(new_val))
        self.change_val(self._current_value.get())
        self.label.destroy()
        self.label = ttk.Label(self.root, text=self._label())
        self.label.grid(row=self.row, column=1, pady=4, padx=4)

    def _label(self):
        return str("{:.3}".format(self._current_value.get()))

    def change_val(self, val):
        pass

    def report_current_value(self):
        return str(self._current_value.get())