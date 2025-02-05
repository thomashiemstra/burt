import tkinter as tk
from tkinter import ttk


class ConfigEditor(object):

    def __init__(self, root, config_val, lower, upper, name='unknown', row=0) -> None:
        self.root = root
        self.row = row
        self.name = name
        self._current_value = tk.DoubleVar()

        self._slider_min = config_val - lower
        self._slider_max = config_val + upper

        ttk.Label(root, text=name + ":").grid(row=row, column=0, pady=4, padx=4,sticky='we')

        self.label_text = tk.StringVar()
        self.label_text.set(self._label_text())
        self.label = ttk.Label(root, textvariable=self.label_text)
        self.label.grid(row=row, column=2, padx=20,sticky='we')

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
            column=1,
            row=row,
            sticky='e'
        )

    def _slider_changed(self, event):
        self.change_val(self._current_value.get())
        self.label_text.set(self._label_text())

    def _label_text(self):
        return str("{:.3}".format(self._current_value.get()))

    def change_val(self, val):
        pass

    def report_current_value(self):
        return str(self._current_value.get())


def setup_config_editor(config):
    root = tk.Tk()
    root.geometry('600x500')
    root.title('Config editor')

    row_index = 0

    def change(val):
        config.max_x_velocity = val
    editor = ConfigEditor(root, config.max_x_velocity, config.max_x_velocity * 0.9, config.max_x_velocity * 2, 'max_x_velocity', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.max_y_velocity = val
    editor = ConfigEditor(root, config.max_y_velocity, config.max_y_velocity * 0.9, config.max_y_velocity * 2, 'max_y_velocity', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.max_yaw_rate = val
    editor = ConfigEditor(root, config.max_yaw_rate, config.max_yaw_rate * 0.9, config.max_yaw_rate * 2, 'max_yaw_rate', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.swing_time = val
    editor = ConfigEditor(root, config.swing_time, config.swing_time * 0.9, config.swing_time * 2, 'swing time', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.overlap_time = val
    editor = ConfigEditor(root, config.overlap_time, config.overlap_time * 0.9, config.overlap_time * 2, 'overlap time', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.delay_factor = val
    editor = ConfigEditor(root, config.delay_factor, config.delay_factor * 0.9, config.delay_factor * 2, 'delay factor', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.alpha = val
    editor = ConfigEditor(root, config.alpha, config.alpha * 0.9, config.alpha * 2, 'alpha', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.beta = val
    editor = ConfigEditor(root, config.beta, config.beta * 0.9, config.beta * 2, 'beta', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.z_clearance = val
    editor = ConfigEditor(root, config.z_clearance, config.z_clearance * 0.9, config.z_clearance * 2, 'z_clearance', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.delta_x = val
    editor = ConfigEditor(root, config.delta_x, config.delta_x * 0.9, config.delta_x * 2, 'delta_x', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.x_shift = val
    editor = ConfigEditor(root, config.x_shift, config.x_shift * 0.9, config.x_shift * 2, 'x_shift', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.delta_y = val
    editor = ConfigEditor(root, config.delta_y, config.delta_y, 0.06, 'delta_y', row=row_index)
    editor.change_val = change
    row_index += 1

    def button_clicked():
        print(config)
    tk.Button(root,
              text="dump config",
              anchor="center",
              command=button_clicked
              ).grid(row=row_index, column=1)

    return root


class ServoEditor(object):

    def __init__(self, root, servo, row=0) -> None:
        self.root = root
        self.row = row
        self.name = "servo" + str(servo.id)
        self._current_value = tk.IntVar()

        self.servo = servo

        offset = self.servo.offset

        ttk.Label(root, text=self.name + ":").grid(row=row, column=0, pady=4, padx=4,sticky='we')

        self.label_text = tk.StringVar()
        self.label_text.set(self._label_text())
        self.label = ttk.Label(root, textvariable=self.label_text)
        self.label.grid(row=row, column=2, padx=20,sticky='we')

        self.slider = ttk.Scale(
            root,
            from_=offset - 50,
            to=offset + 50,
            orient='horizontal',  # vertical
            command=self._slider_changed,
            variable=self._current_value,
            length=1500
        )

        self.slider.set(offset)

        self.slider.grid(
            column=1,
            row=row,
            sticky='e'
        )

    def _slider_changed(self, event):
        self.servo.offset = self._current_value.get()
        self.label_text.set(self._label_text())

    def _label_text(self):
        return str(self._current_value.get())

    def report_current_value(self):
        return str(self._current_value.get())


def setup_servo_editor(servos):
    root = tk.Tk()
    root.geometry('1600x500')
    root.title('Servo editor')

    row_index = 0
    for servo in servos:
        ServoEditor(root, servo, row_index)
        row_index += 1

    def button_clicked():
        res = "["
        for servo in servos:
            res += "{}, ".format(servo.offset)
        res = res[:-2] + "]"
        print(res)

    tk.Button(root,
              text="show offsets",
              anchor="center",
              command=button_clicked
              ).grid(row=row_index, column=1)

    return root
