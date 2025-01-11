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
        new_val = self._current_value.get()
        print(self.name + ": " + str(new_val))
        self.change_val(self._current_value.get())
        self.label_text.set(self._label_text())

    def _label_text(self):
        return str("{:.3}".format(self._current_value.get()))

    def change_val(self, val):
        pass

    def report_current_value(self):
        return str(self._current_value.get())


def setup_editor(config):
    root = tk.Tk()
    root.geometry('600x500')
    root.title('Config editor')

    row_index = 0

    def change(val):
        config.swing_time = val
    editor = ConfigEditor(root, config.swing_time, config.swing_time, config.swing_time, 'swing time', row=row_index)
    editor.change_val = change
    row_index += 1

    def change(val):
        config.overlap_time = val
    editor = ConfigEditor(root, config.overlap_time, config.overlap_time, config.overlap_time, 'overlap time', row=row_index)
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