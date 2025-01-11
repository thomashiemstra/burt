from time import sleep

import math
import threading
from inputs import get_gamepad, UnpluggedError

from src.Util import auto_str, synchronized_with_lock, is_windows


@auto_str
class ControllerState:
    def __init__(self, l_thumb_x, l_thumb_y, r_thumb_x, r_thumb_y, lr_trigger, start, select, x, y, a, b, rb, lb,
                 pad_up, pad_down, pad_left, pad_right,

                 ):
        self.rb = rb
        self.lb = lb
        self.b = b
        self.a = a
        self.y = y
        self.x = x
        self.start = start
        self.select = select
        self.pad_up = pad_up
        self.pad_down = pad_down
        self.pad_left = pad_left
        self.pad_right = pad_right
        self.l_thumb_x = l_thumb_x
        self.l_thumb_y = l_thumb_y
        self.r_thumb_x = r_thumb_x
        self.r_thumb_y = r_thumb_y
        self.lr_trigger = lr_trigger


class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self,
                 dead_zone=30.0,
                 scale=1,
                 check_controller_present=False):

        self.dev = self.controller_startup()

        self.lower_dead_zone = dead_zone * -1
        self.upper_dead_zone = dead_zone

        self.scale = scale

        self.invert_yaxis_val = 1 if is_windows() else -1
        self.lock = threading.RLock()

        self.l_thumb_x = 0.0
        self.l_thumb_y = 0.0
        self.r_thumb_x = 0.0
        self.r_thumb_y = 0.0
        self.lr_trigger = 0

        self.start = False
        self.select = False
        self.x = False
        self.y = False
        self.a = False
        self.b = False
        self.lb = False
        self.rb = False
        self.pad_up = False
        self.pad_down = False
        self.pad_left = False
        self.pad_right = False

        if is_windows():
            self._monitor_thread = threading.Thread(target=self._monitor_controller_windows, args=())
        else:
            self._monitor_thread = threading.Thread(target=self._monitor_controller_linux, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def controller_startup(self):
        if is_windows():
            self.check_controller_windows()
            return

        res = None
        while True:
            try:
                res = self.check_controller_linux()
            except RuntimeError:
                print("no controller found, retrying")
            if res is not None:
                break
            sleep(1)
        return res

    @staticmethod
    def check_controller_windows():
        from inputs import get_gamepad
        try:
            get_gamepad()
        except UnpluggedError:
            raise

    @staticmethod
    def check_controller_linux():
        from evdev import ecodes, InputDevice, ff, util
        dev = None

        # Find first EV_FF capable event device (that we have permissions to use).
        for name in util.list_devices():
            dev = InputDevice(name)
            if dev.name == 'Xbox Wireless Controller':
                break

        if dev is None:
            raise UnpluggedError()
        return dev

    @synchronized_with_lock("lock")
    def stop(self):
        pass

    @synchronized_with_lock("lock")
    def get_left_thumb(self):
        return self.l_thumb_x, self.l_thumb_y

    @synchronized_with_lock("lock")
    def get_right_thumb(self):
        return self.r_thumb_x, self.r_thumb_y

    @synchronized_with_lock("lock")
    def get_lr_trigger(self):
        return self.lr_trigger

    @synchronized_with_lock("lock")
    def get_controller_state(self):
        state = ControllerState(
            self.l_thumb_x, self.l_thumb_y, self.r_thumb_x, self.r_thumb_y, self.lr_trigger,
            self.start, self.select, self.x, self.y, self.a, self.b, self.rb, self.lb,
            self.pad_up, self.pad_down, self.pad_left, self.pad_right)
        return state

    def _monitor_controller_windows(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.ev_type == 'Sync':
                    continue
                match event.code:
                    case 'ABS_Y':
                        self.__left_thumb_y(event.state)
                    case 'ABS_X':
                        self.__left_thumb_x(event.state)
                    case 'ABS_RY':
                        self.__right_thumb_y(event.state)
                    case 'ABS_RX':
                        self.__right_thumb_x(event.state)
                    case 'ABS_Z':
                        self._left_trigger(event.state)
                    case 'ABS_RZ':
                        self._right_trigger(event.state)
                    case 'BTN_TL':
                        self.___left_bumper(event.state)
                    case 'BTN_TR':
                        self.__right_bumper(event.state)
                    case 'BTN_SOUTH':
                        self.__a(event.state)
                    case 'BTN_NORTH':
                        self.__y(event.state)
                    case 'BTN_WEST':
                        self.__x(event.state)
                    case 'BTN_EAST':
                        self.__b(event.state)
                    case 'BTN_START':
                        self._start(event.state)
                    case 'ABS_HAT0X':
                        self._pad_left_right(event.state)
                    case 'ABS_HAT0Y':
                        self._pad_up_down(event.state)
                    case 'BTN_SELECT':
                        self._select(event.state)

    def _monitor_controller_linux(self):
        while True:
            events = self.dev.read_loop()
            for event in events:
                if event.type not in [1, 3]:
                    continue
                match event.code:
                    case 1:
                        self.__left_thumb_y(event.value)
                    case 0:
                        self.__left_thumb_x(event.value)
                    case 34:
                        self.__right_thumb_y(event.value)
                    case 3:
                        self.__right_thumb_x(event.value)
                    case 2:
                        self._left_trigger(event.value)
                    case 5:
                        self._right_trigger(event.value)
                    case 310:
                        self.___left_bumper(event.value)
                    case 311:
                        self.__right_bumper(event.value)
                    case 304:
                        self.__a(event.value)
                    case 308:
                        self.__y(event.value)
                    case 307:
                        self.__x(event.value)
                    case 305:
                        self.__b(event.value)
                    case 314:
                        self._start(event.value)
                    case 16:
                        self._pad_left_right(event.value)
                    case 17:
                        self._pad_up_down(event.value)
                    case 315:
                        self._select(event.value)

    @synchronized_with_lock("lock")
    def __left_thumb_x(self, x_value):
        self.l_thumb_x = self._handle_x_axis_value(x_value)

    @synchronized_with_lock("lock")
    def __left_thumb_y(self, y_value):
        self.l_thumb_y = self._handle_y_axis_value(y_value)

    @synchronized_with_lock("lock")
    def __right_thumb_x(self, x_value):
        self.r_thumb_x = self._handle_x_axis_value(x_value)

    @synchronized_with_lock("lock")
    def __right_thumb_y(self, y_value):
        self.r_thumb_y = self._handle_y_axis_value(y_value)

    @synchronized_with_lock("lock")
    def __x(self, value):
        self.x = True if value == 1 else False

    @synchronized_with_lock("lock")
    def __y(self, value):
        self.y = True if value == 1 else False

    @synchronized_with_lock("lock")
    def __a(self, value):
        self.a = True if value == 1 else False

    @synchronized_with_lock("lock")
    def __b(self, value):
        self.b = True if value == 1 else False

    @synchronized_with_lock("lock")
    def __right_bumper(self, value):
        self.rb = True if value == 1 else False

    @synchronized_with_lock("lock")
    def ___left_bumper(self, value):
        self.lb = True if value == 1 else False

    @synchronized_with_lock("lock")
    def _left_trigger(self, left_trigger_value):
        self.lr_trigger = -(left_trigger_value / XboxController.MAX_TRIG_VAL) * self.scale

    @synchronized_with_lock("lock")
    def _right_trigger(self, right_trigger_value):
        self.lr_trigger = (right_trigger_value / XboxController.MAX_TRIG_VAL) * self.scale

    @synchronized_with_lock("lock")
    def _pad_left_right(self, value):
        if value == -1:
            self.pad_left = True
        elif value == 1:
            self.pad_right = True
        else:
            self.pad_right, self.pad_left = False, False

    @synchronized_with_lock("lock")
    def _pad_up_down(self, value):
        if value == -1:
            self.pad_up = True
        elif value == 1:
            self.pad_down = True
        else:
            self.pad_down, self.pad_up = False, False

    @synchronized_with_lock("lock")
    def _start(self, value):
        self.start = True if value == 1 else False

    @synchronized_with_lock("lock")
    def _select(self, value):
        self.select = True if value == 1 else False

    def _handle_x_axis_value(self, value):
        value = (value / XboxController.MAX_JOY_VAL) * self.scale

        if self.upper_dead_zone > value > self.lower_dead_zone:
            value = 0
        return value

    def _handle_y_axis_value(self, value):
        value = (value / XboxController.MAX_JOY_VAL) * self.scale * self.invert_yaxis_val

        if self.upper_dead_zone > value > self.lower_dead_zone:
            value = 0
        return value


if __name__ == '__main__':
    joy = XboxController(scale=1, dead_zone=0.3, check_controller_present=True)

    while True:
        print(joy.get_controller_state().__str__())
        sleep(0.1)
