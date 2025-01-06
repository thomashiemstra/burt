import functools

from inputs import get_gamepad
import math
import threading


def synchronized_with_lock(lock_name):
    def decorator(method):
        @functools.wraps(method)
        def synced_method(self, *args, **kws):
            lock = getattr(self, lock_name)
            with lock:
                return method(self, *args, **kws)

        return synced_method

    return decorator


class ControllerState:
    def __init__(self, l_thumb_x, l_thumb_y, r_thumb_x, r_thumb_y, lr_trigger, start, x, y, a, b, rb, lb,
                 pad_up, pad_down, pad_left, pad_right,

                 ):
        self.rb = rb
        self.lb = lb
        self.b = b
        self.a = a
        self.y = y
        self.x = x
        self.start = start
        self.pad_up = pad_up
        self.pad_down = pad_down
        self.pad_left = pad_left
        self.pad_right = pad_right
        self.l_thumb_x = l_thumb_x
        self.l_thumb_y = l_thumb_y
        self.r_thumb_x = r_thumb_x
        self.r_thumb_y = r_thumb_y
        self.lr_trigger = lr_trigger


    def __str__(self):
        return 'Buttons: start={} x={} y={} a={}, b={}, rb={}, lb={}, ' \
               'pad_up={}. pad_down={}. pad_left={}, pad_right={}' \
            .format(self.start, self.x, self.y, self.a, self.b, self.rb, self.lb,
                    self.pad_up, self.pad_down, self.pad_left, self.pad_right)


class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self,
                 dead_zone=30,
                 scale=1,
                 invert_yaxis=False):
        self.lower_dead_zone = dead_zone * -1
        self.upper_dead_zone = dead_zone

        self.scale = scale
        self.invert_yaxis_val = -1 if invert_yaxis else 1
        self.lock = threading.RLock()

        self.l_thumb_x = 0
        self.l_thumb_y = 0
        self.r_thumb_x = 0
        self.r_thumb_y = 0
        self.lr_trigger = 0

        self.start = False
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

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

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
            self.l_thumb_x, self.l_thumb_y, self.r_thumb_x, self.r_thumb_y, self.lr_trigger, self.start, self.x, self.y,
            self.a, self.b, self.rb, self.lb,
            self.pad_up, self.pad_down, self.pad_left, self.pad_right)
        return state

    def _monitor_controller(self):
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
    joy = XboxController(scale=100)
    while True:
        print(joy.get_controller_state())
