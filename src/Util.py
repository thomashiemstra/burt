import functools
import os


def auto_str(cls):
    def __str__(self):
        return '%s(%s)' % (
            type(self).__name__,
            ', '.join('%s=%s' % item for item in vars(self).items())
        )
    cls.__str__ = __str__
    return cls


def auto_str_newline(cls):
    def __str__(self):
        return '%s(%s)' % (
            type(self).__name__,
            ', '.join('%s=%s \n' % item for item in vars(self).items())
        )
    cls.__str__ = __str__
    return cls


def synchronized_with_lock(lock_name):
    def decorator(method):
        @functools.wraps(method)
        def synced_method(self, *args, **kws):
            lock = getattr(self, lock_name)
            with lock:
                return method(self, *args, **kws)

        return synced_method

    return decorator


def is_windows():
    return os.name == 'nt'
