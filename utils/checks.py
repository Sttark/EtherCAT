import functools
import time
from typing import Callable, Optional


class PendingCheck:
    def __init__(self, name: str, deadline: float, verify: Callable[[], bool], heal: Optional[Callable[[], None]] = None):
        self.name = name
        self.deadline = deadline
        self.verify = verify
        self.heal = heal


def nonblocking_check(name: str, timeout_s: float = 1.0, heal: Optional[Callable] = None):
    """
    Decorator to schedule a self-check without blocking the caller.
    Assumes the instance has an attribute `_pending_checks` (list) that the
    cyclic process will poll and attempt.
    """

    def decorator(func):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            result = func(self, *args, **kwargs)

            def verify():
                try:
                    return bool(self._verify_last_action())
                except Exception:
                    return False

            deadline = time.time() + timeout_s
            self._pending_checks.append(PendingCheck(name=name, deadline=deadline, verify=verify, heal=heal))
            return result

        return wrapper

    return decorator



