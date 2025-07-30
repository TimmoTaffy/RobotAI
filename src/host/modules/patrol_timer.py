import threading

class PatrolTimer:
    def __init__(self, interval, callback=None):
        self.interval = interval
        self.callback = callback
        self._timer = None

    def _run(self):
        if self.callback:
            self.callback()
        # 定时重复
        self._timer = threading.Timer(self.interval, self._run)
        self._timer.start()

    def start(self):
        if not self._timer:
            self._timer = threading.Timer(self.interval, self._run)
            self._timer.start()

    def stop(self):
        if self._timer:
            self._timer.cancel()
            self._timer = None
