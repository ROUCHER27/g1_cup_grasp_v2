"""The arm adapter plus a receipt-time watchdog for motion status."""
import time
from .robot import GdkRightArm


class FreshTransportArm(GdkRightArm):
    def __init__(self, config, pose_model=None):
        super().__init__(config,pose_model)
        self._last_status_received = 0.0
        core = self._controller._motion_controller
        # The installed DDS wrapper rejects a second subscription to the same topic.
        with core.node_.mutex:
            subscription = core.node_.subs[core.motion_control_status_topic]
            self._sdk_status_callback = subscription[1]
            core.node_.subs[core.motion_control_status_topic] = (
                subscription[0], self._received_status, *subscription[2:])

    def _received_status(self, message):
        self._sdk_status_callback(message)
        self._last_status_received = time.monotonic()

    def read_ready_state(self, timeout_s=5.0):
        deadline = time.monotonic()+timeout_s
        while True:
            state = super().read_ready_state(timeout_s)
            if state.raw is not None and time.monotonic()-self._last_status_received <= 1.0:
                return state
            if time.monotonic() >= deadline:
                raise RuntimeError("motion-control status is not publishing/updating; restore official controller before execution")
            time.sleep(.1)

    def assert_execution_ready(self,state):
        if time.monotonic()-self._last_status_received > 1.0:
            raise RuntimeError("motion-control status is stale")
        super().assert_execution_ready(state)