from __future__ import annotations

import os
import sys


def exit_without_sdk_thread_wait(code: int) -> None:
    # The GDK leaves native DDS threads alive; all result files are already closed.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(int(code))
