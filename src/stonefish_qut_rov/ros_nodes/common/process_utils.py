"""Start and stop only child processes or groups owned by this session."""
import os
import signal
import subprocess
import time

# Shared by both termination paths below; SIGKILL always ends the ladder.
_ESCALATION = ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0))


def start_process(command, log_file=None, *, new_session=True):
    print("$", " ".join(command), flush=True)
    if log_file is None:
        return subprocess.Popen(command, start_new_session=new_session)
    with open(log_file, "w") as output:
        return subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                start_new_session=new_session)


def stop_process(process, name="process", *, process_group=True):
    if process is None:
        return
    if not process_group:
        for sig, timeout in _ESCALATION:
            if process.poll() is not None:
                return
            process.send_signal(sig)
            try:
                process.wait(timeout=timeout)
                return
            except subprocess.TimeoutExpired:
                continue
        return
    # The leader may already have exited while its children are still alive.
    # Group leaders passed here were started with new_session=True.
    for sig, timeout in _ESCALATION:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()  # Reap an exited leader before checking its group.
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.05)
    process.wait(timeout=1.0)
