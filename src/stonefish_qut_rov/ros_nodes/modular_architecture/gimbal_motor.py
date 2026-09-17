"""Motor enable/disable for the verified Z-1 Mini controller, over key-auth SSH.

The helper reads firmware-specific RAM snapshots, so refuse other firmware.
Only a temporary helper is uploaded; no persistent camera settings are written.
"""
import hashlib
import ipaddress
from pathlib import Path
import subprocess
import tempfile


CONTROLLER_SHA256 = 'bbb5d68d3b28ebf2516ee1e4068c351b0563d02731ec377a1d10d3d03093fdb3'


class MotorControl:
    def __init__(self, host):
        ipaddress.IPv4Address(host)
        self.ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                    '-o', 'ConnectTimeout=2', f'root@{host}']
        self.remote = None

    @staticmethod
    def run(command, **kwargs):
        try:
            return subprocess.run(command, check=True, capture_output=True,
                                  timeout=8, start_new_session=True, **kwargs).stdout
        except subprocess.CalledProcessError as exc:
            raise OSError(exc.stderr.decode(errors='replace').strip() or
                          f'Gimbal helper failed ({exc.returncode})') from exc
        except subprocess.TimeoutExpired as exc:
            raise OSError('Gimbal SSH/build timed out') from exc

    def prepare(self):
        source = Path(__file__).resolve().with_suffix('.c')
        digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
        self.remote = f'/tmp/rov-gimbal-motor-{digest}'
        with tempfile.TemporaryDirectory(prefix='rov-gimbal-build-') as directory:
            binary = Path(directory) / 'motor'
            self.run(['arm-linux-gnueabihf-gcc', '-static', '-O2', '-Wall', '-Wextra',
                      '-Werror', str(source), '-o', str(binary)])
            self.run(self.ssh + [f'umask 077; cat > {self.remote}.new && '
                                f'chmod 700 {self.remote}.new && '
                                f'mv {self.remote}.new {self.remote}'], input=binary.read_bytes())

    def set_enabled(self, enabled):
        if self.remote is None:
            try:
                self.prepare()
            except OSError:
                self.remote = None
                raise
        action = 'start' if enabled else 'stop'
        command = (
            'set -e; '
            'actual=$(sha256sum /opt/bin/gcu/gb_control); '
            f'[ "${{actual%% *}}" = {CONTROLLER_SHA256} ] || '
            '{ echo "Unsupported camera firmware; no motor command sent" >&2; exit 9; }; '
            'set -- $(pidof gb_control); [ "$#" = 1 ] || exit 10; '
            f'{self.remote} "$1" {action}'
        )
        try:
            self.run(self.ssh + [command])
        except OSError:
            # /tmp disappears when the camera reboots; redeploy on next retry.
            self.remote = None
            raise
