# Setting up camera/gimbal control on another PC

## Diagnosed on 17 September 2026

The Ethernet route was correct, and the ARM cross compiler was installed.
Noninteractive camera SSH failed because this PC had no SSH key. The current
gimbal code requires SSH for motor start/stop before it sends FPV angles over
TCP 2332. Video can therefore work while gimbal control fails.
The installed ROS package also lacked gimbal_motor.py and gimbal_motor.c;
the source CMakeLists already included them, but needed rebuilding.

Fix: generated this PC's own Ed25519 key, added its public key to the camera
using ssh-copy-id, and rebuilt the ROS package. No password was added to code.
Existing camera authorized keys were retained. Firmware checksum matched the
motor helper's supported firmware.

## New-PC setup

1. Install ROS Jazzy and project dependencies, including these camera helpers:

   ```bash
   sudo apt install openssh-client gcc-arm-linux-gnueabihf libc6-dev-armhf-cross python3-pexpect python3-opencv python3-numpy
   ```

2. Find the tether Ethernet adapter/profile with `ip -br address` and
   `nmcli connection show --active`. Add a free address on the camera subnet:

   ```bash
   sudo nmcli connection modify "Wired connection 1" +ipv4.addresses 192.168.144.10/24
   sudo nmcli device reapply enx00e04c680725
   ip route get 192.168.144.108
   ```

   Substitute that PC's actual profile and interface. Route output should use
   the tether adapter and source 192.168.144.10, not the internet gateway.
   Use a different free address if multiple PCs share this network.

3. Establish trusted SSH access:

   ```bash
   ssh root@192.168.144.108
   ```

   Verify the host identity before accepting its key. Exit the remote shell.
   If no suitable local key exists, create one (never overwrite an existing key):

   ```bash
   ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -C rov-camera
   ssh-copy-id -i ~/.ssh/id_ed25519.pub root@192.168.144.108
   ssh -o BatchMode=yes -o StrictHostKeyChecking=yes root@192.168.144.108 'cat /sys/class/thermal/thermal_zone0/temp'
   ```

   Enter the camera password at the prompt. A passphrase-protected key must
   be unlocked in an SSH agent available to the launcher. For this dedicated
   desktop an unencrypted local key was used. Protect the private key; do not
   copy it between PCs or commit it to the repository.

4. Build and source the workspace:

   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/Desktop/EGH490/ros2_ws
   colcon build --packages-select stonefish_qut_rov --symlink-install
   source install/setup.bash
   ls install/stonefish_qut_rov/lib/stonefish_qut_rov/gimbal_motor.*
   ```

5. Start the normal stack. Gimbal control is deliberately active only while
   the camera window is visible. Open it with the camera button. Release the
   D-pad once after opening, then use up/down for pitch and left/right for roll;
   yaw stays fixed. Hiding the window requests motor-off. The current viewer
   defaults are pitch -90 and yaw +90, which differ from the earlier prototype.

## Troubleshooting

- TCP 2332 is angle control; TCP 554 is video; TCP 22 is motor/temperature SSH.
- `Permission denied`: fix this PC's SSH key authorization or agent.
- `arm-linux-gnueabihf-gcc` missing: install the cross compiler.
- Missing gimbal_motor module/source: rebuild and source the correct workspace.
- Unsupported camera firmware: do not bypass the checksum guard. The motor
  helper reads firmware-specific RAM and must be revalidated after updates.
- Do not run a second gimbal controller simultaneously.
- Camera power cycles erase its /tmp helper; the code uploads it again.
- A lost connection can prevent motor-stop confirmation; check the log.

Temperature SSH readings are millidegrees Celsius (62352 means 62.352 C).
Key authentication also removes the need for CAMERA_SSH_PASSWORD for polling.
