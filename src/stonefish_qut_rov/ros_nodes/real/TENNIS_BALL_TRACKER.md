# Tennis-ball tracker

Start the existing ROV launcher in real mode. The camera viewer starts with
teleop and detects round yellow objects using OpenCV HSV thresholding only
while the camera window is shown. Hiding or closing the window stops detection,
clears the target, and publishes zero commands on the next tracker tick (15 Hz).
Reopening requires three fresh detections before tracking resumes.
Use the camera button (index 4) to show the overlay. The follow button (index 3)
toggles tennis-ball centering in real mode. Existing vehicle arming still applies.
Simulation retains its existing fish follower.

The largest qualifying yellow contour supplies `(x, y)` in original frame pixels:

```text
horizontal_error = (x - width/2) / (width/2)
vertical_error   = (y - height/2) / (height/2)
yaw   = yaw_sign   * clip(gain * horizontal_error, -maximum, maximum)
heave = heave_sign * clip(gain * vertical_error,   -maximum, maximum)
```

Errors within the deadband produce zero. Defaults: gain 0.18, maximum 0.15,
deadband 0.08, and both signs -1. These are normalized vehicle commands,
not angles or distances. Confirm direction with the installed camera orientation;
change the corresponding sign to +1 if required.

Camera-node ROS parameters: `ball_gain`, `ball_max_command` (up to 0.3),
`ball_deadband`, `ball_yaw_sign`, `ball_heave_sign`, `ball_h_min` (20),
`ball_h_max` (45), `ball_s_min` (90), and `ball_v_min` (80).
HSV hue uses OpenCV's 0–179 range. Parameters are read at node startup.

The tracker requires three consecutive detections and camera-health permission.
Missing detections or frames older than 0.25 seconds produce zero output.
Teleop sends the enable heartbeat at 20 Hz; it expires after 0.5 seconds.
Joystick and command watchdogs remain active. Surge is always zero in ball mode.
The overlay shows detected pixels and whether motion is active.

Topics: `/qut_rov/tennis_ball_cmd` (`geometry_msgs/Twist`, angular.z yaw,
linear.z heave), `/qut_rov/tennis_ball_enabled` (`std_msgs/Bool` heartbeat),
and `/qut_rov/tennis_ball_target_valid` (`std_msgs/Bool`). Teleop routes the
selected commands through `/qut_rov/cmd_vel` and the existing mixer.
