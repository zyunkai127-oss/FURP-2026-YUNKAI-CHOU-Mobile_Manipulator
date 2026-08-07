# Troubleshooting and lessons learned

## ROS graph exists but no messages arrive

The most common cause was inconsistent environments between terminals. Every
camera, estimator, fusion, TF, and CLI terminal must use the same
`RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY`, `ROS_DOMAIN_ID`, and
`CYCLONEDDS_URI`. A process started with `ROS_LOCALHOST_ONLY=1` is invisible to
a terminal using `0`. After correcting the environment, restart discovery:

```bash
ros2 daemon stop
sleep 2
ros2 daemon start
```

The message `selected interface "lo" is not multicast-capable` is expected
with localhost-only CycloneDDS and was not itself the failure.

## Camera node/topic appears, but CameraInfo and images are silent

Check the actual process environment under `/proc/PID/environ`, not only the
current shell. Also inspect the launch terminal for `Get first color frame` and
`Get first depth frame`. Topic metadata proves endpoints exist; it does not
prove frames are flowing.

## `uvc_open failed ret = -6`

Another camera process owned the USB device. Use `ps` and `sudo fuser -v
/dev/bus/usb/...`, stop the old process, then launch exactly one node per serial.
Do not start the second camera while an old process still holds its device.

## QoS incompatibility in RViz/estimator

The Berxel publisher used RELIABLE QoS in the final configuration. Subscriber
reliability must be compatible. Earlier BEST_EFFORT/RELIABLE mismatches caused
blank RViz displays and silent callbacks. Inspect with `ros2 topic info
TOPIC --verbose`.

## Marker status has no output

First use the marker viewer. Several apparent ROS failures were simply ID 4
being hidden by the arm or outside one camera's view. Both cameras must see the
marker and have valid depth at its centre.

## Large RGB/depth timestamp skew

Pairing the newest RGB with only the newest depth produced 165–500 ms skew.
The estimator now stores a depth deque and selects the timestamp nearest the
RGB frame. The fusion node similarly buffers both camera point streams. Do not
replace this with a single last-message variable.

## `base_link` and camera frames are separate TF trees

The cameras publish their internal links but do not know the robot base. Start
the saved static transforms, or use the explicitly labelled corrected
calibration fallback. `tf2_echo` may print an initial “frame does not exist”
while discovery starts and then print the transform; the later valid matrix is
the meaningful result.

## Wrong package is executed after rebuilding

Both `~/ros2_ws` and `~/berxel_ros2_ws` once contained the same package name.
Confirm with `ros2 pkg prefix dual_berxel_calibration`. Source `/opt/ros/humble`
and then only the intended workspace overlay.

## CycloneDDS participant index exhaustion

`Failed to find a free participant index` came from many short-lived CLI nodes.
Close unused terminals/nodes and use `MaxAutoParticipantIndex=50`. Avoid
launching many `ros2 topic echo` processes concurrently.

## Python/package defects fixed during development

- `Node.subscriptions` is not assignable; the estimator uses `_subscriptions`.
- The installed console script did not update because the wrong overlay was
  sourced; confirm the package prefix after `colcon build --symlink-install`.
- `setup.py` now exports the accuracy recorder as well as detector, estimator,
  calibrator, and fusion nodes.
- `source setup.bashsh` and a missing value after `--size` were shell typos, not
  driver problems.

## `BrokenPipeError` after `ros2 topic echo | head`

This is expected: `head` exits after N lines and closes the pipe while ROS is
still writing. If the CSV has the requested line count, the capture succeeded.
The first capture occasionally ended with a truncated final line, so files are
validated by line count and numeric parsing before analysis.

## Calibration accuracy does not improve by filtering stable samples

Filtering can reduce random depth noise, but it cannot remove systematic frame,
target-centre, robot-TCP, or calibration bias. More samples at one location
only repeat the same bias. Lower absolute error requires spatially distributed
3-D calibration points, consistent robot tilt/roll and TCP definition, a rigid
target, and independent validation points.

## Guarded approach stopped or missed the planned waypoint

The original guard stopped at 53.2 mm tracking error. Faster three-step motion
was later requested, but fewer steps reduce opportunities to detect IK/tracking
failure. The 10 mm endpoint is smaller than the observed 10–12 mm calibration
error and must remain supervised. Removing a relative-error abort does not make
the system safer; preserve high-clearance waypoints and an absolute workspace
limit, keep the emergency stop accessible, and never command autonomous table
contact with this calibration.
