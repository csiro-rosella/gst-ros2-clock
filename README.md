## GStreamer clock source returning ROS2 timestamps

This GStreamer plugin provides a GstClock implementation that returns
timestamps received from the ROS2 `/clock` topic or directly from an
`rclclock_t` object.

### Dependencies

* Meson build system - v0.59 or later 
* ROS2 core client library - rcl (required)
* ROS2 C client library - rclc (optional, for /clock topic subscription)
* ROS2 C++ client library - rclcpp (optional, for C++ example)
* GStreamer core library - currently only tested with GStreamer 1.16.3 - 1.20.3
* rclpy - for python test script

Tested with ROS2 Galactic, Humble and Iron.

##### Ubuntu 22.04 Dependency Installation (ROS2 Humble)
```
sudo apt install libgstreamer1.0-dev ros-humble-rcl 
sudo apt install ros-humble-rclc ros-humble-rclcpp ros-humble-rosgraph-msgs # (optional)
pip install meson
pip install rclpy # (optional)
```

### Building

```bash
meson setup build
ninja -C build
```

The meson option `subscription_mode` is available to toggle support for using a ROS2
topic as clock source. See meson_options.txt for more info. To disable subscription support
run the meson command as follows:

```bash
meson setup -D subscription_mode=disabled build
```

### Testing

#### Using clock messages as time source
With ROS2 environment sourced (to find rclc libs etc), start the test program:

```
./build/test/test-ros2-clock
```
This will launch a simple gstreamer pipeline containing a videotestsrc element
generating a circle rotating around the center of the video frames
(pattern=ball). Using the argument flip=1, the colors will be inverted every
second (as reported by the pipeline clock).

The pipeline is configured to use the GstRos2Clock clock source. Unless
something is already publishing timestamps on the ROS2 /clock topic, the circle
will initially be static.

To generate timestamps, run the `tools/clock-publisher.py` python script:

```python
python tools/clock-publisher.py --start=0 --pub-rate=100 --time-step=20000000
```
This will publish 100 clock messages per second, stepping the time forward by
20000000ns = 0.02 seconds each time, thus advancing the pipeline time at twice the
speed of the regular system clock.

#### Using an RCL clock object as time source
The `node-clock-source` program will create a ROS2 node that publishes text messages to the */hello*
topic at regular intervals. It will also create a GStreamer pipeline that is configured to use a
GstRos2Clock that uses an rcl_clock_t object as it's time source. To start it
using the regular ROS2 SystemClock as time source, simply run without arguments:

```
./build/test/node-clock-source
```
To run it with simulated time received on the *clock_* topic, run it like this:
```
./build/test/node-clock-source --ros-args -p use_sim_time:=true
```
In order for anything to happen, something needs to publish timestamps on the */clock* topic.

