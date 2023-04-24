
#include <gst/gst.h>
#include <rclc/rclc.h>
#include <stdatomic.h>
#include <stdio.h>

#include "gst-test-pipeline.h"
#include "gstros2clock.h"

#if 0
static rcl_ret_t
test_get_now (void * data, rcl_time_point_value_t * now)
{
  static rcl_time_point_value_t ticks = 0;

  printf ("TICK: %zd\n", ticks);
  *now = ticks;
  ticks += 5000000;
  return RCL_RET_OK;
}
#endif

int
main (int argc, char * argv[])
{
  gst_init (&argc, &argv);
  print_gst_version ();

  GstClock * ros2clock;

  /*rcl_clock_t rcl_clock;*/
  /*rcl_clock.get_now = test_get_now;*/
  /*ros2clock = gst_ros2_clock_new_from_rcl_clock ("ROS2Clock", &rcl_clock);*/
  ros2clock = gst_ros2_clock_new_from_subscription ("ROS2Clock", "/clock");
  g_assert (ros2clock);

  launch_test_pipeline (ros2clock);

  gst_object_unref (ros2clock);
  gst_deinit ();

  return 0;
}
