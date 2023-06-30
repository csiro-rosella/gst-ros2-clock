/* gst-ros2-clock
 *
 * Copyright 2023 Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 *                ABN 41 687 119 230.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gst/gst.h>
#include <rclc/rclc.h>
#include <stdatomic.h>
#include <stdio.h>

#include "gst-test-pipeline.h"
#include "gstros2subclock.h"

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
  ros2clock = gst_ros2_sub_clock_new ("ROS2Clock", "/clock");
  g_assert (ros2clock);

  launch_test_pipeline (ros2clock);

  gst_object_unref (ros2clock);
  gst_deinit ();

  return 0;
}
