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

#ifndef __GST_ROS2_OBJ_CLOCK_H__
#define __GST_ROS2_OBJ_CLOCK_H__

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/gstsystemclock.h>

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE (GstRos2ObjClock, gst_ros2_obj_clock, GST, ROS2_OBJ_CLOCK, GstSystemClock)

typedef struct _GstRos2ObjClockPrivate GstRos2ObjClockPrivate;

/**
 * GstRos2ObjClock: A GstClock implementation that reports time as received
 * from the ROS2 topic /clock.
 *
 */
struct _GstRos2ObjClock {
  GstSystemClock clock;

  /*< private >*/
  GstClockTime last_time;
  GstClockTimeDiff time_offset;

  // keep ros2 related declarations private to avoid making ros2clock users
  // dependent on ROS2 headers
  GstRos2ObjClockPrivate * priv;

  gpointer _gst_reserved[GST_PADDING];
};

GST_API_EXPORT GstClock * gst_ros2_obj_clock_new (const gchar * name, void * rcl_clock);

G_END_DECLS

#endif /* __GST_ROS2_OBJ_CLOCK_H__ */
