/* GStreamer
 * Copyright (C) 1999,2000 Erik Walthinsen <omega@cse.ogi.edu>
 *                    2000 Wim Taymans <wtay@chello.be>
 *
 * gstros2clock.c: GStreamer Clock sourcing time from ROS2
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

/**
 * SECTION:gstros2objclock
 * @title: GstRos2ObjClock
 * @short_description: GstClock sourcing time from a ROS2 rcl_clock_t object
 * @see_also: #GstSystemClock
 *
 * #GstRos2ObjClock <TODO>
 *
 */

#include "gstros2objclock.h"

#include <rcl/time.h>
#include <time.h>

GST_DEBUG_CATEGORY_STATIC (gst_ros2_obj_clock_debug);
#define GST_CAT_DEFAULT gst_ros2_obj_clock_debug

static void gst_ros2_obj_clock_constructed (GObject * object);
static void gst_ros2_obj_clock_finalize (GObject * object);

static GstClockTime gst_ros2_obj_clock_get_internal_time (GstClock * clock);

#define parent_class gst_ros2_obj_clock_parent_class

struct _GstRos2ObjClockPrivate {
  gpointer rcl_clock;
};

G_DEFINE_TYPE_WITH_PRIVATE (GstRos2ObjClock, gst_ros2_obj_clock, GST_TYPE_SYSTEM_CLOCK);

static void
gst_ros2_obj_clock_class_init (GstRos2ObjClockClass * klass)
{
  GstClockClass * gstclock_class;
  GObjectClass * gobject_class;

  gobject_class = (GObjectClass *)klass;
  gstclock_class = (GstClockClass *)klass;

  gobject_class->constructed = gst_ros2_obj_clock_constructed;
  gobject_class->finalize = gst_ros2_obj_clock_finalize;

  gstclock_class->get_internal_time = gst_ros2_obj_clock_get_internal_time;

  GST_DEBUG_CATEGORY_INIT (gst_ros2_obj_clock_debug, "ros2objclock", 0, "ros2objclock");
}

static void
gst_ros2_obj_clock_init (GstRos2ObjClock * clock)
{
  GST_DEBUG_OBJECT (clock, "init");
  clock->last_time = 0;
  clock->time_offset = 0;

  clock->priv = gst_ros2_obj_clock_get_instance_private (clock);
  clock->priv->rcl_clock = NULL;
}

static void
gst_ros2_obj_clock_constructed (GObject * object)
{
  GstRos2ObjClock * clock = GST_ROS2_OBJ_CLOCK (object);
  GST_DEBUG_OBJECT (clock, "constructed");

  G_OBJECT_CLASS (parent_class)->constructed (object);
}

static void
gst_ros2_obj_clock_finalize (GObject * object)
{
  GstRos2ObjClock * clock = GST_ROS2_OBJ_CLOCK (object);

  GST_DEBUG_OBJECT (clock, "finalize");

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static GstClockTime
gst_ros2_obj_clock_get_last_timestamp (GstRos2ObjClock * clock)
{
  rcl_clock_t * rcl_clock = (rcl_clock_t *)clock->priv->rcl_clock;
  g_assert (rcl_clock != NULL);
  rcl_time_point_value_t now;

  // Calling get_now doesn't work until after rclcpp::spin() has been called...
  // Perhaps get_now() isn't valid until the node has started receiving events?
  /*rcl_clock->get_now (rcl_clock, &now);*/
  rcl_ret_t r = rcl_clock_get_now (rcl_clock, &now);
  if (r != RCL_RET_OK) {
    GST_WARNING_OBJECT (clock, "Failed to get rcl_clock time\n");
    return GST_CLOCK_TIME_NONE;
  }
  /*GST_DEBUG_OBJECT (clock, "Returning time: %zd\n", now);*/
  return now;
}

static GstClockTime
gst_ros2_obj_clock_get_internal_time (GstClock * clock)
{
  GstRos2ObjClock * aclock;
  GstClockTime result;

  aclock = GST_ROS2_OBJ_CLOCK (clock);

  result = gst_ros2_obj_clock_get_last_timestamp (aclock);
  if (result == GST_CLOCK_TIME_NONE) {
    result = aclock->last_time;
  } else {
    result += aclock->time_offset;
    /* clock must be increasing */
    if (aclock->last_time < result)
      aclock->last_time = result;
    else
      result = aclock->last_time;
  }

  return result;
}

/**
 * gst_ros2_obj_clock_new:
 * @name: the name of the GStreamer clock object
 * @rcl_clock: pointer to a valid rcl_clock_t object
 *
 * Create a new #GstRos2ObjClock instance.
 *
 * Returns: (transfer full): a new #GstClock that reads timestamps from an rcl_clock_t object
 */
GstClock *
gst_ros2_obj_clock_new (const gchar * name, void * rcl_clock)
{
  GstRos2ObjClock * aclock = GST_ROS2_OBJ_CLOCK (g_object_new (gst_ros2_obj_clock_get_type (),
      "name", name, "clock-type", GST_CLOCK_TYPE_OTHER, NULL));
  aclock->priv->rcl_clock = rcl_clock;

  gst_object_ref_sink (aclock);

  return (GstClock *)aclock;
}

