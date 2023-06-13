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
 * SECTION:gstros2subclock
 * @title: GstRos2SubClock
 * @short_description: GstClock sourcing time from ROS2 clock messages
 * or directly from a rcl_clock_t object
 * @see_also: #Gstros2BaseSink, #GstSystemClock
 *
 * #GstRos2SubClock <TODO>
 *
 */

#include "gstros2subclock.h"

#include <rcl/time.h>
#include <rclc/rclc.h>
#include <time.h>

#include "subtimesrc.h"

GST_DEBUG_CATEGORY_STATIC (gst_ros2_sub_clock_debug);
#define GST_CAT_DEFAULT gst_ros2_sub_clock_debug

static void gst_ros2_sub_clock_constructed (GObject * object);
static void gst_ros2_sub_clock_finalize (GObject * object);

static GstClockTime gst_ros2_sub_clock_get_internal_time (GstClock * clock);

#define parent_class gst_ros2_sub_clock_parent_class

struct _GstRos2SubClockPrivate {
  SubscriptionTimeSource * sub_src;
};

G_DEFINE_TYPE_WITH_PRIVATE (GstRos2SubClock, gst_ros2_sub_clock, GST_TYPE_SYSTEM_CLOCK);

static void
gst_ros2_sub_clock_class_init (GstRos2SubClockClass * klass)
{
  GstClockClass * gstclock_class;
  GObjectClass * gobject_class;

  gobject_class = (GObjectClass *)klass;
  gstclock_class = (GstClockClass *)klass;

  gobject_class->constructed = gst_ros2_sub_clock_constructed;
  gobject_class->finalize = gst_ros2_sub_clock_finalize;

  gstclock_class->get_internal_time = gst_ros2_sub_clock_get_internal_time;

  GST_DEBUG_CATEGORY_INIT (gst_ros2_sub_clock_debug, "ros2clock", 0, "ros2clock");
}

static void
gst_ros2_sub_clock_init (GstRos2SubClock * clock)
{
  GST_DEBUG_OBJECT (clock, "init");
  clock->last_time = 0;
  clock->time_offset = 0;

  clock->priv = gst_ros2_sub_clock_get_instance_private (clock);
  clock->priv->sub_src = NULL;
}

static void
gst_ros2_sub_clock_constructed (GObject * object)
{
  GstRos2SubClock * clock = GST_ROS2_SUB_CLOCK (object);
  GST_DEBUG_OBJECT (clock, "constructed");

  G_OBJECT_CLASS (parent_class)->constructed (object);

  clock->priv->sub_src = subscription_time_source_create ();
  if (!subscription_time_source_start (clock->priv->sub_src)) {
    g_warning ("failed to start clock '%s'", GST_OBJECT_NAME (object));
  }
}

static void
gst_ros2_sub_clock_finalize (GObject * object)
{
  GstRos2SubClock * clock = GST_ROS2_SUB_CLOCK (object);

  GST_DEBUG_OBJECT (clock, "finalize");

  subscription_time_source_finalize (clock->priv->sub_src);
  subscription_time_source_destroy (clock->priv->sub_src);
  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static GstClockTime
gst_ros2_sub_clock_get_last_timestamp (GstRos2SubClock * clock)
{
  return subscription_time_source_get_last_timestamp (clock->priv->sub_src);
}

static GstClockTime
gst_ros2_sub_clock_get_internal_time (GstClock * clock)
{
  GstRos2SubClock * aclock;
  GstClockTime result;

  aclock = GST_ROS2_SUB_CLOCK (clock);

  result = gst_ros2_sub_clock_get_last_timestamp (aclock);
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
 * gst_ros2_sub_clock_new
 * @name: the name of the clock
 * @topic: the ros2 topic to subscribe to for Clock messages
 *
 * Create a new #GstRos2SubClock instance.
 *
 * Returns: (transfer full): a new #GstClock that receives timestamps from a ROS2 topic
 */
GstClock *
gst_ros2_sub_clock_new (const gchar * name, const gchar * topic)
{
  // TODO: actually use topic argument

  GstRos2SubClock * aclock = GST_ROS2_SUB_CLOCK (g_object_new (gst_ros2_sub_clock_get_type (),
      "name", name, "clock-type", GST_CLOCK_TYPE_OTHER, NULL));

  gst_object_ref_sink (aclock);

  return (GstClock *)aclock;
}
