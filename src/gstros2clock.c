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
 * SECTION:gstros2clock
 * @title: GstRos2Clock
 * @short_description: GstClock sourcing time from ROS2 clock messages
 * or directly from a rcl_clock_t object
 * @see_also: #Gstros2BaseSink, #GstSystemClock
 *
 * #GstRos2Clock <TODO>
 *
 */

#include "gstros2clock.h"

#include <rcl/time.h>
#include <rclc/rclc.h>
#include <time.h>

#include "subtimesrc.h"

GST_DEBUG_CATEGORY_STATIC (gst_ros2_clock_debug);
#define GST_CAT_DEFAULT gst_ros2_clock_debug

static void gst_ros2_clock_constructed (GObject * object);
static void gst_ros2_clock_dispose (GObject * object);
static void gst_ros2_clock_finalize (GObject * object);

static void gst_ros2_clock_set_property (GObject * object, guint prop_id, const GValue * value,
    GParamSpec * pspec);
static void gst_ros2_clock_get_property (GObject * object, guint prop_id, GValue * value,
    GParamSpec * pspec);

static GstClockTime gst_ros2_clock_get_internal_time (GstClock * clock);

#define parent_class gst_ros2_clock_parent_class

enum { TIME_SOURCE_SUBSCRIBER, TIME_SOURCE_CALLBACK };

enum { PROP_RCL_CLOCK_PTR = 1000 };

struct _GstRos2ClockPrivate {
  gpointer time_source;
  int time_source_type;
};

G_DEFINE_TYPE_WITH_PRIVATE (GstRos2Clock, gst_ros2_clock, GST_TYPE_SYSTEM_CLOCK);

static void
gst_ros2_clock_class_init (GstRos2ClockClass * klass)
{
  GstClockClass * gstclock_class;
  GObjectClass * gobject_class;

  gobject_class = (GObjectClass *)klass;
  gstclock_class = (GstClockClass *)klass;

  gobject_class->constructed = gst_ros2_clock_constructed;
  gobject_class->dispose = gst_ros2_clock_dispose;
  gobject_class->finalize = gst_ros2_clock_finalize;

  gobject_class->set_property = gst_ros2_clock_set_property;
  gobject_class->get_property = gst_ros2_clock_get_property;

  g_object_class_install_property (gobject_class, PROP_RCL_CLOCK_PTR,
      g_param_spec_pointer ("rcl-clock", "Pointer to valid rcl_clock_t object", "---",
          G_PARAM_READWRITE | G_PARAM_CONSTRUCT_ONLY));

  gstclock_class->get_internal_time = gst_ros2_clock_get_internal_time;

  GST_DEBUG_CATEGORY_INIT (gst_ros2_clock_debug, "ros2clock", 0, "ros2clock");
}

static void
gst_ros2_clock_set_property (GObject * object, guint prop_id, const GValue * value,
    GParamSpec * pspec)
{
  GstRos2Clock * clock = GST_ROS2_CLOCK (object);

  GST_DEBUG_OBJECT (object, "prop_id = %u\n", prop_id);
  switch (prop_id) {
    case PROP_RCL_CLOCK_PTR:
      void * p = g_value_get_pointer (value);
      if (p) {
        clock->priv->time_source = p;
        GST_DEBUG_OBJECT (object, "time-source set to %p", clock->priv->time_source);
        clock->priv->time_source_type = TIME_SOURCE_CALLBACK;
      }
      /*GST_INFO_OBJECT (object, "time-source-type set to %d", clock->priv->time_source_type);*/
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_ros2_clock_get_property (GObject * object, guint prop_id, GValue * value,
    GParamSpec * pspec)
{
  GstRos2Clock * clock = GST_ROS2_CLOCK (object);

  switch (prop_id) {
    case PROP_RCL_CLOCK_PTR:
      g_value_set_pointer (value, clock->priv->time_source);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_ros2_clock_init (GstRos2Clock * clock)
{
  GST_DEBUG_OBJECT (clock, "init");
  clock->last_time = 0;
  clock->time_offset = 0;

  clock->priv = gst_ros2_clock_get_instance_private (clock);
  clock->priv->time_source = NULL;
  clock->priv->time_source_type = TIME_SOURCE_SUBSCRIBER;
}

static void
gst_ros2_clock_constructed (GObject * object)
{
  GstRos2Clock * clock = GST_ROS2_CLOCK (object);
  GST_DEBUG_OBJECT (clock, "constructed, time-source-type = %d", clock->priv->time_source_type);

  G_OBJECT_CLASS (parent_class)->constructed (object);

  if (clock->priv->time_source_type == TIME_SOURCE_SUBSCRIBER) {
    clock->priv->time_source = subscription_time_source_create ();
    if (!subscription_time_source_start (clock->priv->time_source)) {
      g_warning ("failed to start clock '%s'", GST_OBJECT_NAME (object));
    }
  }
}

static void
gst_ros2_clock_dispose (GObject * object)
{
  /*GstRos2Clock * clock = GST_ROS2_CLOCK (object);*/
  /*GST_DEBUG_OBJECT (clock, "dispose");*/

  G_OBJECT_CLASS (parent_class)->dispose (object);
}

static void
gst_ros2_clock_finalize (GObject * object)
{
  GstRos2Clock * clock = GST_ROS2_CLOCK (object);

  GST_DEBUG_OBJECT (clock, "finalize");

  if (clock->priv->time_source_type == TIME_SOURCE_SUBSCRIBER) {
    subscription_time_source_finalize (clock->priv->time_source);
    subscription_time_source_destroy (clock->priv->time_source);
  }
  G_OBJECT_CLASS (parent_class)->finalize (object);
}

static GstClockTime
gst_ros2_clock_get_last_timestamp (GstRos2Clock * clock)
{
  if (clock->priv->time_source_type == TIME_SOURCE_SUBSCRIBER) {
    return subscription_time_source_get_last_timestamp (clock->priv->time_source);
  } else {
    rcl_clock_t * rcl_clock = (rcl_clock_t *)clock->priv->time_source;
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
}

static GstClockTime
gst_ros2_clock_get_internal_time (GstClock * clock)
{
  GstRos2Clock * aclock;
  GstClockTime result;

  aclock = GST_ROS2_CLOCK (clock);

  result = gst_ros2_clock_get_last_timestamp (aclock);
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

  /*GST_DEBUG_OBJECT (clock, "result %" GST_TIME_FORMAT ", last_time %" GST_TIME_FORMAT,*/
  /*GST_TIME_ARGS (result), GST_TIME_ARGS (aclock->last_time));*/

  return result;
}

/**
 * gst_ros2_clock_new_from_subscription:
 * @name: the name of the clock
 * @topic: the ros2 topic to subscribe to for Clock messages
 *
 * Create a new #GstRos2Clock instance.
 *
 * Returns: (transfer full): a new #GstClock that receives timestamps from a ROS2 topic
 */
GstClock *
gst_ros2_clock_new_from_subscription (const gchar * name, const gchar * topic)
{
  // TODO: actually use topic argument
  GstRos2Clock * aclock = GST_ROS2_CLOCK (g_object_new (gst_ros2_clock_get_type (), "name",
      name, "clock-type", GST_CLOCK_TYPE_OTHER, NULL));

  gst_object_ref_sink (aclock);

  return (GstClock *)aclock;
}

/**
 * gst_ros2_clock_new_from_rcl_clock:
 * @name: the name of the GStreamer clock object
 * @rcl_clock: pointer to a valid rcl_clock_t object
 *
 * Create a new #GstRos2Clock instance.
 *
 * Returns: (transfer full): a new #GstClock that reads timestamps from an rcl_clock_t object
 */
GstClock *
gst_ros2_clock_new_from_rcl_clock (const gchar * name, void * rcl_clock)
{
  GstRos2Clock * aclock = GST_ROS2_CLOCK (g_object_new (gst_ros2_clock_get_type (), "name",
      name, "clock-type", GST_CLOCK_TYPE_OTHER, "rcl-clock", rcl_clock, NULL));

  gst_object_ref_sink (aclock);

  return (GstClock *)aclock;
}

