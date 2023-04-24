/* GStreamer
 * Copyright (C) 1999,2000 Erik Walthinsen <omega@cse.ogi.edu>
 *                    2005 Wim Taymans <wim@fluendo.com>
 *
 * gstros2clock.h: Clock for use by ros2 plugins
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

#ifndef __GST_ROS2_CLOCK_H__
#define __GST_ROS2_CLOCK_H__

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/gstsystemclock.h>

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE (GstRos2Clock, gst_ros2_clock, GST, ROS2_CLOCK, GstSystemClock)

typedef struct _GstRos2ClockPrivate GstRos2ClockPrivate;

/**
 * GstRos2Clock: A GstClock implementation that reports time as received
 * from the ROS2 topic /clock.
 *
 */
struct _GstRos2Clock {
  GstSystemClock clock;

  /*< private >*/
  GstClockTime last_time;
  GstClockTimeDiff time_offset;

  // keep ros2 related declarations private to avoid making ros2clock users
  // dependent on ROS2 headers
  GstRos2ClockPrivate * priv;

  gpointer _gst_reserved[GST_PADDING];
};

GST_API_EXPORT GstClock * gst_ros2_clock_new_from_subscription (const gchar * name,
    const gchar * topic);
GST_API_EXPORT GstClock * gst_ros2_clock_new_from_rcl_clock (const gchar * name,
    void * rcl_clock);

G_END_DECLS

#endif /* __GST_ROS2_CLOCK_H__ */