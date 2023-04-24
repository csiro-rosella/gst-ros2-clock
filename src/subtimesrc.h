/*
 * subtimesrc.h - get time by subscribing to ROS2 topic
 */
#pragma once

#include <gst/gst.h>
#include <inttypes.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosgraph_msgs/msg/clock.h>
#include <stdatomic.h>

typedef struct {
  GThread * thread;
  atomic_bool stop;
  atomic_uint_fast64_t ros2_clock;

  rcl_allocator_t allocator;
  rclc_support_t support;
  rcl_node_t ros_node;
  rcl_subscription_t clock_sub;
  rcl_guard_condition_t guard_cond;
  rclc_executor_t executor;

  rosgraph_msgs__msg__Clock clock_msg;
} SubscriptionTimeSource;

SubscriptionTimeSource * subscription_time_source_create ();
void subscription_time_source_destroy (SubscriptionTimeSource *);
gboolean subscription_time_source_start (gpointer);
void subscription_time_source_finalize (gpointer);

uint64_t subscription_time_source_get_last_timestamp (gpointer);
