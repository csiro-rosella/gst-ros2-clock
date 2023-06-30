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
