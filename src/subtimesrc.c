#include "subtimesrc.h"

typedef rosgraph_msgs__msg__Clock clock_msg_t;

static void
guard_cond_callback (const void * msg)
{
}

static gpointer
clock_receiver_thread (gpointer data)
{
  SubscriptionTimeSource * ts = (SubscriptionTimeSource *)data;
  GST_DEBUG ("entering clock receiver thread");

  while (!ts->stop) {
    // rcl_executor_spin_some() will only process a single incoming event at a time. This means
    // that the maximum rate of message reception is determined by the timeout parameter, i.e
    // timeout=10ms means a max processing rate of 1s/10ms=100 messages per second.

    rcl_ret_t rc = rclc_executor_spin_some (&ts->executor, RCL_MS_TO_NS (10));
    if (rc != RCL_RET_OK) {
      GST_ERROR_OBJECT (clock, "rclc_executor_spin_some failed: %s",
          rcl_get_error_string ().str);
      break;
    }
  }
  GST_DEBUG ("exiting clock receiver thread");
  return NULL;
}

static void
clock_msg_callback (const void * msg, void * context)
{
  const clock_msg_t * clk_msg = (const clock_msg_t *)msg;
  if (clk_msg != NULL) {
    uint64_t ns = clk_msg->clock.sec * 1000000000UL + clk_msg->clock.nanosec;

    GST_DEBUG ("Callback: clock: %u.%u\n", clk_msg->clock.sec, clk_msg->clock.nanosec);

    // atomically update ros2_clock member of SubscriptionTimeSource object
    SubscriptionTimeSource * ts = (SubscriptionTimeSource *)context;
    ts->ros2_clock = ns;
  }
}

SubscriptionTimeSource *
subscription_time_source_create ()
{
  SubscriptionTimeSource * ts = g_malloc (sizeof (SubscriptionTimeSource));

  ts->allocator = rcutils_get_zero_initialized_allocator ();
  ts->ros_node = rcl_get_zero_initialized_node ();
  ts->clock_sub = rcl_get_zero_initialized_subscription ();
  ts->guard_cond = rcl_get_zero_initialized_guard_condition ();
  ts->executor = rclc_executor_get_zero_initialized_executor ();

  ts->thread = NULL;
  ts->stop = FALSE;
  ts->ros2_clock = GST_CLOCK_TIME_NONE;

  return ts;
}

void
subscription_time_source_destroy (SubscriptionTimeSource * s)
{
  g_free (s);
}

gboolean
subscription_time_source_start (gpointer tsp)
{
  rcl_ret_t rc;
  SubscriptionTimeSource * ts = (SubscriptionTimeSource *)tsp;

  ts->allocator = rcl_get_default_allocator ();

  rc = rclc_support_init (&ts->support, 0, NULL, &ts->allocator);
  if (rc != RCL_RET_OK) {
    GST_ERROR ("rclc_support_init failed: %s", rcl_get_error_string ().str);
    return FALSE;
  }

  rc = rclc_node_init_default (&ts->ros_node, "node_0", "executor_examples", &ts->support);
  if (rc != RCL_RET_OK) {
    GST_ERROR ("rclc_node_init_default failed: %s", rcl_get_error_string ().str);
    return FALSE;
  }

  const char * topic_name = "/clock";
  const rosidl_message_type_support_t * clock_type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT (rosgraph_msgs, msg, Clock);

  // create subscription
  ts->clock_sub = rcl_get_zero_initialized_subscription ();
  rc = rclc_subscription_init_best_effort (&ts->clock_sub, &ts->ros_node, clock_type_support,
      topic_name);

  if (rc != RCL_RET_OK) {
    GST_ERROR ("rclc_subscription_init_best_effort failed: %s", rcl_get_error_string ().str);
    return FALSE;
  }

  // create guard condition used to signal thread exit
  rc = rcl_guard_condition_init (&ts->guard_cond, &ts->support.context,
      rcl_guard_condition_get_default_options ());

  if (rc != RCL_RET_OK) {
    GST_ERROR ("rcl_guard_condition_init failed: %s", rcl_get_error_string ().str);
    return FALSE;
  }

  const unsigned int num_handles = 2;
  rclc_executor_init (&ts->executor, &ts->support.context, num_handles, &ts->allocator);

  rosgraph_msgs__msg__Clock__init (&ts->clock_msg);

  rc = rclc_executor_add_subscription_with_context (&ts->executor, &ts->clock_sub,
      &ts->clock_msg, &clock_msg_callback, ts, ON_NEW_DATA);

  if (rc != RCL_RET_OK) {
    GST_ERROR ("rclc_executor_add_subscription_with_context failed: %s",
        rcl_get_error_string ().str);
    return FALSE;
  }

  rc = rclc_executor_add_guard_condition (&ts->executor, &ts->guard_cond, &guard_cond_callback);

  ts->stop = FALSE;
  ts->thread = g_thread_try_new ("ros_clock_receiver", clock_receiver_thread, (void *)ts, NULL);

  if (ts->thread == NULL) {
    GST_ERROR_OBJECT (clock, "failed to create thread");
    return FALSE;
  }

  return TRUE;
}

void
subscription_time_source_finalize (gpointer tsp)
{
  rcl_ret_t rc;
  SubscriptionTimeSource * ts = (SubscriptionTimeSource *)tsp;

  ts->stop = TRUE;
  rc = rcl_trigger_guard_condition (&ts->guard_cond);
  g_thread_join (ts->thread);
  ts->thread = NULL;

  rc = rclc_executor_fini (&ts->executor);
  rc += rcl_guard_condition_fini (&ts->guard_cond);
  rc += rcl_subscription_fini (&ts->clock_sub, &ts->ros_node);
  rc += rcl_node_fini (&ts->ros_node);

  rc = rclc_support_fini (&ts->support);
  if (rc != RCL_RET_OK) {
    GST_WARNING_OBJECT (clock, "error while cleaning up: %s", rcl_get_error_string ().str);
  }

  rosgraph_msgs__msg__Clock__fini (&ts->clock_msg);
}

uint64_t
subscription_time_source_get_last_timestamp (gpointer tsp)
{
  SubscriptionTimeSource * ts = (SubscriptionTimeSource *)tsp;

  return ts->ros2_clock;
}
