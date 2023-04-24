
#include <gst/gst.h>
#include <stdio.h>

void
print_gst_version ()
{
  const gchar * nano_str;
  guint major, minor, micro, nano;
  gst_version (&major, &minor, &micro, &nano);

  if (nano == 1)
    nano_str = "(CVS)";
  else if (nano == 2)
    nano_str = "(Prerelease)";
  else
    nano_str = "";

  printf ("This program is linked against GStreamer %d.%d.%d %s\n", major, minor, micro,
      nano_str);
}

int
launch_test_pipeline (GstClock * clock)
{
  GstElement *source, *filter, *sink;

  GstElement * pipeline;
  GstBus * bus;
  GstMessage * msg;
  GstStateChangeReturn ret;
  gboolean terminate = FALSE;

  pipeline = gst_pipeline_new ("my-pipeline");

  /*source = gst_element_factory_make ("firesrc", "source");*/
  source = gst_element_factory_make ("videotestsrc", "source");
  filter = gst_element_factory_make ("videoconvert", "convert");
  sink = gst_element_factory_make ("autovideosink", "sink");

  /*g_object_set (source, "num-buffers", 10, NULL);*/
  g_object_set (source, "pattern", 18, NULL);  // pattern=ball
  g_object_set (source, "motion", 1, NULL);
  g_object_set (source, "flip", 1, NULL);

  /* must add elements to pipeline before linking them */
  gst_bin_add_many (GST_BIN (pipeline), source, filter, sink, NULL);

  /* link */
  if (!gst_element_link_many (source, filter, sink, NULL)) {
    g_warning ("Failed to link elements!");
  }

  /* Set custom pipeline clock source */
  /*ros2clock = gst_ros2_clock_new_from_topic ("ROS2Clock", "/clock");*/
  /*g_assert (ros2clock);*/

  /* Set custom pipeline clock source */
  gst_pipeline_use_clock (GST_PIPELINE (pipeline), clock);

  /* Start playing */
  ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr ("Unable to set the pipeline to the playing state.\n");
    gst_object_unref (pipeline);
    return -1;
  }

  /* Listen to the bus */
  bus = gst_element_get_bus (pipeline);
  do {
    msg = gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE,
        GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_ERROR | GST_MESSAGE_EOS);

    /* Parse message */
    if (msg != NULL) {
      GError * err;
      gchar * debug_info;

      switch (GST_MESSAGE_TYPE (msg)) {
        case GST_MESSAGE_ERROR:
          gst_message_parse_error (msg, &err, &debug_info);
          g_printerr ("Error received from element %s: %s\n", GST_OBJECT_NAME (msg->src),
              err->message);
          g_printerr ("Debugging information: %s\n", debug_info ? debug_info : "none");
          g_clear_error (&err);
          g_free (debug_info);
          terminate = TRUE;
          break;
        case GST_MESSAGE_EOS:
          g_print ("End-Of-Stream reached.\n");
          terminate = TRUE;
          break;
        case GST_MESSAGE_STATE_CHANGED:
          /* We are only interested in state-changed messages from the pipeline */
          if (GST_MESSAGE_SRC (msg) == GST_OBJECT (pipeline)) {
            GstState old_state, new_state, pending_state;
            gst_message_parse_state_changed (msg, &old_state, &new_state, &pending_state);
            g_print ("Pipeline state changed from %s to %s:\n",
                gst_element_state_get_name (old_state), gst_element_state_get_name (new_state));
          }
          break;
        default:
          /* We should not reach here */
          g_printerr ("Unexpected message received.\n");
          break;
      }
      gst_message_unref (msg);
    }
  } while (!terminate);

  gst_element_set_state (pipeline, GST_STATE_NULL);

  gst_object_unref (pipeline);
  return 0;
}

