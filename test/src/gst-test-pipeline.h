#pragma once

#include <gst/gstclock.h>

G_BEGIN_DECLS

void print_gst_version ();
int launch_test_pipeline (GstClock * clock);

G_END_DECLS
