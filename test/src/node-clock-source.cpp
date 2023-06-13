#include <gst/gst.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "gst-test-pipeline.h"
#include "gstros2objclock.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ClockBridge : public rclcpp::Node
{
 public:
  ClockBridge () : Node ("clock_bridge"), count_ (0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String> ("hello", 10);
    timer_ = rclcpp::create_timer (this, this->get_clock (), 500ms,
        std::bind (&ClockBridge::timer_callback, this));
  }

 private:
  void
  timer_callback ()
  {
    auto message = std_msgs::msg::String ();
    message.data = "Hello, world! " + std::to_string (count_++);
    RCLCPP_INFO (this->get_logger (), "Publishing: '%s', Clock=%zd", message.data.c_str (),
        this->get_clock ()->now ().nanoseconds ());

    publisher_->publish (message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

// static rcl_ret_t
// test_get_now (void * data, rcl_time_point_value_t * now)
//{
// static rcl_time_point_value_t ticks = 0;

//// printf ("TICK: %zd\n", ticks);
//*now = ticks;
// ticks += 5000000;
// return RCL_RET_OK;
//}

int
main (int argc, char * argv[])
{
  gst_init (&argc, &argv);
  print_gst_version ();
  rclcpp::init (argc, argv);

  auto node = std::make_shared<ClockBridge> ();
  //  rcl_clock.get_now = test_get_now;

  RCLCPP_INFO (node->get_logger (), "Start time: %zd",
      node->get_clock ()->now ().nanoseconds ());

  // Create GstClock reading time from rcl_clock_t object
  rcl_clock_t * rcl_clock = node->get_clock ()->get_clock_handle ();
  GstClock * gst_ros2_clock = gst_ros2_obj_clock_new ("ROS2Clock", rcl_clock);
  g_assert (gst_ros2_clock);

  // run gstreamer pipeline in separate thread
  auto t = std::thread (launch_test_pipeline, gst_ros2_clock);

  // TODO: proper SIGINT/SIGTERM handling
  rclcpp::spin (node);
  rclcpp::shutdown ();

  gst_deinit ();
  return 0;
}
