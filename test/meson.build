
ros2clock_dep = declare_dependency(
  link_with: libros2clock,
  compile_args: '-DGST_API_EXPORT=extern')

pipe_lib = static_library('gst-test-pipeline',
  ['src/gst-test-pipeline.c'],
  dependencies : [gst_dep])

pipe_dep = declare_dependency(link_with: pipe_lib)

if rclc_dep.found()
  executable('test-ros2-clock', 
    'src/test-ros-clock.c',
    include_directories : '../src/',
    dependencies : [pipe_dep, gst_dep, rclc_dep, ros2clock_dep])
endif

rclcpp_dep = dependency('rclcpp', required : false)

if rclcpp_dep.found() and rosgraph_msgs_dep.found()

  stdmsgs_dep = dependency('std_msgs', 
    modules: ['std_msgs::std_msgs__rosidl_generator_cpp', 
              'std_msgs::std_msgs__rosidl_typesupport_cpp'], 
    required: true)

  executable('node-clock-source',
    ['src/node-clock-source.cpp', 'src/gst-test-pipeline.c'],
    include_directories : '../src/',
    cpp_args: '-Wno-non-virtual-dtor',
    dependencies : [pipe_dep, gst_dep, ros2clock_dep, 
                    rosgraph_msgs_dep, stdmsgs_dep, rclcpp_dep])

endif
