project('gst-ros2clock-plugin', 'c', 'cpp',
  default_options : ['c_std=c11', 'cpp_std=c++17'],
  version : '0.1.1',
  license : 'LGPL')



# rcutils, depended on by rclc, adds a dependency on the dynamic linker library
# "dl".  This library is not supported as a built in dependency by meson before
# v0.62. https://mesonbuild.com/Dependencies.html#dl-libdl
# If running a meson version earlier than 0.62, you will thus see
# messages such as 'WARNING: CMake: Dependency dl for rclc was not found'

# Also the following warning is caused by a bug in libfmt's build config:
# WARNING: CMake: Dependency -Wl,--as-needed for rclc was not found
# See: https://github.com/fmtlib/fmt/issues/3023

gst_dep = dependency('gstreamer-1.0')
rcl_dep = dependency('rcl', required : true)
rclc_dep = dependency('rclc', required: get_option('subscription_mode'))

rosgraph_msgs_dep = dependency('rosgraph_msgs',
  modules: ['rosgraph_msgs::rosgraph_msgs__rosidl_generator_c',
    'rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_c'],
  required: false)

gst_version = meson.project_version()
api_version = '1.0'
plugin_c_args = ['-DHAVE_CONFIG_H']

cdata = configuration_data()
cdata.set_quoted('PACKAGE_VERSION', gst_version)
cdata.set_quoted('PACKAGE', 'gst-ros2clock-plugin')
cdata.set_quoted('GST_LICENSE', 'LGPL')
cdata.set_quoted('GST_API_VERSION', api_version)
cdata.set_quoted('GST_PACKAGE_NAME', 'GStreamer ROS2 Clock Source Plugin')
cdata.set_quoted('GST_PACKAGE_ORIGIN', 'https://www.rosella.org')

# Symbol visibility
cc = meson.get_compiler('c')
if cc.get_id() == 'msvc'
  export_define = '__declspec(dllexport) extern'
elif cc.has_argument('-fvisibility=hidden')
  add_project_arguments('-fvisibility=hidden', language: 'c')
  export_define = 'extern __attribute__ ((visibility ("default")))'
else
  export_define = 'extern'
endif
cdata.set('GST_API_EXPORT', export_define)

plugin_sources = ['src/gstros2objclock.c']
plugin_dependencies = [gst_dep, rcl_dep]

if rclc_dep.found() and rosgraph_msgs_dep.found()
  message('Enabling subscription mode')
  plugin_sources += ['src/gstros2subclock.c', 'src/subtimesrc.c']
  plugin_dependencies += [rclc_dep, rosgraph_msgs_dep]
  cdata.set_quoted('HAVE_RCLC', '1')
endif

configure_file(output : 'config.h', configuration : cdata)

plugins_install_dir = join_paths(get_option('libdir'), 'gstreamer-1.0')

libros2clock = library('gstros2clock',
  plugin_sources,
  c_args: plugin_c_args,
  dependencies : plugin_dependencies,
  install : true,
  install_dir : plugins_install_dir,
  )

subdir('test')
