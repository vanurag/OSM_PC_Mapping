#include <gflags/gflags.h>

// Bundler Parser
DEFINE_string(bundler_file, "", "File name of the bundler file to open. Provide full path.");

// Pc Handler
DEFINE_bool(show_cloud, true, "Whether or not to visualize the point cloud");
DEFINE_bool(show_cameras, true, "Whether or not to visualize the camera viewpoints");
