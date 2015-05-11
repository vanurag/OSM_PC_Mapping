#include <gflags/gflags.h>

// Bundler Parser
DEFINE_string(bundler_file, "", "File name of the bundler file to open. Provide full path.");

// Pc Handler
DEFINE_bool(show_cloud, true, "Whether or not to visualize the point cloud");
DEFINE_bool(show_cameras, true, "Whether or not to visualize the camera viewpoints");
DEFINE_bool(show_normals, true, "Whether or not to visualize the point normals");
DEFINE_double(segmentation_threshold, 0.1, "Points with dot(normals, ground) < this threshold are segmented out");
DEFINE_double(search_radius, 0.4, "Search radius (in meters) used to estiamte point normals");
DEFINE_string(outline_file_path, "", "file path to store point cloud outline");
