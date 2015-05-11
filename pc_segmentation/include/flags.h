#ifndef PC_HANDLER_FLAGS_H_
#define PC_HANDLER_FLAGS_H_

#include <gflags/gflags.h>

// Bundler Parser
DECLARE_string(bundler_file);

// Pc Handler
DECLARE_bool(show_cloud);
DECLARE_bool(show_cameras);
DECLARE_bool(show_normals);
DECLARE_double(segmentation_threshold);
DECLARE_double(search_radius);
DECLARE_string(outline_file_path);

#endif // PC_HANDLER_FLAGS_H_
