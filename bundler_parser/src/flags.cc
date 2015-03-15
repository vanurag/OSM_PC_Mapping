#include <gflags/gflags.h>

// Bundler Parser
DEFINE_string(file, "", "File name of the bundler file to open. Provide full path.");
DEFINE_string(task, "camera", "task: camera query or 3D point query");
DEFINE_int32(index, -1, "query index. camera or 3D point depending on the task");
