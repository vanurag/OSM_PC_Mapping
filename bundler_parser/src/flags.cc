#include <gflags/gflags.h>

// Bundler Parser
DEFINE_string(file, "", "File name of the bundler file to open. Provide full path.");
DEFINE_string(query, "camera", "task: camera query or 3D point query");
DEFINE_string(indices, "1 2", "query indices space seperated. camera or 3D point depending on the query");
