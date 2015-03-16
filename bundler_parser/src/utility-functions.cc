#include "utility-functions.h"

// seeks to the line_number-th line in the file opened by file_handler.
// if is_relative=true seeks relative to current file_handler position
// else, seeks from start of the file. line_number starts from 1.
void seekToLine(std::fstream& file_handle, unsigned int line_number,
                bool is_relative){
    
    CHECK_GT(line_number, 0) << "line_number has to be >= 1";
    
    if (!is_relative) {
      file_handle.seekg(std::ios::beg);
    }
    for(unsigned int i = 0; i < line_number - 1; ++i){
        file_handle.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}
