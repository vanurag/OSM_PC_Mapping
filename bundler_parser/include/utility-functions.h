#ifndef BUNDLER_PARSER_UTILITY_FUNCTIONS_H_
#define BUNDLER_PARSER_UTILITY_FUNCTIONS_H_

#include "common.h"

// seeks to the line_number-th line in the file opened by file_handler.
// if is_relative=true seeks relative to current file_handler position
// else, seeks from start of the file. line_number starts from 1.
std::fstream& seekToLine(std::fstream& file_handle, unsigned int line_number,
                         bool is_relative);

#endif // BUNDLER_PARSER_UTILITY_FUNCTIONS_H_