#include "common.h"
#include "bundler-parser.h"
#include "pc-handler.h"

using namespace bundler_parser;
using namespace pc_handler;

int main(int num_arguments, char** arguments) {
  
  PcHandler pc_handle;
  
  // pack r/g/b into rgb
  uint8_t r = 255, g = 0, b = 0; // Example: Red color
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  pc_handle.point.rgb = *reinterpret_cast<float*>(&rgb);

  
  return 0;
}
