#ifndef OSM_PC_MAPPING_PC_HANDLER_H_
#define OSM_PC_MAPPING_PC_HANDLER_H_

#include "common.h"

namespace pc_handler {

class PcHandler {
 public:
  PcHandler() {};
  virtual ~PcHandler() {};

  pcl::PointXYZRGB point;
};

} // namespace pc_handler

#endif // OSM_PC_MAPPING_PC_HANDLER_H_
