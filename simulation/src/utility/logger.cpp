#include "../../include/utility/logger.h"
#include <memory>
namespace SimCore::log{
 #ifdef DRONE_LOGGING
std::unique_ptr<dataLogger> logger = std::make_unique<dataLogger>();
#endif
}   