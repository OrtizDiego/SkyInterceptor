#include "common/math_utils.hpp"
#include <sstream>
#include <iomanip>

namespace interceptor {

std::string vec3dToString(const Eigen::Vector3d& v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
    return oss.str();
}

} // namespace interceptor
