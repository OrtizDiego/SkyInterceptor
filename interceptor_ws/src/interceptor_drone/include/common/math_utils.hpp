#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace interceptor {

// Skew symmetric matrix
inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
}

// Quaternion to rotation matrix
inline Eigen::Matrix3d quatToRot(const Eigen::Vector4d& q) {
    double w = q(0), x = q(1), y = q(2), z = q(3);
    Eigen::Matrix3d R;
    R << 1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w),
         2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w),
         2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y);
    return R;
}

// Rotation matrix to quaternion
inline Eigen::Vector4d rotToQuat(const Eigen::Matrix3d& R) {
    double trace = R.trace();
    Eigen::Vector4d q;
    
    if (trace > 0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        q(0) = 0.25 / s;
        q(1) = (R(2,1) - R(1,2)) * s;
        q(2) = (R(0,2) - R(2,0)) * s;
        q(3) = (R(1,0) - R(0,1)) * s;
    } else {
        // Handle other cases
        if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
            double s = 2.0 * std::sqrt(1.0 + R(0,0) - R(1,1) - R(2,2));
            q(0) = (R(2,1) - R(1,2)) / s;
            q(1) = 0.25 * s;
            q(2) = (R(0,1) + R(1,0)) / s;
            q(3) = (R(0,2) + R(2,0)) / s;
        } else if (R(1,1) > R(2,2)) {
            double s = 2.0 * std::sqrt(1.0 + R(1,1) - R(0,0) - R(2,2));
            q(0) = (R(0,2) - R(2,0)) / s;
            q(1) = (R(0,1) + R(1,0)) / s;
            q(2) = 0.25 * s;
            q(3) = (R(1,2) + R(2,1)) / s;
        } else {
            double s = 2.0 * std::sqrt(1.0 + R(2,2) - R(0,0) - R(1,1));
            q(0) = (R(1,0) - R(0,1)) / s;
            q(1) = (R(0,2) + R(2,0)) / s;
            q(2) = (R(1,2) + R(2,1)) / s;
            q(3) = 0.25 * s;
        }
    }
    
    return q.normalized();
}

// Saturate vector magnitude
inline Eigen::Vector3d saturateVector(const Eigen::Vector3d& v, double max_norm) {
    double norm = v.norm();
    if (norm > max_norm && norm > 1e-6) {
        return v * (max_norm / norm);
    }
    return v;
}

} // namespace interceptor
