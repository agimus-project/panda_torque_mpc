#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Core>


namespace panda_torque_mpc {

    // Eigen Typedef
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    using Matrix7d = Eigen::Matrix<double, 7, 7>;
}


#endif /* !COMMON_H */
