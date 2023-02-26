#pragma once

#include <chrono>
#include <ratio>
#include <Eigen/Core>
#include <ros/node_handle.h>



namespace panda_torque_mpc {

    // Eigen Typedef
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    using Matrix7d = Eigen::Matrix<double, 7, 7>;


    Vector7d saturateTorqueRate(const Vector7d &tau_d, const Vector7d &tau_d_prev, double delta_max);


    struct TicTac
    {
        /**
         * Tutorial here: https://akrzemi1.wordpress.com/2022/04/11/using-stdchrono/
         * 
         * "steady_clock is intented for handling time interval measurements"
        */

        using clock = std::chrono::steady_clock;
        using nanoseconds = std::chrono::duration<int, std::nano>;

        // member variables
        std::chrono::time_point<clock, nanoseconds> tstart;
        std::chrono::time_point<clock, nanoseconds> tend;
        double duration_ms;

        TicTac(): 
            tstart(clock::now()),
            tend(clock::now()),
            duration_ms(-1.0) 
        {}

        void tic()
        {
            tstart = clock::now();
        }

        double tac()
        {
            tend = clock::now();
            duration_ms = (tend - tstart).count()/1e6;
            return duration_ms;
        }

        void print_tac(const std::string& msg)
        {
            std::cout << std::setprecision(9) << msg << tac() << std::endl;
        }

    };



    template <typename T>
    bool get_param_error_tpl(const ros::NodeHandle &nh, 
                             T &param, 
                             const std::string &param_name, 
                             const std::function< bool(T) > &check = [](T v) {return true;})
    {
        if (!( nh.getParam(param_name, param) && check(param) ))
        {
            ROS_ERROR_STREAM("Error reading/checking parameter " << param_name << ", " << param);
            return false;
        }
        else{
            return true;
        }
    }



    template<typename T>
    std::ostream &operator <<(std::ostream &os, const std::vector<T> &v) {
        using namespace std;
        copy(v.begin(), v.end(), ostream_iterator<T>(os, "\n"));
        return os;
    }

} // namespace panda_torque_mpc
