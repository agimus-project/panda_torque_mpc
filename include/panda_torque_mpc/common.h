#pragma once

#include <chrono>
#include <ratio>
#include <Eigen/Core>
#include <ros/node_handle.h>



namespace panda_torque_mpc {

    // Overloads << operator for std::vector
    template<typename T>
    std::ostream &operator <<(std::ostream &os, const std::vector<T> &v) {
        using namespace std;
        copy(v.begin(), v.end(), ostream_iterator<T>(os, "\n"));
        return os;
    }

    // Eigen Typedef
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    using Matrix7d = Eigen::Matrix<double, 7, 7>;


    inline Vector7d saturateTorqueRate(const Vector7d &tau_d, const Vector7d &tau_d_prev, double delta_max)
    {

        // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is 1000 * (1 / sampling_time).

        Vector7d tau_d_sat{};
        for (size_t i = 0; i < 7; i++)
        {
            double delta_tau_i = tau_d[i] - tau_d_prev[i];
            double delta_tau_i_sat = std::max(std::min(delta_tau_i, delta_max), -delta_max);
            tau_d_sat[i] = tau_d_prev[i] + delta_tau_i_sat;

            if (abs(tau_d_sat[i] - tau_d[i]) > 1e-3)
            {
                // if (abs(delta_tau_i_sat - delta_tau_i) > 1e-3){
                // std::cout << "  i, tau_d[i]: " << i << ", " << tau_d[i] << std::endl;
                // std::cout << "tau_d_prev[i]   : " << tau_d_prev[i] << std::endl;
                // std::cout << "delta_tau_i     : " << delta_tau_i << std::endl;
                // std::cout << "delta_tau_i_sat : " << delta_tau_i_sat << std::endl;
                // std::cout << "tau_d_sat[i]    : " << tau_d_sat[i] << std::endl;
            }
        }
        return tau_d_sat;
    }

    struct TicTac
    {
        /**
         * Tutorial here: https://akrzemi1.wordpress.com/2022/04/11/using-stdchrono/
         * 
         * "steady_clock is intented for handling time interval measurements"
        */

        using clock = std::chrono::steady_clock;
        // Duration represented in nanoseconds as 64 bit unsigned int 
        //   -> would take ~600 years before integer overflow
        using nanoseconds = std::chrono::duration<uint64_t, std::nano>;

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


    /**
     * Does not work with certains types:
     * T in {size_t}
     * 
    */
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
        else
        {
            return true;
        }
    }



} // namespace panda_torque_mpc
