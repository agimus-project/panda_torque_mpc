


#include "panda_torque_mpc/common.h"

#include <iostream>


namespace panda_torque_mpc {


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
                std::cout << "  i, tau_d[i]: " << i << ", " << tau_d[i] << std::endl;
                std::cout << "tau_d_prev[i]   : " << tau_d_prev[i] << std::endl;
                std::cout << "delta_tau_i     : " << delta_tau_i << std::endl;
                std::cout << "delta_tau_i_sat : " << delta_tau_i_sat << std::endl;
                std::cout << "tau_d_sat[i]    : " << tau_d_sat[i] << std::endl;
            }
        }
        return tau_d_sat;
    }




} // namespace panda_torque_mpc
