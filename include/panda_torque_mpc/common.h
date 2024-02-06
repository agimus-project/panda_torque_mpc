#pragma once

#include <vector>
#include <chrono>
#include <ratio>
#include <Eigen/Core>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>


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



    inline pinocchio::Model loadPandaPinocchio()
    {
        // Load panda model with pinocchio and example-robot-data
        // std::string urdf_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/panda_description/urdf/panda.urdf";
        // std::string srdf_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/panda_description/srdf/panda.srdf";
        std::string urdf_path = "/home/gepetto/ros_ws/src/panda_torque_mpc/urdf/robot.urdf";
        std::string srdf_path =  "/home/gepetto/ros_ws/src/panda_torque_mpc/srdf/demo.srdf";
        pinocchio::Model model_pin_full;
        pinocchio::urdf::buildModel(urdf_path, model_pin_full);
        pinocchio::srdf::loadReferenceConfigurations(model_pin_full, srdf_path, false);
        // pinocchio::srdf::loadRotorParameters(model_pin_full, srdf_path, false);
        Eigen::VectorXd q0_full = model_pin_full.referenceConfigurations["default"];
        std::vector<unsigned long> locked_joints_id {model_pin_full.getJointId("panda_finger_joint1"), 
                                                     model_pin_full.getJointId("panda_finger_joint2"),
                                                     };        
        return pinocchio::buildReducedModel(model_pin_full, locked_joints_id, q0_full);
    }

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

        /**
        Return duration since object creation (or last .tic() call) in milliseconds
        */
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
            std::cout << "Error reading/checking parameter " << param_name << ", " << param << std::endl;
            ROS_ERROR_STREAM("Error reading/checking parameter " << param_name << ", " << param);
            return false;
        }
        else
        {
            return true;
        }
    }


    /**
     * \brief Generate a (cos)sinusoidal target trajectory of end effector pose.
     *
     * @param[in] delta_nu trajectory parameter: vector (size 6, [lin,rot]) of the delta motion amplitude (on the SE3 local tangent space)
     * @param[in] period_nu trajectory parameter: vector (size 6, [lin,rot]) of period for delta motion axes (on the SE3 local tangent space)
     * @param[in] pose_0 initial pose
     * @param[in] t current time with q(0) = q0
     * @param[out] x_r target joint configuration
     * @param[out] dx_r target joint velocity
     * @param[out] ddx_r target joint acceleration
     */
    inline void compute_sinusoid_pose_reference(const Vector6d &delta_nu, const Vector6d &period_nu, const pinocchio::SE3 &pose_0, double t,
                                                pinocchio::SE3 &x_r, pinocchio::Motion &dx_r, pinocchio::Motion &ddx_r)
    {
        // Ai and Ci obtained for each joint using constraints:
        // T(t=0.0) = pose_0
        // T(t=period/2) = pose_0 * Exp(delta_nu)

        Vector6d w = (2 * M_PI / period_nu.array()).matrix();
        Vector6d a = -delta_nu;
        Vector6d c = delta_nu;

        Vector6d nu = (a.array() * cos(w.array() * t)).matrix() + c;
        dx_r = pinocchio::Motion((-w.array() * a.array() * sin(w.array() * t)).matrix());
        ddx_r = pinocchio::Motion((-w.array().square() * a.array() * cos(w.array() * t)).matrix()); // non null initial acceleration!! needs to be dampened (e.g. torque staturation)

        x_r = pose_0 * pinocchio::exp6(nu);
    }


    inline pinocchio::SE3 XYZQUATToSE3(const std::vector<double>& pose)
    {
        // pose: [px, py, pz,    qx, qy, qz, qw]
        Eigen::Vector3d t(pose[0], pose[1], pose[2]);
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]); // this constructor order is different : qw, qx, qy, qz 

        q.normalize();
        
        return pinocchio::SE3(q.matrix(), t);
    }


    inline pinocchio::SE3 posemsg2SE3(const geometry_msgs::Pose& pose)
    {
        std::vector<double> xyzquat = {pose.position.x, pose.position.y, pose.position.z, 
                                       pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
        return XYZQUATToSE3(xyzquat);
    }


    inline geometry_msgs::Pose SE32posemsg(const pinocchio::SE3& T)
    {
        geometry_msgs::Pose pose;
        Eigen::Quaterniond q(T.rotation()); 

        pose.position.x = T.translation()[0];
        pose.position.y = T.translation()[1];
        pose.position.z = T.translation()[2];
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    }


    inline void publish_SE3_posestamped(const ros::Publisher& publisher, const pinocchio::SE3& pose, const std_msgs::Header& header)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose = SE32posemsg(pose);
        pose_msg.header = header;
        publisher.publish(pose_msg);
    }

} // namespace panda_torque_mpc
