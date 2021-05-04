#ifndef END_EFF_INFO_H_
#define END_EFF_INFO_H_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

namespace end_eff_info
{
class EndEffInfo
{
public:
    EndEffInfo();
    ~EndEffInfo();

    const unsigned int end_eff_position_mode_ = (1<<0);
    const unsigned int end_eff_velocity_mode_ = (1<<1);
    const unsigned int end_eff_effort_mode_ = (1<<2);

    unsigned int end_eff_dof_;

    std::vector<std::string> end_eff_joint_names_;

    std::vector<double> cur_end_eff_joint_positions_;
    std::vector<double> cur_end_eff_joint_velocities_;
    std::vector<double> cur_end_eff_joint_efforts_;

    std::vector<double> cmd_end_eff_joint_positions_;
    std::vector<double> cmd_end_eff_joint_velocities_;
    std::vector<double> cmd_end_eff_joint_efforts_;

    std::vector<unsigned int> end_eff_control_modes_;

    key_t end_eff_shm_key_;
    key_t end_eff_sem_key_;
};

}

#endif