#ifndef ARM_INFO_H_
#define ARM_INFO_H_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

namespace arm_info
{
class ArmInfo
{
public:
    ArmInfo();
    ~ArmInfo();

    const unsigned int arm_position_mode_ = (1<<0);
    const unsigned int arm_velocity_mode_ = (1<<1);
    const unsigned int arm_effort_mode_ = (1<<2);

    unsigned int arm_dof_;

    std::vector<std::string> arm_joint_names_;

    std::vector<double> cur_arm_joint_positions_;
    std::vector<double> cur_arm_joint_velocities_;
    std::vector<double> cur_arm_joint_efforts_;

    std::vector<double> cmd_arm_joint_positions_;
    std::vector<double> cmd_arm_joint_velocities_;
    std::vector<double> cmd_arm_joint_efforts_;

    std::vector<unsigned int> arm_control_modes_;

    key_t arm_shm_key_;
    key_t arm_sem_key_;

    key_t arm_state_shm_key_;
    key_t arm_state_sem_key_;
};

}

#endif