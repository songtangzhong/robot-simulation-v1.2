#include <rclcpp/rclcpp.hpp>
#include <process_commu/shm_common.h>
#include <process_commu/arm_shm.h>
#include <robot_info/arm_info.h>
#include <robot_info/robot_basic_macro.h>
#include <iostream>

#if END_EFF_TRUE
#include <process_commu/end_eff_shm.h>
#include <robot_info/end_eff_info.h>
#endif

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<arm_info::ArmInfo> arm_info = std::make_shared<arm_info::ArmInfo>();
    arm_shm::ArmShm *arm_shm;
    int arm_shm_id;

    arm_shm_id = shm_common::create_shm(arm_info->arm_shm_key_, &arm_shm);
    if (arm_shm_id != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_arm_shm"), "Create arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_arm_shm"), "Create arm shared memory failed.");
        return 0;
    }

    for (unsigned int j=0; j< arm_info->arm_dof_; j++)
    {
        arm_shm->cur_arm_joint_positions_[j] = 1;
        arm_shm->cur_arm_joint_velocities_[j] = 1;
        arm_shm->cur_arm_joint_efforts_[j] = 1;

        arm_shm->cmd_arm_joint_positions_[j] = 1;
        arm_shm->cmd_arm_joint_velocities_[j] = 1;
        arm_shm->cmd_arm_joint_efforts_[j] = 1;

        arm_shm->arm_control_modes_[j] = 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    for (unsigned int j=0; j< arm_info->arm_dof_; j++)
    {
        arm_info->cur_arm_joint_positions_[j] = arm_shm->cur_arm_joint_positions_[j];
        arm_info->cur_arm_joint_velocities_[j] = arm_shm->cur_arm_joint_velocities_[j];
        arm_info->cur_arm_joint_efforts_[j] = arm_shm->cur_arm_joint_efforts_[j];

        arm_info->cmd_arm_joint_positions_[j] = arm_shm->cmd_arm_joint_positions_[j];
        arm_info->cmd_arm_joint_velocities_[j] = arm_shm->cmd_arm_joint_velocities_[j];
        arm_info->cmd_arm_joint_efforts_[j] = arm_shm->cmd_arm_joint_efforts_[j];

        arm_info->arm_control_modes_[j] = arm_shm->arm_control_modes_[j];

        std::cout << "arm_info->cur_arm_joint_positions_[" << j << "]: " << arm_info->cur_arm_joint_positions_[j] << std::endl;
        std::cout << "arm_info->cur_arm_joint_velocities_[" << j << "]: " << arm_info->cur_arm_joint_velocities_[j] << std::endl;
        std::cout << "arm_info->cur_arm_joint_efforts_[" << j << "]: " << arm_info->cur_arm_joint_efforts_[j] << std::endl;
    
        std::cout << "arm_info->cmd_arm_joint_positions_[" << j << "]: " << arm_info->cmd_arm_joint_positions_[j] << std::endl;
        std::cout << "arm_info->cmd_arm_joint_velocities_[" << j << "]: " << arm_info->cmd_arm_joint_velocities_[j] << std::endl;
        std::cout << "arm_info->cmd_arm_joint_efforts_[" << j << "]: " << arm_info->cmd_arm_joint_efforts_[j] << std::endl;
    
        std::cout << "arm_info->arm_control_modes_[" << j << "]: " << arm_info->arm_control_modes_[j] << std::endl;
    }

    if (shm_common::release_shm(arm_shm_id, &arm_shm) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_arm_shm"), "Release arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_arm_shm"), "Release arm shared memory failed.");
        return 0;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
#if END_EFF_TRUE
    std::shared_ptr<end_eff_info::EndEffInfo> end_eff_info = std::make_shared<end_eff_info::EndEffInfo>();
    end_eff_shm::EndEffShm *end_eff_shm;
    int end_eff_shm_id;

    end_eff_shm_id = shm_common::create_shm(end_eff_info->end_eff_shm_key_, &end_eff_shm);
    if (end_eff_shm_id != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_end_eff_shm"), "Create end-effector shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_end_eff_shm"), "Create end-effector shared memory failed.");
        return 0;
    }

    for (unsigned int j=0; j< end_eff_info->end_eff_dof_; j++)
    {
        end_eff_shm->cur_end_eff_joint_positions_[j] = 1;
        end_eff_shm->cur_end_eff_joint_velocities_[j] = 1;
        end_eff_shm->cur_end_eff_joint_efforts_[j] = 1;

        end_eff_shm->cmd_end_eff_joint_positions_[j] = 1;
        end_eff_shm->cmd_end_eff_joint_velocities_[j] = 1;
        end_eff_shm->cmd_end_eff_joint_efforts_[j] = 1;

        end_eff_shm->end_eff_control_modes_[j] = 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    for (unsigned int j=0; j< end_eff_info->end_eff_dof_; j++)
    {
        end_eff_info->cur_end_eff_joint_positions_[j] = end_eff_shm->cur_end_eff_joint_positions_[j];
        end_eff_info->cur_end_eff_joint_velocities_[j] = end_eff_shm->cur_end_eff_joint_velocities_[j];
        end_eff_info->cur_end_eff_joint_efforts_[j] = end_eff_shm->cur_end_eff_joint_efforts_[j];

        end_eff_info->cmd_end_eff_joint_positions_[j] = end_eff_shm->cmd_end_eff_joint_positions_[j];
        end_eff_info->cmd_end_eff_joint_velocities_[j] = end_eff_shm->cmd_end_eff_joint_velocities_[j];
        end_eff_info->cmd_end_eff_joint_efforts_[j] = end_eff_shm->cmd_end_eff_joint_efforts_[j];

        end_eff_info->end_eff_control_modes_[j] = end_eff_shm->end_eff_control_modes_[j];

        std::cout << "end_eff_info->cur_end_eff_joint_positions_[" << j << "]: " << end_eff_info->cur_end_eff_joint_positions_[j] << std::endl;
        std::cout << "end_eff_info->cur_end_eff_joint_velocities_[" << j << "]: " << end_eff_info->cur_end_eff_joint_velocities_[j] << std::endl;
        std::cout << "end_eff_info->cur_end_eff_joint_efforts_[" << j << "]: " << end_eff_info->cur_end_eff_joint_efforts_[j] << std::endl;
    
        std::cout << "end_eff_info->cmd_end_eff_joint_positions_[" << j << "]: " << end_eff_info->cmd_end_eff_joint_positions_[j] << std::endl;
        std::cout << "end_eff_info->cmd_end_eff_joint_velocities_[" << j << "]: " << end_eff_info->cmd_end_eff_joint_velocities_[j] << std::endl;
        std::cout << "end_eff_info->cmd_end_eff_joint_efforts_[" << j << "]: " << end_eff_info->cmd_end_eff_joint_efforts_[j] << std::endl;
    
        std::cout << "end_eff_info->end_eff_control_modes_[" << j << "]: " << end_eff_info->end_eff_control_modes_[j] << std::endl;
    }

    if (shm_common::release_shm(end_eff_shm_id, &end_eff_shm) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_end_eff_shm"), "Release end-effector shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_end_eff_shm"), "Release end-effector shared memory failed.");
        return 0;
    }
#endif

    rclcpp::shutdown();
    return 0;
}
