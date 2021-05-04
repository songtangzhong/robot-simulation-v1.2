#include <rclcpp/rclcpp.hpp>
#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>
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
        RCLCPP_INFO(rclcpp::get_logger("test_arm_control"), "Create arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_arm_control"), "Create arm shared memory failed.");
        return 0;
    }

    int arm_sem_id;
    arm_sem_id = sem_common::create_semaphore(arm_info->arm_sem_key_);
    if (arm_sem_id != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_arm_control"), "Create arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_arm_control"), "Create arm semaphore failed.");
        return 0;
    }

    rclcpp::WallRate loop_rate(1000);
    unsigned int i = 0;
    while (rclcpp::ok())
    {
        sem_common::semaphore_p(arm_sem_id);
        for (unsigned int j=0; j< arm_info->arm_dof_; j++)
        {
            arm_shm->arm_control_modes_[j] = arm_info->arm_position_mode_;

            arm_shm->cmd_arm_joint_positions_[j] = arm_shm->cur_arm_joint_positions_[j]+0.001;
            arm_shm->cmd_arm_joint_velocities_[j] = 0;
            arm_shm->cmd_arm_joint_efforts_[j] = 0;
        }
        sem_common::semaphore_v(arm_sem_id);
        loop_rate.sleep();
        if (i++ == 1000)
        {
            RCLCPP_INFO(rclcpp::get_logger("test_arm_control"), "Execute arm control successfully.");
            break;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
#if END_EFF_TRUE
    std::shared_ptr<end_eff_info::EndEffInfo> end_eff_info = std::make_shared<end_eff_info::EndEffInfo>();
    end_eff_shm::EndEffShm *end_eff_shm;
    int end_eff_shm_id;

    end_eff_shm_id = shm_common::create_shm(end_eff_info->end_eff_shm_key_, &end_eff_shm);
    if (end_eff_shm_id != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_end_eff_control"), "Create end-effector shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_end_eff_control"), "Create end-effector shared memory failed.");
        return 0;
    }

    int end_eff_sem_id;
    end_eff_sem_id = sem_common::create_semaphore(end_eff_info->end_eff_sem_key_);
    if (end_eff_sem_id != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_end_eff_control"), "Create end-effector semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_end_eff_control"), "Create end-effector semaphore failed.");
        return 0;
    }

    i = 0;
    while (rclcpp::ok())
    {
        sem_common::semaphore_p(end_eff_sem_id);
        for (unsigned int j=0; j< end_eff_info->end_eff_dof_; j++)
        {
            end_eff_shm->end_eff_control_modes_[j] = end_eff_info->end_eff_position_mode_;

            end_eff_shm->cmd_end_eff_joint_positions_[j] = end_eff_shm->cur_end_eff_joint_positions_[j]+0.001;
            end_eff_shm->cmd_end_eff_joint_velocities_[j] = 0;
            end_eff_shm->cmd_end_eff_joint_efforts_[j] = 0;
        }
        sem_common::semaphore_v(end_eff_sem_id);
        loop_rate.sleep();
        if (i++ == 1000)
        {
            RCLCPP_INFO(rclcpp::get_logger("test_end_eff_control"), "Execute end-effector control successfully.");
            break;
        }
    }
#endif

    rclcpp::shutdown();
    return 0;
}
