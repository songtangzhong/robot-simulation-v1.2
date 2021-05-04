#include <rclcpp/rclcpp.hpp>
#include <process_commu/sem_common.h>
#include <robot_info/arm_info.h>
#include <robot_info/robot_basic_macro.h>

#if END_EFF_TRUE
#include <robot_info/end_eff_info.h>
#endif

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<arm_info::ArmInfo> arm_info = std::make_shared<arm_info::ArmInfo>();
    int arm_sem_id;

    arm_sem_id = sem_common::create_semaphore(arm_info->arm_sem_key_);
    if (arm_sem_id != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_arm_sem"), "Create arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_arm_sem"), "Create arm semaphore failed.");
        return 0;
    }

    if (sem_common::semaphore_p(arm_sem_id) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_arm_sem"), "Get arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_arm_sem"), "Get arm semaphore failed.");
        return 0;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    if (sem_common::semaphore_v(arm_sem_id) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_arm_sem"), "Release arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_arm_sem"), "Release arm semaphore failed.");
        return 0;
    }

    if (sem_common::delete_semaphore(arm_sem_id) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_arm_sem"), "Delete arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_arm_sem"), "Delete arm semaphore failed.");
        return 0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////
#if END_EFF_TRUE
    std::shared_ptr<end_eff_info::EndEffInfo> end_eff_info = std::make_shared<end_eff_info::EndEffInfo>();
    int end_eff_sem_id;

    end_eff_sem_id = sem_common::create_semaphore(end_eff_info->end_eff_sem_key_);
    if (end_eff_sem_id != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_end_eff_sem"), "Create end-effector semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_end_eff_sem"), "Create end-effector semaphore failed.");
        return 0;
    }

    if (sem_common::semaphore_p(end_eff_sem_id) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_end_eff_sem"), "Get end-effector semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_end_eff_sem"), "Get end-effector semaphore failed.");
        return 0;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    if (sem_common::semaphore_v(end_eff_sem_id) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_end_eff_sem"), "Release end-effector semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_end_eff_sem"), "Release ebd-effector semaphore failed.");
        return 0;
    }

    if (sem_common::delete_semaphore(end_eff_sem_id) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("test_end_eff_sem"), "Delete end-effector semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_end_eff_sem"), "Delete end-effector semaphore failed.");
        return 0;
    }
#endif

    rclcpp::shutdown();
    return 0;
}
