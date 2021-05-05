#include <chrono>
#include <memory>
#include <string>
#include <controller_manager_msgs/srv/load_start_controller.hpp>
#include <controller_manager_msgs/srv/load_configure_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_hw_interface/controller_configure.h>

using namespace std::chrono_literals;

namespace controller_configure
{
ControllerConfigure::ControllerConfigure(const std::string & node_name)
{
    nh_ = std::make_shared<rclcpp::Node>(node_name);

    load_start_controller_cli_ = 
    nh_->create_client<controller_manager_msgs::srv::LoadStartController>("/controller_manager/load_and_start_controller");

    load_configure_controller_cli_ = 
    nh_->create_client<controller_manager_msgs::srv::LoadConfigureController>("/controller_manager/load_and_configure_controller");

    switch_controller_cli_ = 
    nh_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    arm_shm_id_ = shm_common::create_shm(arm_info_->arm_shm_key_, &arm_shm_);
    if (arm_shm_id_ != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("ControllerConfigure"), "Create arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("ControllerConfigure"), "Create arm shared memory failed.");
    }

    arm_sem_id_ = sem_common::create_semaphore(arm_info_->arm_sem_key_);
    if (arm_sem_id_ != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("ControllerConfigure"), "Create arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("ControllerConfigure"), "Create arm semaphore failed.");
    }
}

ControllerConfigure::~ControllerConfigure(){}

void ControllerConfigure::load_start_controller(const std::string & controller_name)
{
    auto request = std::make_shared<controller_manager_msgs::srv::LoadStartController::Request>();
    request->name = controller_name;
    while (!load_start_controller_cli_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("load_start_controller"), "Interrupted while waiting for the service. Exiting.");
        }

        RCLCPP_INFO(rclcpp::get_logger("load_start_controller"), "service [/controller_manager/load_and_start_controller] not available, waiting again...");
    }

    auto result = load_start_controller_cli_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("load_start_controller"), "load and start %s successfully.", controller_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("load_start_controller"), "Failed to load and start %s.", controller_name.c_str());
    }
}

void ControllerConfigure::load_configure_controller(const std::string & controller_name)
{
    auto request = std::make_shared<controller_manager_msgs::srv::LoadConfigureController::Request>();
    request->name = controller_name;
    while (!load_configure_controller_cli_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("load_configure_controller"), "Interrupted while waiting for the service. Exiting.");
        }

        RCLCPP_INFO(rclcpp::get_logger("load_configure_controller"), "service [/controller_manager/load_and_configure_controller] not available, waiting again...");
    }

    auto result = load_configure_controller_cli_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("load_configure_controller"), "load and configure %s successfully.", controller_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("load_configure_controller"), "Failed to load and start %s.", controller_name.c_str());
    }
}

void ControllerConfigure::switch_controller(const std::string & start_controller, const std::string & stop_controller)
{
    std::vector<std::string> start_controller_ = {start_controller};
    std::vector<std::string> stop_controller_ = {stop_controller};
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->start_controllers = start_controller_;
    request->stop_controllers = stop_controller_;
    request->strictness = request->BEST_EFFORT;
    request->start_asap = false;
    request->timeout = rclcpp::Duration(static_cast<rcl_duration_value_t>(0.0));
    while (!switch_controller_cli_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("switch_controller"), "Interrupted while waiting for the service. Exiting.");
        }

        RCLCPP_INFO(rclcpp::get_logger("switch_controller"), "service [/controller_manager/switch_controller] not available, waiting again...");
    }

    auto result = switch_controller_cli_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        sem_common::semaphore_p(arm_sem_id_);
        for (unsigned int j=0; j< arm_info_->arm_dof_; j++)
        {
            if (start_controller == "position_controllers")
            {
                // Set current arm joint positions to commands by ros2 controller manager,
                // not by shared memory.

                arm_shm_->arm_control_modes_[j] = arm_info_->arm_position_mode_;
            }
            else if (start_controller == "velocity_controllers")
            {
                // Set current arm joint velocities (zeros) to commands by ros2 controller manager,
                // not by shared memory.

                arm_shm_->arm_control_modes_[j] = arm_info_->arm_velocity_mode_;
            }
            else if (start_controller == "effort_controllers")
            {
                // Set current arm joint efforts (zeros) to commands by ros2 controller manager,
                // not by shared memory.

                arm_shm_->arm_control_modes_[j] = arm_info_->arm_effort_mode_;
            }
        }
        sem_common::semaphore_v(arm_sem_id_);

        RCLCPP_INFO(rclcpp::get_logger("switch_controller"), "switch %s to %s successfully.", stop_controller.c_str(), start_controller.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("switch_controller"), "Failed to switch %s to %s.", stop_controller.c_str(), start_controller.c_str());
    }
}

}
