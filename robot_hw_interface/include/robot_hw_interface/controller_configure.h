#ifndef CONTROLLER_CONFIGURE_H_
#define CONTROLLER_CONFIGURE_H_

#include <chrono>
#include <memory>
#include <string>
#include <controller_manager_msgs/srv/load_start_controller.hpp>
#include <controller_manager_msgs/srv/load_configure_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_info/arm_info.h>
#include <process_commu/arm_shm.h>
#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>

namespace controller_configure
{
class ControllerConfigure
{
public:
    ControllerConfigure(const std::string & node_name);
    ~ControllerConfigure();

    void load_start_controller(const std::string & controller_name);

    void load_configure_controller(const std::string & controller_name);

    void switch_controller(const std::string & start_controller, const std::string & stop_controller);

private:
    std::shared_ptr<rclcpp::Node> nh_;

    rclcpp::Client<controller_manager_msgs::srv::LoadStartController>::SharedPtr load_start_controller_cli_;
    rclcpp::Client<controller_manager_msgs::srv::LoadConfigureController>::SharedPtr load_configure_controller_cli_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_cli_;

    std::shared_ptr<arm_info::ArmInfo> arm_info_ = std::make_shared<arm_info::ArmInfo>();

    arm_shm::ArmShm *arm_shm_;
    int arm_shm_id_;
    int arm_sem_id_;
};

}

#endif
