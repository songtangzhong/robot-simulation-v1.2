#ifndef ROBOT_FUN_H_
#define ROBOT_FUN_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robot_info/arm_info.h>
#include <process_commu/arm_state_shm.h>
#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>

namespace robot_fun
{
class RobotFun : public rclcpp::Node
{
public:
    RobotFun(const std::string & node_name);
    ~RobotFun();

    void get_arm_joint_positions(double * positions);
    void get_arm_joint_velocities(double * positions);
    void get_arm_joint_efforts(double * positions);

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_state_sub_;

    void callback_robot_state_sub_(const sensor_msgs::msg::JointState::SharedPtr msg);

    std::shared_ptr<arm_info::ArmInfo> arm_info_ = std::make_shared<arm_info::ArmInfo>();

    arm_state_shm::ArmStateShm *arm_state_shm_;
    int arm_state_shm_id_;
    int arm_state_sem_id_;
};

}

#endif