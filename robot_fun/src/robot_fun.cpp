#include <robot_fun/robot_fun.h>
#include <memory>

using std::placeholders::_1;

namespace robot_fun
{
RobotFun::RobotFun(const std::string & node_name)
: rclcpp::Node(node_name)
{
    robot_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 100, std::bind(&RobotFun::callback_robot_state_sub_, this, _1));

    arm_state_shm_id_ = shm_common::create_shm(arm_info_->arm_state_shm_key_, &arm_state_shm_);
    if (arm_state_shm_id_ != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotFun"), "Create arm state shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotFun"), "Create arm state shared memory failed.");
    }

    arm_state_sem_id_ = sem_common::create_semaphore(arm_info_->arm_state_sem_key_);
    if (arm_state_sem_id_ != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotFun"), "Create arm state semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotFun"), "Create arm state semaphore failed.");
    }
}

RobotFun::~RobotFun(){}

void RobotFun::callback_robot_state_sub_(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    sem_common::semaphore_p(arm_state_sem_id_);
    for (unsigned int j=0; j< msg->name.size(); j++)
    {
        for (unsigned int i=0; i< arm_info_->arm_dof_; i++)
        {
            if (msg->name[j] == arm_info_->arm_joint_names_[i])
            {
                arm_state_shm_->cur_arm_joint_positions_[i] = msg->position[j];
                arm_state_shm_->cur_arm_joint_velocities_[i] = msg->velocity[j];
                arm_state_shm_->cur_arm_joint_efforts_[i] = msg->effort[j];
            }
        }
    }
    sem_common::semaphore_v(arm_state_sem_id_);
}

void RobotFun::get_arm_joint_positions(double * positions)
{
    sem_common::semaphore_p(arm_state_sem_id_);
    for (unsigned int j=0; j< arm_info_->arm_dof_; j++)
    {
        *(positions+j) = arm_state_shm_->cur_arm_joint_positions_[j];
    }
    sem_common::semaphore_v(arm_state_sem_id_);
}

void RobotFun::get_arm_joint_velocities(double * velocities)
{
    sem_common::semaphore_p(arm_state_sem_id_);
    for (unsigned int j=0; j< arm_info_->arm_dof_; j++)
    {
        *(velocities+j) = arm_state_shm_->cur_arm_joint_velocities_[j];
    }
    sem_common::semaphore_v(arm_state_sem_id_);
}

void RobotFun::get_arm_joint_efforts(double * efforts)
{
    sem_common::semaphore_p(arm_state_sem_id_);
    for (unsigned int j=0; j< arm_info_->arm_dof_; j++)
    {
        *(efforts+j) = arm_state_shm_->cur_arm_joint_efforts_[j];
    }
    sem_common::semaphore_v(arm_state_sem_id_);
}

}
