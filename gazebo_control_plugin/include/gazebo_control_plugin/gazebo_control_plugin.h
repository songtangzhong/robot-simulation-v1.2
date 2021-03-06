#ifndef GAZEBO_CONTROL_PLUGIN_H_
#define GAZEBO_CONTROL_PLUGIN_H_

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Joint.hh>

#include <robot_info/arm_info.h>
#include <robot_info/robot_info.h>
#include <robot_info/robot_basic_macro.h>
#include <process_commu/arm_shm.h>
#include <process_commu/robot_state_shm.h>
#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>

#if END_EFF_TRUE
#include <robot_info/end_eff_info.h>
#include <process_commu/end_eff_shm.h>
#endif

namespace gazebo_control_plugin
{
class ControlPlugin : public gazebo::ModelPlugin
{
public:
    ControlPlugin();
    ~ControlPlugin();

    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    void Update();

private:
    gazebo::physics::ModelPtr parent_model_;

    std::vector<gazebo::physics::JointPtr> arm_joints_;

#if END_EFF_TRUE
    std::vector<gazebo::physics::JointPtr> end_eff_joints_;
#endif

    gazebo::event::ConnectionPtr update_connection_;

    std::shared_ptr<arm_info::ArmInfo> arm_info_ = std::make_shared<arm_info::ArmInfo>();
    arm_shm::ArmShm *arm_shm_;
    int arm_shm_id_;
    int arm_sem_id_;

    std::shared_ptr<robot_info::RobotInfo> robot_info_ = std::make_shared<robot_info::RobotInfo>();
    robot_state_shm::RobotStateShm *robot_state_shm_;
    int robot_state_shm_id_;
    int robot_state_sem_id_;

#if END_EFF_TRUE
    std::shared_ptr<end_eff_info::EndEffInfo> end_eff_info_ = std::make_shared<end_eff_info::EndEffInfo>();
    end_eff_shm::EndEffShm *end_eff_shm_;
    int end_eff_shm_id_;
    int end_eff_sem_id_;
#endif
}; 

}

#endif
