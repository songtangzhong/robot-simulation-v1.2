#include <gazebo_control_plugin/gazebo_control_plugin.h>
#include <rclcpp/rclcpp.hpp>

namespace gazebo_control_plugin
{
ControlPlugin::ControlPlugin()
{
    RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Start Robot Simulation ...");

    ///////////////////////////////////////////////////////////////////////////////////////////
    arm_shm_id_ = shm_common::create_shm(arm_info_->arm_shm_key_, &arm_shm_);
    if (arm_shm_id_ != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Create arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Create arm shared memory failed.");
    }

    for (unsigned int j=0; j< arm_info_->arm_dof_; j++)
    {
        arm_shm_->cur_arm_joint_positions_[j] = arm_info_->cur_arm_joint_positions_[j];
        arm_shm_->cur_arm_joint_velocities_[j] = arm_info_->cur_arm_joint_velocities_[j];
        arm_shm_->cur_arm_joint_efforts_[j] = arm_info_->cur_arm_joint_efforts_[j];

        arm_shm_->cmd_arm_joint_positions_[j] = arm_info_->cmd_arm_joint_positions_[j];
        arm_shm_->cmd_arm_joint_velocities_[j] = arm_info_->cmd_arm_joint_velocities_[j];
        arm_shm_->cmd_arm_joint_efforts_[j] = arm_info_->cmd_arm_joint_efforts_[j];

        arm_shm_->arm_control_modes_[j] = arm_info_->arm_control_modes_[j];
    }

    arm_sem_id_ = sem_common::create_semaphore(arm_info_->arm_sem_key_);
    if (arm_sem_id_ != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Create arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Create arm semaphore failed.");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    robot_state_shm_id_ = shm_common::create_shm(robot_info_->robot_state_shm_key_, &robot_state_shm_);
    if (robot_state_shm_id_ != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Create robot state shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Create robot state shared memory failed.");
    }

    for (unsigned int j=0; j< robot_info_->arm_info_->arm_dof_; j++)
    {
        robot_state_shm_->cur_arm_joint_positions_[j] = robot_info_->arm_info_->cur_arm_joint_positions_[j];
        robot_state_shm_->cur_arm_joint_velocities_[j] = robot_info_->arm_info_->cur_arm_joint_velocities_[j];
        robot_state_shm_->cur_arm_joint_efforts_[j] = robot_info_->arm_info_->cur_arm_joint_efforts_[j];
    }
#if END_EFF_TRUE
    for (unsigned int j=0; j< robot_info_->end_eff_info_->end_eff_dof_; j++)
    {
        robot_state_shm_->cur_end_eff_joint_positions_[j] = robot_info_->end_eff_info_->cur_end_eff_joint_positions_[j];
        robot_state_shm_->cur_end_eff_joint_velocities_[j] = robot_info_->end_eff_info_->cur_end_eff_joint_velocities_[j];
        robot_state_shm_->cur_end_eff_joint_efforts_[j] = robot_info_->end_eff_info_->cur_end_eff_joint_efforts_[j];
    }
#endif

    robot_state_sem_id_ = sem_common::create_semaphore(robot_info_->robot_state_sem_key_);
    if (robot_state_sem_id_ != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Create robot state semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Create robot state semaphore failed.");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
#if END_EFF_TRUE
    end_eff_shm_id_ = shm_common::create_shm(end_eff_info_->end_eff_shm_key_, &end_eff_shm_);
    if (end_eff_shm_id_ != SHM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Create end-effector shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Create end-effector shared memory failed.");
    }

    for (unsigned int j=0; j< end_eff_info_->end_eff_dof_; j++)
    {
        end_eff_shm_->cur_end_eff_joint_positions_[j] = end_eff_info_->cur_end_eff_joint_positions_[j];
        end_eff_shm_->cur_end_eff_joint_velocities_[j] = end_eff_info_->cur_end_eff_joint_velocities_[j];
        end_eff_shm_->cur_end_eff_joint_efforts_[j] = end_eff_info_->cur_end_eff_joint_efforts_[j];

        end_eff_shm_->cmd_end_eff_joint_positions_[j] = end_eff_info_->cmd_end_eff_joint_positions_[j];
        end_eff_shm_->cmd_end_eff_joint_velocities_[j] = end_eff_info_->cmd_end_eff_joint_velocities_[j];
        end_eff_shm_->cmd_end_eff_joint_efforts_[j] = end_eff_info_->cmd_end_eff_joint_efforts_[j];

        end_eff_shm_->end_eff_control_modes_[j] = end_eff_info_->end_eff_control_modes_[j];
    }

    end_eff_sem_id_ = sem_common::create_semaphore(end_eff_info_->end_eff_sem_key_);
    if (end_eff_sem_id_ != SEM_STATE_NO)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Create end-effector semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Create end-effector semaphore failed.");
    }
#endif
}

ControlPlugin::~ControlPlugin()
{ 
    if (shm_common::release_shm(arm_shm_id_, &arm_shm_) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Release arm shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Release arm shared memory failed.");
    }

    if (sem_common::delete_semaphore(arm_sem_id_) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Delete arm semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Delete arm semaphore failed.");
    }

    ////////////////////////////////////////////////////////////////////////////////////
    if (shm_common::release_shm(robot_state_shm_id_, &robot_state_shm_) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Release robot state shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Release robot state shared memory failed.");
    }

    if (sem_common::delete_semaphore(robot_state_sem_id_) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Delete robot state semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Delete robot state semaphore failed.");
    }

    ////////////////////////////////////////////////////////////////////////////////////
#if END_EFF_TRUE
    if (shm_common::release_shm(end_eff_shm_id_, &end_eff_shm_) == SHM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Release end-effector shared memory successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Release end-effector shared memory failed.");
    }

    if (sem_common::delete_semaphore(end_eff_sem_id_) == SEM_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Delete end-effector semaphore successfully.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("gazebo"), "Delete end-effector semaphore failed.");
    }
#endif

    RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Simulation has been finished.");
}

void ControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Load control plugin ...");

  parent_model_ = parent;

  double sim_rate = parent_model_->GetWorld()->Physics()->GetRealTimeUpdateRate();
  RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Simulation rate: %.2f Hz.", sim_rate);

  for (unsigned int j=0; j< arm_info_->arm_dof_; j++)
  {
     gazebo::physics::JointPtr arm_joint = parent_model_->GetJoint(arm_info_->arm_joint_names_[j]);
     arm_joints_.push_back(arm_joint);
  }

#if END_EFF_TRUE
  for (unsigned int j=0; j< end_eff_info_->end_eff_dof_; j++)
  {
     gazebo::physics::JointPtr end_eff_joint = parent_model_->GetJoint(end_eff_info_->end_eff_joint_names_[j]);
     end_eff_joints_.push_back(end_eff_joint);
  }
#endif

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ControlPlugin::Update, this));

  RCLCPP_INFO(rclcpp::get_logger("gazebo"), "Load control plugin successfully.");
}

void ControlPlugin::Update()
{
  /* Note: 
     We must do writing operation, and then do reading operation (for gazebo) in this function.
     Also, we must do reading operation, and then do writing operation (for ROS2) in ROS2 controller manager.
  */

  sem_common::semaphore_p(arm_sem_id_);
  for (unsigned int j=0; j< arm_info_->arm_dof_; j++)
  {
    if ((arm_info_->arm_control_modes_[j]=arm_shm_->arm_control_modes_[j]) & arm_info_->arm_position_mode_)
    {
      arm_joints_[j]->SetPosition(0, arm_info_->cmd_arm_joint_positions_[j]=arm_shm_->cmd_arm_joint_positions_[j]);
    }
    else if ((arm_info_->arm_control_modes_[j]=arm_shm_->arm_control_modes_[j]) & arm_info_->arm_velocity_mode_)
    {
      arm_joints_[j]->SetVelocity(0, arm_info_->cmd_arm_joint_velocities_[j]=arm_shm_->cmd_arm_joint_velocities_[j]);
    }
    else if ((arm_info_->arm_control_modes_[j]=arm_shm_->arm_control_modes_[j]) & arm_info_->arm_effort_mode_)
    {
      arm_joints_[j]->SetForce(0, arm_info_->cmd_arm_joint_efforts_[j]=arm_shm_->cmd_arm_joint_efforts_[j]);
    }
    
    arm_shm_->cur_arm_joint_positions_[j] = arm_info_->cur_arm_joint_positions_[j] = arm_joints_[j]->Position(0);
    arm_shm_->cur_arm_joint_velocities_[j] = arm_info_->cur_arm_joint_velocities_[j] = arm_joints_[j]->GetVelocity(0);
    arm_shm_->cur_arm_joint_efforts_[j] = arm_info_->cur_arm_joint_efforts_[j] = arm_joints_[j]->GetForce(0u);
  }
  sem_common::semaphore_v(arm_sem_id_);

#if END_EFF_TRUE
  sem_common::semaphore_p(end_eff_sem_id_);
  for (unsigned int j=0; j< end_eff_info_->end_eff_dof_; j++)
  {
    if ((end_eff_info_->end_eff_control_modes_[j]=end_eff_shm_->end_eff_control_modes_[j]) & end_eff_info_->end_eff_position_mode_)
    {
      end_eff_joints_[j]->SetPosition(0, end_eff_info_->cmd_end_eff_joint_positions_[j]=end_eff_shm_->cmd_end_eff_joint_positions_[j]);
    }
    else if ((end_eff_info_->end_eff_control_modes_[j]=end_eff_shm_->end_eff_control_modes_[j]) & end_eff_info_->end_eff_velocity_mode_)
    {
      end_eff_joints_[j]->SetVelocity(0, end_eff_info_->cmd_end_eff_joint_velocities_[j]=end_eff_shm_->cmd_end_eff_joint_velocities_[j]);
    }
    else if ((end_eff_info_->end_eff_control_modes_[j]=end_eff_shm_->end_eff_control_modes_[j]) & end_eff_info_->end_eff_effort_mode_)
    {
      end_eff_joints_[j]->SetForce(0, end_eff_info_->cmd_end_eff_joint_efforts_[j]=end_eff_shm_->cmd_end_eff_joint_efforts_[j]);
    }
    
    end_eff_shm_->cur_end_eff_joint_positions_[j] = end_eff_info_->cur_end_eff_joint_positions_[j] = end_eff_joints_[j]->Position(0);
    end_eff_shm_->cur_end_eff_joint_velocities_[j] = end_eff_info_->cur_end_eff_joint_velocities_[j] = end_eff_joints_[j]->GetVelocity(0);
    end_eff_shm_->cur_end_eff_joint_efforts_[j] = end_eff_info_->cur_end_eff_joint_efforts_[j] = end_eff_joints_[j]->GetForce(0u);
  }
  sem_common::semaphore_v(end_eff_sem_id_);
#endif
}

GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

}
