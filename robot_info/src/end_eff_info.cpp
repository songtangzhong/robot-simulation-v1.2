#include <robot_info/end_eff_info.h>
#include <robot_info/robot_basic_macro.h>

namespace end_eff_info
{
EndEffInfo::EndEffInfo()
{
    end_eff_dof_ = END_EFF_DOF;

    end_eff_joint_names_.resize(end_eff_dof_);
    
    cur_end_eff_joint_positions_.resize(end_eff_dof_);
    cur_end_eff_joint_velocities_.resize(end_eff_dof_);
    cur_end_eff_joint_efforts_.resize(end_eff_dof_);

    cmd_end_eff_joint_positions_.resize(end_eff_dof_);
    cmd_end_eff_joint_velocities_.resize(end_eff_dof_);
    cmd_end_eff_joint_efforts_.resize(end_eff_dof_);

    end_eff_control_modes_.resize(end_eff_dof_);

    end_eff_joint_names_ = {"panda_finger_joint1","panda_finger_joint2"};

    for (unsigned int j=0; j<end_eff_dof_; j++)
    {
        cur_end_eff_joint_positions_[j] = cmd_end_eff_joint_positions_[j] = 0;
        cur_end_eff_joint_velocities_[j] = cmd_end_eff_joint_velocities_[j] = 0;
        cur_end_eff_joint_efforts_[j] = cmd_end_eff_joint_efforts_[j] = 0;

        end_eff_control_modes_[j] = end_eff_position_mode_; // default: position mode
    }

    end_eff_shm_key_ = END_EFF_SHM_KEY;
    end_eff_sem_key_ = END_EFF_SEM_KEY;
}

EndEffInfo::~EndEffInfo(){}

}
