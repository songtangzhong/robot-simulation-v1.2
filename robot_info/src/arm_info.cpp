#include <robot_info/arm_info.h>
#include <robot_info/robot_basic_macro.h>

namespace arm_info
{
ArmInfo::ArmInfo()
{
    arm_dof_ = ARM_DOF;

    arm_joint_names_.resize(arm_dof_);
    
    cur_arm_joint_positions_.resize(arm_dof_);
    cur_arm_joint_velocities_.resize(arm_dof_);
    cur_arm_joint_efforts_.resize(arm_dof_);

    cmd_arm_joint_positions_.resize(arm_dof_);
    cmd_arm_joint_velocities_.resize(arm_dof_);
    cmd_arm_joint_efforts_.resize(arm_dof_);

    arm_control_modes_.resize(arm_dof_);

    arm_joint_names_ = {"panda_joint1","panda_joint2","panda_joint3","panda_joint4",
                        "panda_joint5","panda_joint6","panda_joint7"};

    for (unsigned int j=0; j<arm_dof_; j++)
    {
        cur_arm_joint_positions_[j] = cmd_arm_joint_positions_[j] = 0;
        cur_arm_joint_velocities_[j] = cmd_arm_joint_velocities_[j] = 0;
        cur_arm_joint_efforts_[j] = cmd_arm_joint_efforts_[j] = 0;

        arm_control_modes_[j] = arm_position_mode_; // default: position mode
    }
    cur_arm_joint_positions_[3] = cmd_arm_joint_positions_[3] =  -1.5;
    cur_arm_joint_positions_[3] = cmd_arm_joint_positions_[5] = 1.88;

    arm_shm_key_ = ARM_SHM_KEY;
    arm_sem_key_ = ARM_SEM_KEY;

    arm_state_shm_key_ = ARM_STATE_SHM_KEY;
    arm_state_sem_key_ = ARM_STATE_SEM_KEY;
}

ArmInfo::~ArmInfo(){}

}
