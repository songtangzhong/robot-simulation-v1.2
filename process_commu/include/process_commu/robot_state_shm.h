#ifndef ROBOT_STATE_SHM_H_
#define ROBOT_STATE_SHM_H_

#include <robot_info/robot_basic_macro.h>

namespace robot_state_shm
{
typedef struct
{
    double cur_arm_joint_positions_[ARM_DOF];
    double cur_arm_joint_velocities_[ARM_DOF];
    double cur_arm_joint_efforts_[ARM_DOF];

#if END_EFF_TRUE
    double cur_end_eff_joint_positions_[END_EFF_DOF];
    double cur_end_eff_joint_velocities_[END_EFF_DOF];
    double cur_end_eff_joint_efforts_[END_EFF_DOF];
#endif
} RobotStateShm;

}

#endif
