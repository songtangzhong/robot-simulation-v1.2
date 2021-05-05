#ifndef ARM_STATE_SHM_H_
#define ARM_STATE_SHM_H_

#include <robot_info/robot_basic_macro.h>

namespace arm_state_shm
{
typedef struct
{
    double cur_arm_joint_positions_[ARM_DOF];
    double cur_arm_joint_velocities_[ARM_DOF];
    double cur_arm_joint_efforts_[ARM_DOF];
} ArmStateShm;

}

#endif
