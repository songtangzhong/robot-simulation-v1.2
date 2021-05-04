#ifndef END_EFF_SHM_H_
#define END_EFF_SHM_H_

#include <robot_info/robot_basic_macro.h>

namespace end_eff_shm
{
typedef struct
{
    double cur_end_eff_joint_positions_[END_EFF_DOF];
    double cur_end_eff_joint_velocities_[END_EFF_DOF];
    double cur_end_eff_joint_efforts_[END_EFF_DOF];

    double cmd_end_eff_joint_positions_[END_EFF_DOF];
    double cmd_end_eff_joint_velocities_[END_EFF_DOF];
    double cmd_end_eff_joint_efforts_[END_EFF_DOF];

    unsigned int end_eff_control_modes_[END_EFF_DOF];
} EndEffShm;

}

#endif