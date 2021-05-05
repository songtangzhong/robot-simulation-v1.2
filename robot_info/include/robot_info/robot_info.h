#ifndef ROBOT_INFO_H_
#define ROBOT_INFO_H_

#include <robot_info/arm_info.h>
#include <robot_info/end_eff_info.h>

namespace robot_info
{
class RobotInfo
{
public:
    RobotInfo();
    ~RobotInfo();

    std::shared_ptr<arm_info::ArmInfo> arm_info_ = std::make_shared<arm_info::ArmInfo>();

    std::shared_ptr<end_eff_info::EndEffInfo> end_eff_info_ = std::make_shared<end_eff_info::EndEffInfo>();

    key_t robot_state_shm_key_;
    key_t robot_state_sem_key_;
};

}

#endif