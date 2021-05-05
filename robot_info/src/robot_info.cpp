#include <robot_info/robot_info.h>
#include <robot_info/robot_basic_macro.h>

namespace robot_info
{
RobotInfo::RobotInfo()
{
    robot_state_shm_key_ = ROBOT_STATE_SHM_KEY;
    robot_state_sem_key_ = ROBOT_STATE_SEM_KEY;
}

RobotInfo::~RobotInfo(){}

}