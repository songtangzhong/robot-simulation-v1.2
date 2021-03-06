#ifndef ROBOT_HW_INTERFACE_H_
#define ROBOT_HW_INTERFACE_H_

#include <memory>
#include <string>
#include <vector>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_info/arm_info.h>
#include <robot_info/robot_basic_macro.h>
#include <process_commu/arm_shm.h>
#include <process_commu/shm_common.h>
#include <process_commu/sem_common.h>

#if END_EFF_TRUE
#include <robot_info/end_eff_info.h>
#include <process_commu/end_eff_shm.h>
#endif

namespace robot_hw
{
class RobotHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;

  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

private:
  double start_duration_sec_;
  double stop_duration_sec_;

  std::shared_ptr<arm_info::ArmInfo> arm_info_ = std::make_shared<arm_info::ArmInfo>();
  arm_shm::ArmShm *arm_shm_;
  int arm_shm_id_;
  int arm_sem_id_;

#if END_EFF_TRUE
  std::shared_ptr<end_eff_info::EndEffInfo> end_eff_info_ = std::make_shared<end_eff_info::EndEffInfo>();
  end_eff_shm::EndEffShm *end_eff_shm_;
  int end_eff_shm_id_;
  int end_eff_sem_id_;
#endif
};

}

#endif
