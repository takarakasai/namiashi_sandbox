#pragma once

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include "namiashi_control_hw/serial_motor.h"

namespace dp {
  
constexpr size_t kDof = 12;

struct Motor {
  uint8_t id;
  std::string name;
  int8_t dir;
  double ratio;
  double offset;
  double initial;
  double default_pose;
};

class NamiashiHardwareInterface : public hardware_interface::RobotHW {
public:
    NamiashiHardwareInterface();
    void init(const std::string& port, int baudrate);
    void default_pose();
    void read();
    void write();

private:
    void get_current_positions();
    double calc_mdeg(const Motor& motor, double cmd_rad) const;

    double pos_[kDof] = {0.0};
    double vel_[kDof] = {0.0};
    double eff_[kDof] = {0.0};
    double cmd_[kDof] = {0.0};

    int count_ = 0;
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_position_interface_;

    std::shared_ptr<lkmotor> lkm_;
};

}