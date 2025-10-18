#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "namiashi_control_hw/namiashi_hardware_interface.h"
#include "namiashi_control_hw/serial_motor.h" // 追加

// #define DP_DISABLE_SERVO_CONTROL
// #define DP_DISABLE_PERIODIC_GETPOS

namespace dp {

  double deg2rad(double deg) {
    return deg * (M_PI / 180.0);
  }

  double rad2deg(double rad) {
    return rad * (180.0 / M_PI);
  }

#if 0
  Motor motors_[kDof] = {
    {0x0a, "FL_hip_joint",   +1, 1.00,  +34.0,    0.0,   0.0},  // 10
    {0x0b, "FL_thigh_joint", +1, 1.00,  +67.0,  +45.0, +45.0},  // 11
    {0x0c, "FL_calf_joint",  +1, 1.47, -165.0, -162.0, -90.0},  // 12

    {0x0d, "RL_hip_joint",   -1, 1.00,  +34.0,    0.0,   0.0},  // 13
    {0x0e, "RL_thigh_joint", +1, 1.00,  +67.0,  +50.0, +45.0},  // 14
    {0x0f, "RL_calf_joint",  +1, 1.47, -165.0,  -75.0, -90.0},  // 15

    {0x14, "FR_hip_joint",   +1, 1.00,  -34.0,    0.0,   0.0},  // 20
    {0x15, "FR_thigh_joint", -1, 1.00,  +67.0,  +45.0, +45.0},  // 21
    {0x16, "FR_calf_joint",  -1, 1.47, -165.0, -162.0, -90.0},  // 22

    {0x17, "RR_hip_joint",   -1, 1.00,  -34.0,    0.0,   0.0},  // 23
    {0x18, "RR_thigh_joint", -1, 1.00,  +67.0,  +50.0, +45.0},  // 24
    {0x19, "RR_calf_joint",  -1, 1.47, -165.0,  -75.0, -90.0}   // 25
  };
#else
  Motor motors_[kDof] = {
    {0x0a, "FL_hip_joint",   +1, 1.00,  +34.0,    0.0,   0.0},  // 10
    {0x0b, "FL_thigh_joint", +1, 1.00,  +67.0,  +45.0, +45.0},  // 11
    {0x0c, "FL_calf_joint",  +1, 1.47, -160.0, -162.0, -90.0},  // 12

    {0x0d, "RL_hip_joint",   -1, 1.00,  +34.0,    0.0,   0.0},  // 13
    {0x0e, "RL_thigh_joint", +1, 1.00,  +67.0,  +50.0, +45.0},  // 14
    {0x0f, "RL_calf_joint",  +1, 1.47, -160.0,  -75.0, -90.0},  // 15

    {0x14, "FR_hip_joint",   +1, 1.00,  -34.0,    0.0,   0.0},  // 20
    {0x15, "FR_thigh_joint", -1, 1.00,  +67.0,  +45.0, +45.0},  // 21
    {0x16, "FR_calf_joint",  -1, 1.47, -160.0, -162.0, -90.0},  // 22

    {0x17, "RR_hip_joint",   -1, 1.00,  -34.0,    0.0,   0.0},  // 23
    {0x18, "RR_thigh_joint", -1, 1.00,  +67.0,  +50.0, +45.0},  // 24
    {0x19, "RR_calf_joint",  -1, 1.47, -160.0,  -75.0, -90.0}   // 25
  };
#endif

  NamiashiHardwareInterface::NamiashiHardwareInterface() {    
    for (size_t i = 0; i < kDof; i++) {
      auto& motor = motors_[i];
      auto& name  = motor.name;
      hardware_interface::JointStateHandle state_handle(name, &pos_[i], &vel_[i], &eff_[i]);
      jnt_state_interface_.registerHandle(state_handle);
    }

    for (size_t i = 0; i < kDof; i++) {
      auto& motor = motors_[i];
      auto& name  = motor.name;
      hardware_interface::JointHandle position_handle(jnt_state_interface_.getHandle(name), &cmd_[i]);
      jnt_position_interface_.registerHandle(position_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_position_interface_);
  }

  // 追加: mdeg計算のプライベート関数
  double NamiashiHardwareInterface::calc_mdeg(const Motor& motor, double cmd_rad) const {
    return (rad2deg(cmd_rad) - motor.offset) * motor.ratio * motor.dir;
  }

  void NamiashiHardwareInterface::init(const std::string& port, int baudrate) { 
    lkm_ = std::make_shared<lkmotor>(port, baudrate); // モータインスタンス生成

    // 必要なら初期化処理
    for (size_t i = 0; i < kDof; i++) {
      auto id  = motors_[i].id;
      lkm_->connect(id);
      lkm_->set_0deg(id);
    }
    get_current_positions();
    for (size_t i = 0; i < kDof; i++) {
      auto& motor = motors_[i];
      auto id     = motor.id;
      auto& name  = motor.name;
      cmd_[i] = pos_[i]; // 現在位置を初期コマンドにセット
      ROS_INFO("%s : %5.2lf", name.c_str(), cmd_[i]);
    }
    get_current_positions();

#if defined(DP_DISABLE_SERVO_CONTROL)
#else
    sleep(1);

    ROS_INFO("================== go to starting pose ====================");
    for (size_t j = 0; j < 5; j++) {
      for (size_t i = 0; i < kDof; i++) {
        auto& motor = motors_[i];
        cmd_[i] = deg2rad(motor.initial);
        auto mdeg = calc_mdeg(motor, cmd_[i]);
        lkm_->send_mdeg(motors_[i].id, mdeg, 20);
        ROS_INFO("%s : %5.2lf[deg] %5.2lf[rad]", motors_[i].name.c_str(), mdeg, cmd_[i]);
        usleep(1000);
      }
      usleep(10 * 1000);
    }
    sleep(10);
    // wait trigger to start
#endif
  }

  void NamiashiHardwareInterface::default_pose() { 
    assert(lkm_);
#if defined(DP_DISABLE_SERVO_CONTROL)
#else
    ROS_INFO("================== go to default pose =====================");
    for (size_t j = 0; j < 5; j++) {
      for (size_t i = 0; i < kDof; i++) {
        auto& motor = motors_[i];
        cmd_[i] = deg2rad(motor.default_pose);
        auto mdeg = calc_mdeg(motor, cmd_[i]);
        lkm_->send_mdeg(motors_[i].id, mdeg, 150);
        ROS_INFO("%s : %5.2lf[deg] %5.2lf[rad]", motors_[i].name.c_str(), mdeg, cmd_[i]);
        usleep(1000);
      }
      usleep(10 * 1000);
    }
    sleep(2);
#endif
  }

  void NamiashiHardwareInterface::read() {
#if defined(DP_DISABLE_PERIODIC_GETPOS)
#else
    // モータ角度取得
    if (count_ % 100 == 0) {
      // ROS_INFO("%s : %s", __func__, __FILE__);
      // ROS_DEBUG("%s : %s", __func__, __FILE__);
      get_current_positions();
    }
#endif
  }

  void NamiashiHardwareInterface::write() {
    count_++;
#if defined(DP_DISABLE_SERVO_CONTROL)
    for (size_t i = 0; i < kDof; i++) {
      usleep(1000);
    }
#else
    for (size_t i = 0; i < kDof; i++) {
      // ROS_INFO("%s : %5.2lf[rad]", motors_[i].name.c_str(), cmd_[i]);
      auto& motor = motors_[i];
      auto mdeg = calc_mdeg(motor, cmd_[i]);
      lkm_->send_mdeg(motor.id, mdeg, 150);
      // lkm_->send_mdeg(motor.id, mdeg);
      // usleep(1000);
      usleep(600);
    }
#endif
  }

  void NamiashiHardwareInterface::get_current_positions() {
    for (size_t i = 0; i < kDof; i++) {
      auto id  = motors_[i].id;
      ssize_t retry_count = 5;
      while(retry_count--) {
        auto val = lkm_->get_pos(id);
        if (val) {
          auto fval = val.value() * motors_[i].dir / motors_[i].ratio + motors_[i].offset;
          pos_[i] = deg2rad(fval);
          ROS_INFO("id:%02x : %5.2lf[rad] %5.2lf[deg] -> %5.2lf[rad]", id, pos_[i], fval, cmd_[i]);
          break;
        }
        if (retry_count == 0) {
          ROS_ERROR("id:%02x : no response", id);
        }
      }
    }
    usleep(100);
  }

}

PLUGINLIB_EXPORT_CLASS(dp::NamiashiHardwareInterface, hardware_interface::RobotHW);
