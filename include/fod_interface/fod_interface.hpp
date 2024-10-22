#ifndef FOD_INTERFACE__FOD_INTERFACE_HPP_
#define FOD_INTERFACE__FOD_INTERFACE_HPP_

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

#include <std_msgs/msg/string.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
// #include <pacmod3_msgs/msg/global_rpt.hpp>
// #include <pacmod3_msgs/msg/steering_cmd.hpp>
// #include <pacmod3_msgs/msg/system_cmd_float.hpp>
// #include <pacmod3_msgs/msg/system_cmd_int.hpp>
// #include <pacmod3_msgs/msg/system_rpt_float.hpp>
// #include <pacmod3_msgs/msg/system_rpt_int.hpp>
// #include <pacmod3_msgs/msg/wheel_speed_rpt.hpp>
#include <tier4_api_msgs/msg/door_status.hpp>
#include <tier4_external_api_msgs/srv/set_door.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "can_msgs/msg/frame.hpp"
#include "ros2_socketcan_msgs/msg/fd_frame.hpp"
#include "fod_interface/LockedData.hpp"
#include "fod_interface/can_messages.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <set>
#include <type_traits>  // 用于std::is_same
#include <unordered_map>
#include <chrono>

namespace pacmod3
{


// CANIDFilter模板类，适配任意消息类型，只要它包含id字段
template<typename T>
class CANIDFilter : public message_filters::SimpleFilter<T> {
public:
  CANIDFilter(const std::set<uint32_t>& target_ids) : target_ids_(target_ids) {}

  void add(const typename T::ConstSharedPtr msg) {
    // 过滤多个ID的CAN消息
    if (target_ids_.find(msg->id) != target_ids_.end()) {
      this->signalMessage(msg);  // 通过消息给后续处理
    }
  }

private:
  std::set<uint32_t> target_ids_;
};

class FodInterface : public rclcpp::Node
{
public:
  using ActuationCommandStamped = tier4_vehicle_msgs::msg::ActuationCommandStamped;
  using ActuationStatusStamped = tier4_vehicle_msgs::msg::ActuationStatusStamped;
  using SteeringWheelStatusStamped = tier4_vehicle_msgs::msg::SteeringWheelStatusStamped;
  using ControlModeCommand = autoware_vehicle_msgs::srv::ControlModeCommand;
  FodInterface();

private:
  /* subscribers */
  // From Autoware
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_indicators_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    hazard_lights_cmd_sub_;
  rclcpp::Subscription<ActuationCommandStamped>::SharedPtr actuation_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_sub_;

  // From Pacmod
  // rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr frames_sub_;
  // rclcpp::Subscription<ros2_socketcan_msgs::msg::FdFrame>::SharedPtr fd_frames_sub_;
  std::unique_ptr<message_filters::Subscriber<can_msgs::msg::Frame>> frames_sub_;
  std::unique_ptr<message_filters::Subscriber<ros2_socketcan_msgs::msg::FdFrame>> fd_frames_sub_;
  std::unique_ptr<CANIDFilter<can_msgs::msg::Frame>> frame_filter_;
  std::unique_ptr<CANIDFilter<ros2_socketcan_msgs::msg::FdFrame>> fd_frame_filter_;

  // rclcpp::Subscription<pacmod3_msgs::msg::SystemRptInt>::SharedPtr rear_door_rpt_sub_;
  // std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>> steer_wheel_rpt_sub_;
  // std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>> wheel_speed_rpt_sub_;
  // std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>> accel_rpt_sub_;
  // std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>> brake_rpt_sub_;
  // std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>> shift_rpt_sub_;
  // std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>> turn_rpt_sub_;
  // std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::GlobalRpt>> global_rpt_sub_;
  // std::unique_ptr<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>> pacmod_feedbacks_sync_;

  /* publishers */
  // To Pacmod
  // std::shared_ptr<lc::LifecyclePublisher<can_msgs::msg::Frame>> frames_pub_;
  // std::shared_ptr<lc::LifecyclePublisher<ros2_socketcan_msgs::msg::FdFrame>> fd_frames_pub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr frames_pub_;
  rclcpp::Publisher<ros2_socketcan_msgs::msg::FdFrame>::SharedPtr fd_frames_pub_;

  // rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr accel_cmd_pub_;
  // rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr brake_cmd_pub_;
  // rclcpp::Publisher<pacmod3_msgs::msg::SteeringCmd>::SharedPtr steer_cmd_pub_;
  // rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr shift_cmd_pub_;
  // rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr turn_cmd_pub_;
  // rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr door_cmd_pub_;
  // rclcpp::Publisher<pacmod3_msgs::msg::SteeringCmd>::SharedPtr raw_steer_cmd_pub_;  // only for debug

  // To Autoware
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_indicators_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    hazard_lights_status_pub_;
  rclcpp::Publisher<ActuationStatusStamped>::SharedPtr actuation_status_pub_;
  rclcpp::Publisher<SteeringWheelStatusStamped>::SharedPtr steering_wheel_status_pub_;
  // rclcpp::Publisher<tier4_api_msgs::msg::DoorStatus>::SharedPtr door_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rpt500_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rpt501_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rpt502_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rpt503_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rpt504_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rpt505_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rpt506_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rpt512_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_rpt_;

  /* ros param */
  bool enable_fd_;
  std::string base_frame_id_;
  int command_timeout_ms_;  // vehicle_cmd timeout [ms]
  bool is_pacmod_rpt_received_ = false;
  bool is_pacmod_enabled_ = false;
  bool is_clear_override_needed_ = false;
  bool prev_override_ = false;
  double loop_rate_;           // [Hz]
  double tire_radius_;         // [m]
  double wheel_base_;          // [m]
  double steering_offset_;     // [rad] def: measured = truth + offset
  double vgr_coef_a_;          // variable gear ratio coeffs
  double vgr_coef_b_;          // variable gear ratio coeffs
  double vgr_coef_c_;          // variable gear ratio coeffs
  double accel_pedal_offset_;  // offset of accel pedal value
  double brake_pedal_offset_;  // offset of brake pedal value

  double emergency_brake_;              // brake command when emergency [m/s^2]
  bool use_external_emergency_brake_;   // set to true to not use emergency_brake_
  double max_throttle_;                 // max throttle [0~1]
  double max_brake_;                    // max throttle [0~1]
  double max_steering_wheel_;           // max steering wheel angle [rad]
  double max_steering_wheel_rate_;      // [rad/s]
  double min_steering_wheel_rate_;      // [rad/s]
  double steering_wheel_rate_low_vel_;  // [rad/s]
  double steering_wheel_rate_stopped_;  // [rad/s]
  double low_vel_thresh_;               // [m/s]

  bool enable_steering_rate_control_;  // use steering angle speed for command [rad/s]

  double hazard_thresh_time_;
  int hazard_recover_count_ = 0;
  const int hazard_recover_cmd_num_ = 5;

  double margin_time_for_gear_change_;  // [s]

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // Service
  tier4_api_utils::Service<tier4_external_api_msgs::srv::SetDoor>::SharedPtr srv_;
  rclcpp::Service<ControlModeCommand>::SharedPtr control_mode_server_;

  /* input values */
  ActuationCommandStamped::ConstSharedPtr actuation_cmd_ptr_;
  autoware_control_msgs::msg::Control::ConstSharedPtr control_cmd_ptr_;
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr turn_indicators_cmd_ptr_;
  autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr hazard_lights_cmd_ptr_;
  autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;
  tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr emergency_cmd_ptr_;

  // pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt_ptr_;  // [rad]
  // pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt_ptr_;   // [m/s]
  // pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt_ptr_;
  // pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt_ptr_;   // [m/s]
  // pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr gear_cmd_rpt_ptr_;  // [m/s]
  // pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt_ptr_;       // [m/s]
  // pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt_ptr_;
  // pacmod3_msgs::msg::SteeringCmd prev_steer_cmd_;

  bool engage_cmd_{false};
  rclcpp::Time control_command_received_time_{rclcpp::Time(0, 0)};
  rclcpp::Time actuation_command_received_time_;
  rclcpp::Time last_shift_inout_matched_time_;
  std::shared_ptr<rclcpp::Time> last_time_to_change_gear_ptr_;
  uint16_t prev_gear_command_ = static_cast<uint16_t>(pacmod3::GearTarget::PARK);

  /* callbacks */
  void callbackActuationCmd(const ActuationCommandStamped::ConstSharedPtr msg);
  void callbackControlCmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
  void callbackEmergencyCmd(
    const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
  void callbackGearCmd(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void callbackTurnIndicatorsCommand(
    const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  void callbackHazardLightsCommand(
    const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);

  /*  functions */
  void publishCommands();
  double calculateVehicleVelocity();
  double calculateVariableGearRatio(const double vel, const double steer_wheel);
  double calcSteerWheelRateCmd(const double gear_ratio);
  GearTarget toFodShiftCmd(const uint8_t & gear_cmd);
  uint16_t getGearCmdForPreventChatter(uint16_t gear_command);

  std::optional<int32_t> toAutowareShiftReport(GearTarget gear_actual);
  int32_t toAutowareTurnIndicatorsReport(FodTurnLight turn_light_actual);
  int32_t toAutowareHazardLightsReport(FodTurnLight turn_light_actual);
  double steerWheelRateLimiter(
    const double current_steer_cmd, const double prev_steer_cmd,
    const rclcpp::Time & current_steer_time, const rclcpp::Time & prev_steer_time,
    const double steer_rate, const double current_steer_output, const bool engage);
  void onControlModeRequest(
    const ControlModeCommand::Request::SharedPtr request,
    const ControlModeCommand::Response::SharedPtr response);

  // windsome添加的
  std::unordered_map<unsigned int, std::pair<rclcpp::Time, std::shared_ptr<Fod3TxMsg>>> can_rpts_; // CAN上报到autoware消息映射
  rclcpp::Time can_rpts_time_{rclcpp::Time(0, 0)};
  // template<typename T>
  // void callbackFromVehicleCanX(const typename T::SharedPtr msg);
  void callbackFromVehicleCan(const can_msgs::msg::Frame::ConstSharedPtr& msg);
  void callbackCanRptWrap();
  void callbackCanRpt();
  std::unordered_map<unsigned int, std::pair<rclcpp::Time, std::shared_ptr<LockedData>>> can_cmds_; // autoware下发到CAN命令映射
  void prepareCommands();
  FodTurnLight toFodTurnCmd(const uint8_t & turn, const uint8_t & hazard);
  static constexpr auto INTER_MSG_PAUSE = std::chrono::milliseconds(1);
  // rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_rx_;

  void get_params();
  void printBuf(uint32_t id, std::vector<uint8_t>& data);

  bool started_{false};
  bool warmed_{false};
  void warmMachine();
};

}
#endif  // FOD_INTERFACE__FOD_INTERFACE_HPP_
