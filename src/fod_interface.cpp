#include <fod_interface/fod_interface.hpp>
#include <fod_interface/can_messages.hpp>
#include <fod_interface/utils.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <sstream>  // for std::ostringstream

namespace pacmod3
{
constexpr std::chrono::milliseconds FodInterface::INTER_MSG_PAUSE;
// using GearTarget = pacmod3::GearTarget;
// using FodTurnLight = pacmod3::FodTurnLight;

FodInterface::FodInterface()
: Node("fod_interface"),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  enable_fd_ = this->declare_parameter("enable_can_fd", false);

  /* setup parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000);
  loop_rate_ = declare_parameter("loop_rate", 30.0);

  /* parameters for vehicle specifications */
  tire_radius_ = vehicle_info_.wheel_radius_m;
  wheel_base_ = vehicle_info_.wheel_base_m;

  steering_offset_ = declare_parameter("steering_offset", 0.0);
  enable_steering_rate_control_ = declare_parameter("enable_steering_rate_control", false);

  /* parameters for emergency stop */
  emergency_brake_ = declare_parameter("emergency_brake", 0.7);
  use_external_emergency_brake_ = declare_parameter("use_external_emergency_brake", false);

  /* vehicle parameters */
  vgr_coef_a_ = declare_parameter("vgr_coef_a", 15.713);
  vgr_coef_b_ = declare_parameter("vgr_coef_b", 0.053);
  vgr_coef_c_ = declare_parameter("vgr_coef_c", 0.042);
  accel_pedal_offset_ = declare_parameter("accel_pedal_offset", 0.0);
  brake_pedal_offset_ = declare_parameter("brake_pedal_offset", 0.0);

  /* parameters for limitter */
  max_throttle_ = declare_parameter("max_throttle", 0.2);
  max_brake_ = declare_parameter("max_brake", 0.8);
  max_steering_wheel_ = declare_parameter("max_steering_wheel", 2.7 * M_PI);
  max_steering_wheel_rate_ = declare_parameter("max_steering_wheel_rate", 6.6);
  min_steering_wheel_rate_ = declare_parameter("min_steering_wheel_rate", 0.5);
  steering_wheel_rate_low_vel_ = declare_parameter("steering_wheel_rate_low_vel", 5.0);
  steering_wheel_rate_stopped_ = declare_parameter("steering_wheel_rate_stopped", 5.0);
  low_vel_thresh_ = declare_parameter("low_vel_thresh", 1.389);  // 5.0kmh

  /* parameters for turn signal recovery */
  hazard_thresh_time_ = declare_parameter("hazard_thresh_time", 0.20);  // s

  /* parameter for preventing gear chattering */
  margin_time_for_gear_change_ = declare_parameter("margin_time_for_gear_change", 2.0);

  /* initialize */
  // prev_steer_cmd_.header.stamp = this->now();
  // prev_steer_cmd_.command = 0.0;

  /* subscribers */
  using std::placeholders::_1;
  using std::placeholders::_2;

  // From autoware
  control_cmd_sub_ = create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", 1, std::bind(&FodInterface::callbackControlCmd, this, _1));
  gear_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&FodInterface::callbackGearCmd, this, _1));
  turn_indicators_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
    "/control/command/turn_indicators_cmd", rclcpp::QoS{1},
    std::bind(&FodInterface::callbackTurnIndicatorsCommand, this, _1));
  hazard_lights_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
    "/control/command/hazard_lights_cmd", rclcpp::QoS{1},
    std::bind(&FodInterface::callbackHazardLightsCommand, this, _1));

  actuation_cmd_sub_ = create_subscription<ActuationCommandStamped>(
    "/control/command/actuation_cmd", 1,
    std::bind(&FodInterface::callbackActuationCmd, this, _1));
  emergency_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1,
    std::bind(&FodInterface::callbackEmergencyCmd, this, _1));
  control_mode_server_ = create_service<ControlModeCommand>(
    "input/control_mode_request", std::bind(&FodInterface::onControlModeRequest, this, _1, _2));

  // From pacmod
  // 设定需要过滤的CAN ID列表(底盘上报的CAN消息)
  std::set<uint32_t> can_ids = {0x500, 0x501, 0x502, 0x503, 0x504, 0x505, 0x506, 0x507, 0x508, 0x509, 0x510, 0x511, 0x512, 0x514, 0x515, 0x516};  // 你想要过滤的ID
  if (!enable_fd_) {
    frame_filter_ = std::make_unique<CANIDFilter<can_msgs::msg::Frame>>(can_ids);
    frames_sub_ = std::make_unique<message_filters::Subscriber<can_msgs::msg::Frame>>(this, "from_can_bus");
    // 连接订阅者和过滤器
    // auto callback = std::bind(&pacmod3::CANIDFilter<can_msgs::msg::Frame>::add, this, std::placeholders::_1);
    // frames_sub_->registerCallback(callback);
    frames_sub_->registerCallback(std::bind(&pacmod3::CANIDFilter<can_msgs::msg::Frame>::add, frame_filter_.get(), std::placeholders::_1));
    // frame_filter_->registerCallback(std::bind(&FodInterface::callbackFromVehicleCan, this, std::placeholders::_1));
    // frame_filter_->registerCallback([this](const can_msgs::msg::Frame::ConstSharedPtr& msg) {
    //   // 处理过滤后的消息
    //   std::cout << "Received filtered CAN message with ID: " << msg->id << std::endl;
    // });

    // frames_sub_->registerCallback([this](const can_msgs::msg::Frame::ConstSharedPtr& msg) {
    //   this->frame_filter_->add(msg);  // 调用CANIDFilter的add方法
    // });
    frame_filter_->registerCallback([this](const can_msgs::msg::Frame::ConstSharedPtr& msg) {
      // 处理过滤后的消息
      std::cout << "Received filtered CAN message with ID: " << msg->id << std::endl;
    });

  } else {
    // fd_frame_filter_ = std::make_unique<CANIDFilter<ros2_socketcan_msgs::msg::FdFrame>>(can_ids);
    // fd_frames_sub_ = std::make_unique<message_filters::Subscriber<ros2_socketcan_msgs::msg::FdFrame>>(this, "from_can_bus_fd");
    // // 连接订阅者和过滤器
    // fd_frames_sub_->connectInput(fd_frame_filter_.get());
    // fd_frame_filter_->registerCallback(std::bind(&FodInterface::callbackFromVehicleCanX<ros2_socketcan_msgs::msg::FdFrame>, this, std::placeholders::_1));
  }

  // rear_door_rpt_sub_ = create_subscription<pacmod3_msgs::msg::SystemRptInt>(
  //   "/pacmod/rear_pass_door_rpt", 1, std::bind(&FodInterface::callbackRearDoor, this, _1));

  // steer_wheel_rpt_sub_ =
  //   std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
  //     this, "/pacmod/steering_rpt");
  // wheel_speed_rpt_sub_ =
  //   std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>>(
  //     this, "/pacmod/wheel_speed_rpt");
  // accel_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
  //   this, "/pacmod/accel_rpt");
  // brake_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
  //   this, "/pacmod/brake_rpt");
  // shift_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
  //   this, "/pacmod/shift_rpt");
  // turn_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
  //   this, "/pacmod/turn_rpt");
  // global_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::GlobalRpt>>(
  //   this, "/pacmod/global_rpt");

  // pacmod_feedbacks_sync_ =
  //   std::make_unique<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>>(
  //     PacmodFeedbacksSyncPolicy(10), *steer_wheel_rpt_sub_, *wheel_speed_rpt_sub_, *accel_rpt_sub_,
  //     *brake_rpt_sub_, *shift_rpt_sub_, *turn_rpt_sub_, *global_rpt_sub_);

  // pacmod_feedbacks_sync_->registerCallback(std::bind(
  //   &FodInterface::callbackPacmodRpt, this, std::placeholders::_1, std::placeholders::_2,
  //   std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
  //   std::placeholders::_7));

  /* publisher */
  // To pacmod
  if (!enable_fd_) {
    frames_pub_ = create_publisher<can_msgs::msg::Frame>("to_can_bus", rclcpp::QoS{1});
  } else {
    fd_frames_pub_ = create_publisher<ros2_socketcan_msgs::msg::FdFrame>("to_can_bus_fd", rclcpp::QoS{1});
  }
  // accel_cmd_pub_ =
  //   create_publisher<pacmod3_msgs::msg::SystemCmdFloat>("/pacmod/accel_cmd", rclcpp::QoS{1});
  // brake_cmd_pub_ =
  //   create_publisher<pacmod3_msgs::msg::SystemCmdFloat>("/pacmod/brake_cmd", rclcpp::QoS{1});
  // steer_cmd_pub_ =
  //   create_publisher<pacmod3_msgs::msg::SteeringCmd>("/pacmod/steering_cmd", rclcpp::QoS{1});
  // shift_cmd_pub_ =
  //   create_publisher<pacmod3_msgs::msg::SystemCmdInt>("/pacmod/shift_cmd", rclcpp::QoS{1});
  // turn_cmd_pub_ =
  //   create_publisher<pacmod3_msgs::msg::SystemCmdInt>("/pacmod/turn_cmd", rclcpp::QoS{1});
  // door_cmd_pub_ =
  //   create_publisher<pacmod3_msgs::msg::SystemCmdInt>("/pacmod/rear_pass_door_cmd", rclcpp::QoS{1});
  // raw_steer_cmd_pub_ = create_publisher<pacmod3_msgs::msg::SteeringCmd>(
  //   "/pacmod/raw_steer_cmd", rclcpp::QoS{1});  // only for debug

  // To Autoware
  control_mode_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  vehicle_twist_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  steering_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
  gear_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
  turn_indicators_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
    "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
  hazard_lights_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
  actuation_status_pub_ =
    create_publisher<ActuationStatusStamped>("/vehicle/status/actuation_status", 1);
  steering_wheel_status_pub_ =
    create_publisher<SteeringWheelStatusStamped>("/vehicle/status/steering_wheel_status", 1);
  // door_status_pub_ =
  //   create_publisher<tier4_api_msgs::msg::DoorStatus>("/vehicle/status/door_status", 1);

  /* service */
  //  From autoware
  // tier4_api_utils::ServiceProxyNodeInterface proxy(this);
  // srv_ = proxy.create_service<tier4_external_api_msgs::srv::SetDoor>(
  //   "/api/vehicle/set/door",
  //   std::bind(&FodInterface::setDoor, this, std::placeholders::_1, std::placeholders::_2));

  // Timer
  const auto period_ns = rclcpp::Rate(loop_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&FodInterface::publishCommands, this));

  can_cmds_[ThrottleCmdMsg::CAN_ID] = std::make_pair(this->now(), std::shared_ptr<LockedData>(new LockedData(ThrottleCmdMsg::DATA_LENGTH)));
  can_cmds_[BrakeCmdMsg::CAN_ID] = std::make_pair(this->now(), std::shared_ptr<LockedData>(new LockedData(BrakeCmdMsg::DATA_LENGTH)));
  can_cmds_[SteeringCmdMsg::CAN_ID] = std::make_pair(this->now(), std::shared_ptr<LockedData>(new LockedData(SteeringCmdMsg::DATA_LENGTH)));
  can_cmds_[GearCmdMsg::CAN_ID] = std::make_pair(this->now(), std::shared_ptr<LockedData>(new LockedData(GearCmdMsg::DATA_LENGTH)));
  can_cmds_[ParkCmdMsg::CAN_ID] = std::make_pair(this->now(), std::shared_ptr<LockedData>(new LockedData(ParkCmdMsg::DATA_LENGTH)));
  can_cmds_[VehicleModeCmdMsg::CAN_ID] = std::make_pair(this->now(), std::shared_ptr<LockedData>(new LockedData(VehicleModeCmdMsg::DATA_LENGTH)));
  pub_can_rx_ = create_publisher<can_msgs::msg::Frame>("to_can_bus", 100);

}

// template<typename T>
// void FodInterface::callbackFromVehicleCanX(const typename T::SharedPtr msg) {
//   RCLCPP_INFO(this->get_logger(), "Received CAN frame with ID: %x, Data: %s", msg->id, msg->data.c_str());
//   // 判断T是否为can_msgs::msg::Frame类型
//   if constexpr (std::is_same<T, can_msgs::msg::Frame>::value) {
//     RCLCPP_INFO(this->get_logger(), "Received can_msgs::msg::Frame with ID: %x, Data: %s", msg->id, msg->data.c_str());
//   } else if constexpr (std::is_same<T, ros2_socketcan_msgs::msg::FdFrame>::value) {
//     // 判断T是否为ros2_socketcan_msgs::msg::FdFrame类型
//     RCLCPP_INFO(this->get_logger(), "Received ros2_socketcan_msgs::msg::FdFrame with ID: %x, Data: %s", msg->id, msg->data.c_str());
//   } else { // 如果需要处理其他类型，可以继续扩展
//     RCLCPP_WARN(this->get_logger(), "Unknown message type.");
//   }

//   uint32_t id = msg->id;
//   switch (id)
//   {
//   case 0x500: // BO_ 1280 Throttle_Report: 8 VCU
//     /* code */
//     break;
  
//   default:
//     break;
//   }
//   FrameType type;
//   if (msg->is_rtr) {
//     type = FrameType::REMOTE;
//   } else if (msg->is_error) {
//     type = FrameType::ERROR;
//   } else {
//     type = FrameType::DATA;
//   }

// }
// void callbackFromVehicleCan(const std::shared_ptr<can_msgs::msg::Frame> &msg);
void FodInterface::callbackFromVehicleCan(const std::shared_ptr<can_msgs::msg::Frame> &msg) {
  // 处理CAN上报的消息
  std::ostringstream data_stream;
  for (const auto& byte : msg->data) {
      data_stream << std::hex << static_cast<int>(byte) << " ";
  }
  RCLCPP_INFO(this->get_logger(), "Received CAN frame with ID: %x, Data: %s", msg->id, data_stream.str().c_str());
  // RCLCPP_INFO(this->get_logger(), "Received CAN frame with ID: %x, Data: %s", msg->id, msg->data.c_str());

  auto parser_class = Fod3TxMsg::make_message(msg->id);
  if (parser_class != nullptr) {
    const std::vector<uint8_t> data_copy(msg->data.begin(), msg->data.end());
    parser_class->parse(data_copy);
    can_rpts_[msg->id] = std::make_pair(this->now(), parser_class);

    if (msg->id == VcuRptMsg::CAN_ID) {
      // auto dc_parser = std::dynamic_pointer_cast<VcuRptMsg>(parser_class);
      // auto enabled_msg = std::make_unique<std_msgs::msg::Bool>();
      // enabled_msg->data = dc_parser->enabled;
      // pub_enabled_->publish(std::move(enabled_msg));
      // if (dc_parser->override_active || dc_parser->system_fault_active) {
      //   set_enable(false);
      // }
    }
    callbackCanRptWrap();
  }
}

void FodInterface::callbackCanRptWrap() {
  // 判断是否需要发送CAN消息到autoware
  // 自上次上报autoware后，再次收集齐各个CAN上报消息时，上报autoware并记录上报时间。
  if (can_rpts_.size() < 5) {
    // 收集CAN消息数量不够，无需上报autoware
    return;
  }
  for (auto & rpt : can_rpts_) {
    auto rpt_time = rpt.second.first;
    if (rpt_time < can_rpts_time_) {
      // 该条CAN上报消息自上次上报autoware后，还未新消息到，不需上报到autoware
      return;
    }
  }
  // 每条消息都到达了，需要发送消息到autoware，设置发送时间。
  can_rpts_time_ = this->now();
  callbackCanRpt();
}

void FodInterface::callbackCanRpt()
{
  // 获取上报的消息，并转换类型
  auto rpt_throttle = std::dynamic_pointer_cast<ThrottleRptMsg>(can_rpts_[ThrottleRptMsg::CAN_ID].second);
  auto rpt_brake = std::dynamic_pointer_cast<BrakeRptMsg>(can_rpts_[BrakeRptMsg::CAN_ID].second);
  auto rpt_gear = std::dynamic_pointer_cast<GearRptMsg>(can_rpts_[GearRptMsg::CAN_ID].second);
  auto rpt_steering = std::dynamic_pointer_cast<SteeringRptMsg>(can_rpts_[SteeringRptMsg::CAN_ID].second);
  auto rpt_park = std::dynamic_pointer_cast<ParkRptMsg>(can_rpts_[ParkRptMsg::CAN_ID].second);
  auto rpt_vcu = std::dynamic_pointer_cast<VcuRptMsg>(can_rpts_[VcuRptMsg::CAN_ID].second);
  auto rpt_wheel_speed = std::dynamic_pointer_cast<WheelSpeedRptMsg>(can_rpts_[WheelSpeedRptMsg::CAN_ID].second);
  auto rpt_bms = std::dynamic_pointer_cast<BmsRptMsg>(can_rpts_[BmsRptMsg::CAN_ID].second);

  // 将CAN上报的消息转化成autoware内部消息，上报到autoware
  const double current_velocity = rpt_vcu->vehicle_speed;  // current vehicle speed > 0 [m/s]
  const double current_steer_wheel = degrees_to_radians(rpt_steering->angle_actual); // rad

  const double adaptive_gear_ratio =
    calculateVariableGearRatio(current_velocity, current_steer_wheel);
  const double current_steer = current_steer_wheel / adaptive_gear_ratio + steering_offset_;

  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = get_clock()->now();

  /* 发布方向盘状态 */
  {
    SteeringWheelStatusStamped steering_wheel_status_msg;
    steering_wheel_status_msg.stamp = header.stamp;
    steering_wheel_status_msg.data = current_steer_wheel; // rad
    steering_wheel_status_pub_->publish(steering_wheel_status_msg);
  }

  /* 发布车辆控制模式的状态 */
  {
    autoware_vehicle_msgs::msg::ControlModeReport control_mode_msg;
    control_mode_msg.stamp = header.stamp;
    if (rpt_vcu->vehicle_mode_state == 1) {
      control_mode_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    } else if (rpt_vcu->vehicle_mode_state == 0) {
      control_mode_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
    } else {
      control_mode_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::NOT_READY;
    }
    control_mode_pub_->publish(control_mode_msg);
  }

  /* 发布车辆状态中的速度信息 */
  {
    autoware_vehicle_msgs::msg::VelocityReport twist;
    twist.header = header;
    twist.longitudinal_velocity = current_velocity;                                 // [m/s]
    twist.heading_rate = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s]
    vehicle_twist_pub_->publish(twist);
  }

  /* 发布当前的换挡状态信息 */
  {
    autoware_vehicle_msgs::msg::GearReport gear_report_msg;
    gear_report_msg.stamp = header.stamp;
    const auto opt_gear_report = toAutowareShiftReport(static_cast<GearTarget>(rpt_gear->gear_actual));
    if (opt_gear_report) {
      gear_report_msg.report = *opt_gear_report;
      gear_status_pub_->publish(gear_report_msg);
    }
  }

  /* 发布当前的方向盘状态 */
  {
    autoware_vehicle_msgs::msg::SteeringReport steer_msg;
    steer_msg.stamp = header.stamp;
    steer_msg.steering_tire_angle = current_steer;
    steering_status_pub_->publish(steer_msg);
  }

  /* 发布控制状态信息 */
  {
    ActuationStatusStamped actuation_status;
    actuation_status.header = header;
    actuation_status.status.accel_status = rpt_vcu->vehicle_acc;
    actuation_status.status.brake_status = rpt_brake->pedal_actual;
    actuation_status.status.steer_status = current_steer;
    actuation_status_pub_->publish(actuation_status);
  }

  /* 发送转向信号和危险警告灯的状态报告 */
  {
    auto turn_light_actual = rpt_vcu->turn_light_actual;

    autoware_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
    turn_msg.stamp = header.stamp;
    turn_msg.report = toAutowareTurnIndicatorsReport(static_cast<FodTurnLight>(turn_light_actual));
    turn_indicators_status_pub_->publish(turn_msg);
    autoware_vehicle_msgs::msg::HazardLightsReport hazard_msg;
    hazard_msg.stamp = header.stamp;
    hazard_msg.report = toAutowareHazardLightsReport(static_cast<FodTurnLight>(turn_light_actual));
    hazard_lights_status_pub_->publish(hazard_msg);
  }
}

void FodInterface::callbackActuationCmd(const ActuationCommandStamped::ConstSharedPtr msg)
{
  actuation_command_received_time_ = this->now();
  actuation_cmd_ptr_ = msg;
  prepareCommands();
}

void FodInterface::callbackEmergencyCmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  is_emergency_ = msg->emergency;
  prepareCommands();
}

void FodInterface::callbackControlCmd(
  const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
{
  control_command_received_time_ = this->now();
  control_cmd_ptr_ = msg;
  prepareCommands();
}

void FodInterface::callbackGearCmd(
  const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  gear_cmd_ptr_ = msg;
  prepareCommands();
}

void FodInterface::callbackTurnIndicatorsCommand(
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  turn_indicators_cmd_ptr_ = msg;
  prepareCommands();
}

void FodInterface::callbackHazardLightsCommand(
  const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_cmd_ptr_ = msg;
  prepareCommands();
}

void FodInterface::onControlModeRequest(
  const ControlModeCommand::Request::SharedPtr request,
  const ControlModeCommand::Response::SharedPtr response)
{
  if (request->mode == ControlModeCommand::Request::AUTONOMOUS) {
    engage_cmd_ = true;
    is_clear_override_needed_ = true;
    response->success = true;
    return;
  }

  if (request->mode == ControlModeCommand::Request::MANUAL) {
    engage_cmd_ = false;
    is_clear_override_needed_ = true;
    response->success = true;
    return;
  }

  RCLCPP_ERROR(get_logger(), "unsupported control_mode!!");
  response->success = false;
  return;
}

void FodInterface::prepareCommands()
{
  /* guard */
  if (!actuation_cmd_ptr_ || !control_cmd_ptr_ || !is_pacmod_rpt_received_ || !gear_cmd_ptr_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "vehicle_cmd = %d, pacmod3_msgs = %d", actuation_cmd_ptr_ != nullptr,
      is_pacmod_rpt_received_);
    return;
  }

  const rclcpp::Time current_time = get_clock()->now();

  bool desired_park = false;
  if (gear_cmd_ptr_->command == autoware_vehicle_msgs::msg::GearCommand::PARK) {
    desired_park = true;
  }
  double desired_throttle = actuation_cmd_ptr_->actuation.accel_cmd + accel_pedal_offset_;
  double desired_brake = actuation_cmd_ptr_->actuation.brake_cmd + brake_pedal_offset_;
  if (actuation_cmd_ptr_->actuation.brake_cmd <= std::numeric_limits<double>::epsilon()) {
    desired_brake = 0.0;
  }
  // double desired_steer = actuation_cmd_ptr_->actuation.steer_cmd;

  /* check emergency and timeout */
  const double control_cmd_delta_time_ms =
    (current_time - control_command_received_time_).seconds() * 1000.0;
  const double actuation_cmd_delta_time_ms =
    (current_time - actuation_command_received_time_).seconds() * 1000.0;
  bool timeouted = false;
  const int t_out = command_timeout_ms_;
  if (t_out >= 0 && (control_cmd_delta_time_ms > t_out || actuation_cmd_delta_time_ms > t_out)) {
    timeouted = true;
  }
  /* check emergency and timeout */
  const bool emergency_brake_needed =
    (is_emergency_ && !use_external_emergency_brake_) || timeouted;
  if (emergency_brake_needed) {
    RCLCPP_ERROR(
      get_logger(), "Emergency Stopping, emergency = %d, timeouted = %d", is_emergency_, timeouted);
    desired_throttle = 0.0;
    desired_brake = emergency_brake_;
  }

  const double current_velocity = calculateVehicleVelocity();
  auto rpt_steering = std::dynamic_pointer_cast<SteeringRptMsg>(can_rpts_[SteeringRptMsg::CAN_ID].second);
  const double current_steer_wheel = degrees_to_radians(rpt_steering->angle_actual);
  // const double current_steer_wheel = steer_wheel_rpt_ptr_->output;

  /* calculate desired steering wheel */
  double adaptive_gear_ratio = calculateVariableGearRatio(current_velocity, current_steer_wheel);
  double desired_steer_wheel =
    (control_cmd_ptr_->lateral.steering_tire_angle - steering_offset_) * adaptive_gear_ratio;
  desired_steer_wheel =
    std::min(std::max(desired_steer_wheel, -max_steering_wheel_), max_steering_wheel_);

  /* check shift change */
  const double brake_for_shift_trans = 0.7;
  auto rpt_vcu = std::dynamic_pointer_cast<GearRptMsg>(can_rpts_[GearRptMsg::CAN_ID].second);
  auto desired_shift = rpt_vcu->gear_actual;
  // uint16_t desired_shift = gear_cmd_rpt_ptr_->output;
  if (std::fabs(current_velocity) < 0.1) {  // velocity is low -> the shift can be changed
    uint8_t shift_val_ = static_cast<uint8_t>(toFodShiftCmd(*gear_cmd_ptr_));
    if (shift_val_ != desired_shift) {  // need shift change.
      desired_throttle = 0.0;
      desired_brake = brake_for_shift_trans;  // set brake to change the shift
      desired_shift = shift_val_;
      RCLCPP_DEBUG(
        get_logger(), "Doing shift change. current = %d, desired = %d. set brake_cmd to %f",
        desired_shift, shift_val_, desired_brake);
    }
  }
  uint8_t turn_light_ctrl = static_cast<uint8_t>(toFodTurnCmdWithHazardRecover(*turn_indicators_cmd_ptr_, *hazard_lights_cmd_ptr_));

  ThrottleCmdMsg encoder_throttle;
  // encoder_throttle.encode(true, 0, std::max(0.0, std::min(desired_throttle, max_throttle_)), current_velocity);
  encoder_throttle.encode(true, std::max(0.0, std::min(desired_throttle, max_throttle_)), 0, current_velocity);
  BrakeCmdMsg encoder_brake;
  encoder_brake.encode(true, 0, 0, desired_brake);
  SteeringCmdMsg encoder_steering;
  encoder_steering.encode(true, 0, desired_steer_wheel);
  GearCmdMsg encoder_gear;
  encoder_gear.encode(true, desired_shift);
  ParkCmdMsg encoder_park;
  encoder_park.encode(true, desired_park);
  VehicleModeCmdMsg encoder_vehicle_mode;
  encoder_vehicle_mode.encode(0, true, 1, turn_light_ctrl, false, false); // 标准方向盘模式,自动驾驶,速度模式,转向灯,头灯关闭,禁用车辆识别码请求

  can_cmds_[ThrottleCmdMsg::CAN_ID].second->setData(std::move(encoder_throttle.data));
  can_cmds_[BrakeCmdMsg::CAN_ID].second->setData(std::move(encoder_brake.data));
  can_cmds_[SteeringCmdMsg::CAN_ID].second->setData(std::move(encoder_steering.data));
  can_cmds_[GearCmdMsg::CAN_ID].second->setData(std::move(encoder_gear.data));
  can_cmds_[ParkCmdMsg::CAN_ID].second->setData(std::move(encoder_park.data));
  can_cmds_[VehicleModeCmdMsg::CAN_ID].second->setData(std::move(encoder_vehicle_mode.data));
  can_cmds_[ThrottleCmdMsg::CAN_ID].first= this->now();
  can_cmds_[BrakeCmdMsg::CAN_ID].first= this->now();
  can_cmds_[SteeringCmdMsg::CAN_ID].first= this->now();
  can_cmds_[GearCmdMsg::CAN_ID].first= this->now();
  can_cmds_[ParkCmdMsg::CAN_ID].first= this->now();
  can_cmds_[VehicleModeCmdMsg::CAN_ID].first= this->now();
}

void FodInterface::publishCommands()
{
  // 发送底盘CAN命令
  for (auto & cmd : can_cmds_) {
    auto msg = std::make_unique<can_msgs::msg::Frame>();
    auto data = cmd.second.second->getData();

    msg->id = cmd.first;
    msg->is_rtr = false;
    msg->is_extended = false;
    msg->is_error = false;
    msg->dlc = data.size();
    std::move(data.begin(), data.end(), msg->data.begin());

    pub_can_rx_->publish(std::move(msg));

    std::this_thread::sleep_for(INTER_MSG_PAUSE);
  }
}

double FodInterface::calcSteerWheelRateCmd(const double gear_ratio)
{
  const auto current_vel = std::fabs(calculateVehicleVelocity());

  // send low steer rate at low speed
  if (current_vel < std::numeric_limits<double>::epsilon()) {
    return steering_wheel_rate_stopped_;
  } else if (current_vel < low_vel_thresh_) {
    return steering_wheel_rate_low_vel_;
  }

  if (!enable_steering_rate_control_) {
    return max_steering_wheel_rate_;
  }

  constexpr double margin = 1.5;
  const double rate = margin * control_cmd_ptr_->lateral.steering_tire_rotation_rate * gear_ratio;
  return std::min(std::max(std::fabs(rate), min_steering_wheel_rate_), max_steering_wheel_rate_);
}

// double FodInterface::calculateVehicleVelocity(
//   const pacmod3_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
//   const pacmod3_msgs::msg::SystemRptInt & shift_rpt)
// {
//   const double sign = (shift_rpt.output == pacmod3_msgs::msg::SystemRptInt::SHIFT_REVERSE) ? -1 : 1;
//   const double vel =
//     (wheel_speed_rpt.rear_left_wheel_speed + wheel_speed_rpt.rear_right_wheel_speed) * 0.5 *
//     tire_radius_;
//   return sign * vel;
// }

double FodInterface::calculateVehicleVelocity()
{
  auto rpt_vcu = std::dynamic_pointer_cast<VcuRptMsg>(can_rpts_[VcuRptMsg::CAN_ID].second);
  return rpt_vcu->vehicle_speed;
}

double FodInterface::calculateVariableGearRatio(const double vel, const double steer_wheel)
{
  return std::max(
    1e-5, vgr_coef_a_ + vgr_coef_b_ * vel * vel - vgr_coef_c_ * std::fabs(steer_wheel));
}

GearTarget FodInterface::toFodShiftCmd(const autoware_vehicle_msgs::msg::GearCommand & gear_cmd)
{
  if (gear_cmd.command == autoware_vehicle_msgs::msg::GearCommand::PARK) {
    return GearTarget::PARK;
  }
  if (gear_cmd.command == autoware_vehicle_msgs::msg::GearCommand::REVERSE) {
    return GearTarget::REVERSE;
  }
  if (gear_cmd.command == autoware_vehicle_msgs::msg::GearCommand::DRIVE) {
    return GearTarget::DRIVE;
  }
  if (gear_cmd.command == autoware_vehicle_msgs::msg::GearCommand::LOW) {
    return GearTarget::DRIVE;
  }
  if (gear_cmd.command == autoware_vehicle_msgs::msg::GearCommand::NEUTRAL) {
    return GearTarget::NEUTRAL;
  }

  return GearTarget::INVALID;
}

uint16_t FodInterface::getGearCmdForPreventChatter(uint16_t gear_command)
{
  // first time to change gear
  if (!last_time_to_change_gear_ptr_) {
    // send gear change command
    last_time_to_change_gear_ptr_ = std::make_shared<rclcpp::Time>(this->now());
    prev_gear_command_ = gear_command;
    return gear_command;
  }

  // no gear change
  if (gear_command == prev_gear_command_) {
    return gear_command;
  }

  const auto time_from_last_gear_change = (this->now() - *last_time_to_change_gear_ptr_).seconds();
  if (time_from_last_gear_change < margin_time_for_gear_change_) {
    // hold current gear
    RCLCPP_INFO_STREAM(get_logger(), "current_gear_command: " << static_cast<int>(gear_command));
    RCLCPP_INFO_STREAM(get_logger(), "prev_gear_command: " << static_cast<int>(prev_gear_command_));
    RCLCPP_INFO_STREAM(get_logger(), "send prev_gear_command for preventing gear-chattering");

    return prev_gear_command_;
  }
  // send gear change command
  last_time_to_change_gear_ptr_ = std::make_shared<rclcpp::Time>(this->now());
  prev_gear_command_ = gear_command;
  return gear_command;
}

std::optional<int32_t> FodInterface::toAutowareShiftReport(GearTarget gear_actual)
{
  using autoware_vehicle_msgs::msg::GearReport;

  if (gear_actual == GearTarget::PARK) {
    return GearReport::PARK;
  }
  if (gear_actual == GearTarget::REVERSE) {
    return GearReport::REVERSE;
  }
  if (gear_actual == GearTarget::NEUTRAL) {
    return GearReport::NEUTRAL;
  }
  if (gear_actual == GearTarget::DRIVE) {
    return GearReport::DRIVE;
  }
  
  return {};
}

FodTurnLight FodInterface::toFodTurnCmd(
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & turn,
  const autoware_vehicle_msgs::msg::HazardLightsCommand & hazard)
{
  using autoware_vehicle_msgs::msg::HazardLightsCommand;
  using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  using pacmod3::FodTurnLight;

  // NOTE: hazard lights command has a highest priority here.
  if (hazard.command == HazardLightsCommand::ENABLE) {
    return FodTurnLight::TURN_HAZARDS;
  }
  if (turn.command == TurnIndicatorsCommand::ENABLE_LEFT) {
    return FodTurnLight::TURN_LEFT;
  }
  if (turn.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
    return FodTurnLight::TURN_RIGHT;
  }
  return FodTurnLight::TURN_NONE;
}

FodTurnLight FodInterface::toFodTurnCmdWithHazardRecover(
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & turn,
  const autoware_vehicle_msgs::msg::HazardLightsCommand & hazard)
{
  // using pacmod3::FodTurnLight;
  auto rpt_vcu = std::dynamic_pointer_cast<VcuRptMsg>(can_rpts_[VcuRptMsg::CAN_ID].second);
  FodTurnLight hazard_cmd_val_ = hazard.command == 2 ? FodTurnLight::TURN_HAZARDS : FodTurnLight::TURN_NONE; // 0无, 1关闭, 2开启
  FodTurnLight hazard_status_ = static_cast<FodTurnLight>(rpt_vcu->turn_light_actual);

  if (!engage_cmd_ || hazard_cmd_val_ == hazard_status_) {
    last_shift_inout_matched_time_ = this->now();
    return toFodTurnCmd(turn, hazard);
  }

  if ((this->now() - last_shift_inout_matched_time_).seconds() < hazard_thresh_time_) {
    return toFodTurnCmd(turn, hazard);
  }

  // hazard recover mode
  if (hazard_recover_count_ > hazard_recover_cmd_num_) {
    last_shift_inout_matched_time_ = this->now();
    hazard_recover_count_ = 0;
  }
  hazard_recover_count_++;

  if (
    hazard_cmd_val_ != FodTurnLight::TURN_HAZARDS &&
    hazard_status_ == FodTurnLight::TURN_HAZARDS) {
    // publish hazard commands for turning off the hazard lights
    return FodTurnLight::TURN_HAZARDS;
  } else if (  // NOLINT
    hazard_cmd_val_ == FodTurnLight::TURN_HAZARDS &&
    hazard_status_ != FodTurnLight::TURN_HAZARDS) {
    // publish none commands for turning on the hazard lights
    return FodTurnLight::TURN_NONE;
  } else {
    // something wrong
    RCLCPP_ERROR_STREAM(
      get_logger(), "turn signal command and output do not match. "
                      << "COMMAND: " << static_cast<uint8_t>(hazard_cmd_val_)
                      << "; OUTPUT: " << static_cast<uint8_t>(hazard_status_));
    return toFodTurnCmd(turn, hazard);
  }
}

int32_t FodInterface::toAutowareTurnIndicatorsReport(FodTurnLight turn_light_actual)
{
  using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
  if (turn_light_actual == FodTurnLight::TURN_RIGHT) {
    return TurnIndicatorsReport::ENABLE_RIGHT;
  } else if (turn_light_actual == FodTurnLight::TURN_LEFT) {
    return TurnIndicatorsReport::ENABLE_LEFT;
  } else if (turn_light_actual == FodTurnLight::TURN_NONE) {
    return TurnIndicatorsReport::DISABLE;
  }
  return TurnIndicatorsReport::DISABLE;
}

int32_t FodInterface::toAutowareHazardLightsReport(FodTurnLight turn_light_actual)
{
  using autoware_vehicle_msgs::msg::HazardLightsReport;
  if (turn_light_actual == FodTurnLight::TURN_HAZARDS) {
    return HazardLightsReport::ENABLE;
  }
  return HazardLightsReport::DISABLE;
}

double FodInterface::steerWheelRateLimiter(
  const double current_steer_cmd, const double prev_steer_cmd,
  const rclcpp::Time & current_steer_time, const rclcpp::Time & prev_steer_time,
  const double steer_rate, const double current_steer_output, const bool engage)
{
  if (!engage) {
    // return current steer as steer command ( do not apply steer rate filter )
    return current_steer_output;
  }

  const double dsteer = current_steer_cmd - prev_steer_cmd;
  const double dt = std::max(0.0, (current_steer_time - prev_steer_time).seconds());
  const double max_dsteer = std::fabs(steer_rate) * dt;
  const double limited_steer_cmd =
    prev_steer_cmd + std::min(std::max(-max_dsteer, dsteer), max_dsteer);
  return limited_steer_cmd;
}

}