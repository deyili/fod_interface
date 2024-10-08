fod_interface，与pocman车底座通过CAN进行通信，与autoware通讯

## 过程
1. 初始化FodInterface
  + 解析params
  + 开始监听autoware下发的命令
  + 开始监听from_can_bus话题，接收CAN上报的车辆状态消息
  + 初始化发送往CAN的话题to_can_bus
  + 初始化发送往autoware的各种话题（转发CAN上报消息）
  + 初始化定时发送can命令的定时器

2. 监听底盘CAN上报的消息，并发送给autoware
  + FodInterface::callbackFromVehicleCan接收can消息
  + 解析消息后保存到车辆当前状态中
  + 判断消息是否满了，满了后上报。

3. 监听autoware发给底盘的命令，并处理后转发给CAN
  + 接收各种callback*回调
  + 调用prepareCommands，转化为CAN命令格式，存储在can_cmds_
  + 定时器定时调用publishCommands，发送命令到CAN。
  
## 模擬測試，模擬autoware發送的消息
### Control
ros2 topic pub --once /control/command/control_cmd autoware_control_msgs/msg/Control "{lateral: {steering_tire_angle: 0.1, steering_tire_rotation_rate: 0.01}, longitudinal: {velocity: 10.0, acceleration: 0.5}}"

### GearCommand
ros2 topic pub --once /control/command/gear_cmd autoware_vehicle_msgs/msg/GearCommand "{stamp: {sec: 0, nanosec: 0}, command: 1}"

### TurnIndicatorsCommand
ros2 topic pub --once /control/command/turn_indicators_cmd autoware_vehicle_msgs/msg/TurnIndicatorsCommand "{stamp: {sec: 0, nanosec: 0}, command: 1}"

### HazardLightsCommand
ros2 topic pub --once /control/command/hazard_lights_cmd autoware_vehicle_msgs/msg/HazardLightsCommand "{stamp: {sec: 0, nanosec: 0}, command: 1}"

### ActuationCommandStamped
ros2 topic pub --once /control/command/actuation_cmd tier4_vehicle_msgs/msg/ActuationCommandStamped "{header: {stamp: {sec: 0, nanosec: 0}}, actuation: { accel_cmd: 1.0, brake_cmd: 0.0, steer_cmd: 0.0}}"

### VehicleEmergencyStamped
ros2 topic pub --once /control/command/emergency_cmd tier4_vehicle_msgs/msg/VehicleEmergencyStamped "{stamp: {sec: 0, nanosec: 0}, emergency: true}"

## gdb调试

colcon build --packages-select fod_interface --cmake-args -DCMAKE_BUILD_TYPE=Debug

gdb --args ~/dev/autoware/install/fod_interface/lib/fod_interface/fod_interface --ros-args  --params-file /home/guo/dev/autoware/install/fod_interface/share/fod_interface/config/fod.param.yaml --params-file /home/guo/dev/autoware/install/autoware_vehicle_info_utils/share/autoware_vehicle_info_utils/config/vehicle_info.param.yaml -r to_can_bus:=/canalystii/to_can_bus -r from_can_bus:=/canalystii/from_can_bus -r input/control_mode_request:=/control/control_mode_request

