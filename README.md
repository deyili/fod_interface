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
ros2 topic pub --once /control/command/control_cmd autoware_control_msgs/msg/Control "{lateral: {steering_tire_angle: 0.1, steering_tire_rotation_rate: 0.01}, longitudinal: {velocity: 1.0, acceleration: 0.5}}"

### GearCommand
+ DRIVE
ros2 topic pub --once /control/command/gear_cmd autoware_vehicle_msgs/msg/GearCommand "{stamp: {sec: 0, nanosec: 0}, command: 2}"

+ 空挡
ros2 topic pub --once /control/command/gear_cmd autoware_vehicle_msgs/msg/GearCommand "{stamp: {sec: 0, nanosec: 0}, command: 1}"

+ 驻车
ros2 topic pub --once /control/command/gear_cmd autoware_vehicle_msgs/msg/GearCommand "{stamp: {sec: 0, nanosec: 0}, command: 22}"

### TurnIndicatorsCommand
ros2 topic pub --once /control/command/turn_indicators_cmd autoware_vehicle_msgs/msg/TurnIndicatorsCommand "{stamp: {sec: 0, nanosec: 0}, command: 1}"

### HazardLightsCommand (1開0關)
ros2 topic pub --once /control/command/hazard_lights_cmd autoware_vehicle_msgs/msg/HazardLightsCommand "{stamp: {sec: 0, nanosec: 0}, command: 0}"

### ActuationCommandStamped
ros2 topic pub --once /control/command/actuation_cmd tier4_vehicle_msgs/msg/ActuationCommandStamped "{header: {stamp: {sec: 0, nanosec: 0}}, actuation: { accel_cmd: 1.0, brake_cmd: 0.0, steer_cmd: 0.0}}"

### VehicleEmergencyStamped
ros2 topic pub --once /control/command/emergency_cmd tier4_vehicle_msgs/msg/VehicleEmergencyStamped "{stamp: {sec: 0, nanosec: 0}, emergency: true}"

### 啓動時發送命令列表
ros2 topic pub --once /control/command/gear_cmd autoware_vehicle_msgs/msg/GearCommand "{stamp: {sec: 0, nanosec: 0}, command: 2}"
ros2 topic pub --once /control/command/turn_indicators_cmd autoware_vehicle_msgs/msg/TurnIndicatorsCommand "{stamp: {sec: 0, nanosec: 0}, command: 2}"
ros2 topic pub --once /control/command/hazard_lights_cmd autoware_vehicle_msgs/msg/HazardLightsCommand "{stamp: {sec: 0, nanosec: 0}, command: 1}"
ros2 topic pub --once /control/command/actuation_cmd tier4_vehicle_msgs/msg/ActuationCommandStamped "{header: {stamp: {sec: 0, nanosec: 0}}, actuation: { accel_cmd: 0.0, brake_cmd: 0.0, steer_cmd: 0.0}}"
ros2 topic pub --once /control/command/emergency_cmd tier4_vehicle_msgs/msg/VehicleEmergencyStamped "{stamp: {sec: 0, nanosec: 0}, emergency: true}"
ros2 topic pub --once /control/command/control_cmd autoware_control_msgs/msg/Control "{lateral: {steering_tire_angle: 1.0, steering_tire_rotation_rate: 0.5}, longitudinal: {velocity: 0.3, acceleration: 0.5}}"

## gdb调试

colcon build --packages-select fod_interface --cmake-args -DCMAKE_BUILD_TYPE=Debug

gdb --args ~/dev/autoware/install/fod_interface/lib/fod_interface/fod_interface --ros-args  --params-file /home/guo/dev/autoware/install/fod_interface/share/fod_interface/config/fod.param.yaml --params-file /home/guo/dev/autoware/install/autoware_vehicle_info_utils/share/autoware_vehicle_info_utils/config/vehicle_info.param.yaml -r to_can_bus:=/canalystii/to_can_bus -r from_can_bus:=/canalystii/from_can_bus -r input/control_mode_request:=/control/control_mode_request

## 測試命令值
01 xx 00 00 00 xx xx 00     m100/256  加速度，目标速度
01 00 00 xx xx 00 00 00     m101/257  剎車值：01 2C    
01 00 00 xx xx 00 00 00     m102/258  轉向角度
01 xx 00 00 00 00 00 00     m103/259  檔位：04 D, 02 R, 03 N
01 xx 00 00 00 00 00 00     m104/260  駐車：00 駐車釋放， 01駐車
xx xx xx 00 00 00 00 00     m105/261  字節0：80自动驾驶模式，01前後異向；字節1：01速度模式；字節2轉向燈：00關閉，01左轉，02右轉

## 目前发下去的值
01 00 00 00 00 00 00 00     m100/256  目标速度：0
01 00 00 00 00 00 00 00     m101/257  剎車值：0
01 00 00 01 f4 00 00 00     m102/258  轉向角度： 500 -> 0
01 04 00 00 00 00 00 00     m103/259  檔位：04 D
01 00 00 00 00 00 00 00     m104/260  駐車：00 駐車釋放
80 01 01 00 00 00 00 00     m105/261  字節0：80自动驾驶模式；字節1：01速度模式；字節2轉向燈：01左轉

01 0a 00 00 00 3e 80 00     m100/256  目标速度：10，加速度0.1
01 0a 00 00 00 06 40 00     m100/256  目标速度：1，加速度0.4

## 监听测试topic

ros2 topic echo --once /vehicle/status/rpt500 &&
ros2 topic echo --once /vehicle/status/rpt501 &&
ros2 topic echo --once /vehicle/status/rpt502 &&
ros2 topic echo --once /vehicle/status/rpt503 &&
ros2 topic echo --once /vehicle/status/rpt504 &&
ros2 topic echo --once /vehicle/status/rpt505 &&
ros2 topic echo --once /vehicle/status/rpt506 &&
ros2 topic echo --once /vehicle/status/rpt512



ros2 topic echo /canalystii/from_can_bus | head -n 200
