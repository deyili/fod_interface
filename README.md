fod_interface

1. 与pocman车底座通过CAN进行通信
2. 监听autoware发给底盘的消息，并发送给CAN
3. 监听底盘CAN上报的消息，并发送给autoware
  + FodInterface::callbackFromVehicleCanX接收can消息，解析后保存到车辆当前状态中，并上报。