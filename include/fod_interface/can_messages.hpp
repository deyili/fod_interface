// Copyright (c) 2019 AutonomouStuff, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef FOD_INTERFACE__CAN_MESSAGES_HPP_
#define FOD_INTERFACE__CAN_MESSAGES_HPP_

#include <cstring>
#include <sstream>
#include <cstdint>
#include <memory>
#include <vector>
#include <string>

namespace pacmod3
{
enum class FodTurnLight
{
  TURN_NONE = 0,
  TURN_LEFT = 1,
  TURN_RIGHT = 2,
  TURN_HAZARDS = 3
};
enum class GearTarget
{
  INVALID = 0,
  PARK = 1,
  REVERSE = 2,
  NEUTRAL = 3,
  DRIVE = 4
};

// 基类：接收autoware发过来的命令，转化成CAN数据.
class Fod3RxMsg
{
public:
  std::vector<uint8_t> data;
};

// 基类：接收CAN上报的数据，转化成内部消息格式
class Fod3TxMsg
{
public:
  static std::shared_ptr<Fod3TxMsg> make_message(const uint32_t & can_id);
  virtual void parse(const std::vector<uint8_t> & in) = 0;
  virtual std::string toString() = 0;
};

// 下发CAN的命令生成
class ThrottleCmdMsg : public Fod3RxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x100;
  static constexpr uint8_t DATA_LENGTH = 8;

  void encode(
    bool enctrl, // 0|1@0+ (1,0) [0|1] "" Vector__XXX
    float acc, // 15|10@0+ (0.01,0) [0|10] "m/s^2" Vector__XXX
    float throttle_pedal_target, // 31|16@0+ (0.1,0) [0|100] "%" Vector__XXX
    float speed_target // 47|12@0+ (0.01,0) [0|40.95] "m/s" Vector__XXX
    );
};

class BrakeCmdMsg : public Fod3RxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x101;
  static constexpr uint8_t DATA_LENGTH = 8;

  void encode(
    bool enctrl, // 0|1@0+ (1,0) [0|1] "" Vector__XXX
    bool aeb_enctrl, // 1|1@0+ (1,0) [0|0] "" Vector__XXX
    float dec, // 15|10@0+ (0.01,0) [0|10] "m/s^2" Vector__XXX
    float brake_pedal_target // 31|16@0+ (0.1,0) [0|100] "%" Vector__XXX
    );
};

class SteeringCmdMsg : public Fod3RxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x102;
  static constexpr uint8_t DATA_LENGTH = 8;

  void encode(
    bool enctrl, // 0|1@0+ (1,0) [0|1] "" Vector__XXX
    float angle_speed, // 15|8@0+ (1,0) [0|250] "deg/s" Vector__XXX
    float angle_target // 31|16@0+ (1,-500) [-500|500] "deg" Vector__XXX
    );
};

class GearCmdMsg : public Fod3RxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x103;
  static constexpr uint8_t DATA_LENGTH = 8;

  void encode(
    bool enctrl, // 0|1@0+ (1,0) [0|1] "" Vector__XXX
    uint8_t target // 10|3@0+ (1,0) [0|4] "" Vector__XXX
    );
};

class ParkCmdMsg : public Fod3RxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x104;
  static constexpr uint8_t DATA_LENGTH = 8;

  void encode(
    bool enctrl, // 0|1@0+ (1,0) [0|1] "" Vector__XXX
    bool target // 8|1@0+ (1,0) [0|1] "" Vector__XXX
    );
};

class VehicleModeCmdMsg : public Fod3RxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x105;
  static constexpr uint8_t DATA_LENGTH = 8;

  void encode(
    uint8_t steer_mode_ctrl, // 2|3@0+ (1,0) [0|7] "" Vector__XXX
    bool auto_professional, // 7|1@0+ (1,0) [0|1] "" Vector__XXX
    uint8_t drive_mode_ctrl, // 10|3@0+ (1,0) [0|7] "" Vector__XXX
    uint8_t turn_light_ctrl, // 17|2@0+ (1,0) [0|7] "" Vector__XXX
    bool head_light_ctrl, // 18|1@0+ (1,0) [0|0] "" Vector__XXX
    bool vehicle_vin_req // 24|1@0+ (1,0) [0|1] "" Vector__XXX
    );
};

// CAN上报消息的结构
class ThrottleRptMsg : public Fod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x500;

  uint8_t enstate; // 1|2@0+ (1,0) [0|2] ""  ACU
  uint8_t flt1; // 15|8@0+ (1,0) [0|1] ""  ACU
  uint8_t flt2; // 23|8@0+ (1,0) [0|1] ""  ACU
  float throttle_pedal_actual; // 31|16@0+ (0.1,0) [0|100] "%"  ACU

  void parse(const std::vector<uint8_t> & in);
  std::string toString();
};

class BrakeRptMsg : public Fod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x501;

  uint8_t enstate; // 1|2@0+ (1,0) [0|2] ""  ACU
  uint8_t flt1; // 15|8@0+ (1,0) [0|1] ""  ACU
  uint8_t flt2; // 23|8@0+ (1,0) [0|1] ""  ACU
  float brake_pedal_actual; // 31|16@0+ (0.1,0) [0|100] "%"  ACU

  void parse(const std::vector<uint8_t> & in);
  std::string toString();
};

class SteeringRptMsg : public Fod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x502;

  uint8_t enstate; // 1|2@0+ (1,0) [0|2] ""  ACU
  uint8_t flt1; // 15|8@0+ (1,0) [0|255] ""  ACU
  uint8_t flt2; // 23|8@0+ (1,0) [0|255] ""  ACU
  float angle_actual; // 31|16@0+ (1,-500) [-500|500] "deg"  ACU
  float angle_rear_actual; // 47|16@0+ (1,-500) [-500|500] "deg" Vector__XXX
  float angle_speed_actual; // 63|8@0+ (1,0) [0|250] "deg/s"  ACU

  void parse(const std::vector<uint8_t> & in);
  std::string toString();
};

class GearRptMsg : public Fod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x503;

  uint8_t gear_actual; // 2|3@0+ (1,0) [0|4] ""  ACU
  uint8_t gear_flt; // 15|8@0+ (1,0) [0|1] ""  ACU

  void parse(const std::vector<uint8_t> & in);
  std::string toString();
};

class ParkRptMsg : public Fod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x504;

  bool parking_actual; // 0|1@0+ (1,0) [0|1] ""  ACU
  uint8_t park_flt; // 15|8@0+ (1,0) [0|1] ""  ACU

  void parse(const std::vector<uint8_t> & in);
  std::string toString();
};

class VcuRptMsg : public Fod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x505;

  float vehicle_acc; // 7|12@0- (0.01,0) [-10|10] "m/s^2"  ACU
  uint8_t steer_mode_status; // 10|3@0+ (1,0) [0|7] "" Vector__XXX
  bool brake_light_actual; // 11|1@0+ (1,0) [0|1] "" Vector__XXX
  float vehicle_speed; // 23|16@0- (0.001,0) [-32.768|32.767] "m/s"  ACU
  bool aeb_brake_state; // 32|1@0+ (1,0) [0|0] "" Vector__XXX
  bool vehicle_front_crash_state; // 33|1@0+ (1,0) [0|0] "" Vector__XXX
  bool back_crash_state; // 34|1@0+ (1,0) [0|0] "" Vector__XXX
  uint8_t vehicle_mode_state; // 36|2@0+ (1,0) [0|0] "" Vector__XXX
  uint8_t drive_mode_status; // 39|3@0+ (1,0) [0|7] "" Vector__XXX
  uint8_t vehicle_errcode; // 47|8@0+ (1,0) [0|255] "" Vector__XXX
  uint8_t car_work_state; // 51|4@0+ (1,0) [0|0] "" Vector__XXX
  uint8_t car_power_state; // 53|2@0+ (1,0) [0|0] "" Vector__XXX
  bool auto_professional_fb; // 55|1@0+ (1,0) [0|1] "" Vector__XXX
  uint8_t turn_light_actual; // 57|2@0+ (1,0) [0|0] "" Vector__XXX
  bool aeb_trigger_state; // 58|1@0+ (1,0) [0|1] "" Vector__XXX
  bool headlight_actual; // 59|1@0+ (1,0) [0|0] "" Vector__XXX

  void parse(const std::vector<uint8_t> & in);
  std::string toString();
};

class WheelSpeedRptMsg : public Fod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x506;

  float fl; // 7|16@0+ (0.001,0) [0|65.535] "m/s"  ACU
  float fr; // 23|16@0+ (0.001,0) [0|65.535] "m/s"  ACU
  float rl; // 39|16@0+ (0.001,0) [0|65.535] "m/s"  ACU
  float rr; // 55|16@0+ (0.001,0) [0|65.535] "m/s"  ACU
  void parse(const std::vector<uint8_t> & in);
  std::string toString();
};

class BmsRptMsg : public Fod3TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x512;

  float voltage; // 7|16@0+ (0.01,0) [0|300] "V" Vector__XXX
  float current; // 23|16@0+ (0.1,-3200) [-3200|3353.5] "A" Vector__XXX
  uint8_t soc; // 39|8@0+ (1,0) [0|100] "%" Vector__XXX
  float leadacid_voltage; // 55|8@0+ (0.1,0) [0|0] "V" Vector__XXX
  void parse(const std::vector<uint8_t> & in);
  std::string toString();
};

}  // namespace pacmod3

#endif  // FOD_INTERFACE__CAN_MESSAGES_HPP_
