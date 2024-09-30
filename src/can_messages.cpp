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

#include <memory>
#include <vector>

#include "fod_interface/can_messages.hpp"
#include "fod_interface/utils.hpp"

namespace pacmod3
{

// System Commands
constexpr uint32_t ThrottleCmdMsg::CAN_ID;
constexpr uint32_t BrakeCmdMsg::CAN_ID;
constexpr uint32_t SteeringCmdMsg::CAN_ID;
constexpr uint32_t GearCmdMsg::CAN_ID;
constexpr uint32_t ParkCmdMsg::CAN_ID;
constexpr uint32_t VehicleModeCmdMsg::CAN_ID;

// System Reports
constexpr uint32_t ThrottleRptMsg::CAN_ID;
constexpr uint32_t BrakeRptMsg::CAN_ID;
constexpr uint32_t SteeringRptMsg::CAN_ID;
constexpr uint32_t GearRptMsg::CAN_ID;
constexpr uint32_t ParkRptMsg::CAN_ID;
constexpr uint32_t VcuRptMsg::CAN_ID;
constexpr uint32_t WheelSpeedRptMsg::CAN_ID;
constexpr uint32_t BmsRptMsg::CAN_ID;

std::shared_ptr<Fod3TxMsg> Fod3TxMsg::make_message(const uint32_t & can_id)
{
  switch (can_id) {
    case ThrottleRptMsg::CAN_ID:
      return std::shared_ptr<Fod3TxMsg>(new ThrottleRptMsg);
      break;
    case BrakeRptMsg::CAN_ID:
      return std::shared_ptr<Fod3TxMsg>(new BrakeRptMsg);
      break;
    case SteeringRptMsg::CAN_ID:
      return std::shared_ptr<Fod3TxMsg>(new SteeringRptMsg);
      break;
    case GearRptMsg::CAN_ID:
      return std::shared_ptr<Fod3TxMsg>(new GearRptMsg);
      break;
    case ParkRptMsg::CAN_ID:
      return std::shared_ptr<Fod3TxMsg>(new ParkRptMsg);
      break;
    case VcuRptMsg::CAN_ID:
      return std::shared_ptr<Fod3TxMsg>(new VcuRptMsg);
      break;
    case WheelSpeedRptMsg::CAN_ID:
      return std::shared_ptr<Fod3TxMsg>(new WheelSpeedRptMsg);
      break;
    case BmsRptMsg::CAN_ID:
      return std::shared_ptr<Fod3TxMsg>(new BmsRptMsg);
      break;
    default:
      return nullptr;
  }
}

// CAN上报的消息
void ThrottleRptMsg::parse(const std::vector<uint8_t> & in)
{
  enstate = in[0] & 0x03;
  flt1 = in[1];
  flt2 = in[2];
  pedal_actual = ((static_cast<int16_t>(in[3]) << 8) | in[4]) / 10.0;
}

void BrakeRptMsg::parse(const std::vector<uint8_t> & in)
{
  enstate = in[0] & 0x03;
  flt1 = in[1];
  flt2 = in[2];
  pedal_actual = ((static_cast<int16_t>(in[3]) << 8) | in[4]) / 10.0;
}

void SteeringRptMsg::parse(const std::vector<uint8_t> & in)
{
  enstate = in[0] & 0x03;
  flt1 = in[1];
  flt2 = in[2];
  angle_actual = ((static_cast<int16_t>(in[3]) << 8) | in[4]) / 10.0;
  angle_rear_actual = ((static_cast<int16_t>(in[5]) << 8) | in[6]) / 10.0;
  angle_speed_actual = in[7];
}

void GearRptMsg::parse(const std::vector<uint8_t> & in)
{
  gear_actual = in[0] & 0x07;
  gear_flt = in[1];
}

void ParkRptMsg::parse(const std::vector<uint8_t> & in)
{
  parking_actual = ((in[0] & 0x01) > 0);
  park_flt = in[1];
}

void VcuRptMsg::parse(const std::vector<uint8_t> & in)
{
  vehicle_acc = ((static_cast<int16_t>(in[0]) << 4) | ((in[1] & 0xF0) >> 4) ) / 10.0;
  steer_mode_status = in[1] & 0x07;
  brake_light_actual = ((in[1] & 0x80) > 0);
  vehicle_speed = ((static_cast<int16_t>(in[2]) << 8) | in[3]) / 1000.0;
  aeb_brake_state = ((in[4] & 0x01) > 0);
  vehicle_front_crash_state = ((in[4] & 0x02) > 0);
  back_crash_state = ((in[4] & 0x04) > 0);
  vehicle_mode_state = ((in[4] & 0x18) >> 3);
  drive_mode_status = ((in[4] & 0x18) >> 5);
  vehicle_errcode = in[5];
  car_work_state = in[6] & 0x0F;
  car_power_state = (in[6] & 0x30) >> 4;
  auto_professional_fb = ((in[6] & 0x80) > 0);
  turn_light_actual = in[7] & 0x03;
  aeb_trigger_state = ((in[7] & 0x04) > 0);
  headlight_actual = ((in[7] & 0x08) > 0);
}

void WheelSpeedRptMsg::parse(const std::vector<uint8_t> & in)
{
  fl = ((static_cast<int16_t>(in[0]) << 8) | in[1]) / 1000.0;
  fr = ((static_cast<int16_t>(in[2]) << 8) | in[3]) / 1000.0;
  rl = ((static_cast<int16_t>(in[4]) << 8) | in[5]) / 1000.0;
  rr = ((static_cast<int16_t>(in[6]) << 8) | in[7]) / 1000.0;
}

void BmsRptMsg::parse(const std::vector<uint8_t> & in)
{
  voltage = ((static_cast<int16_t>(in[0]) << 8) | in[1]) / 100.0;
  current = ((static_cast<int16_t>(in[2]) << 8) | in[3]) / 10.0;
  soc = in[4];
  leadacid_voltage = static_cast<float>(in[6]) / 10.0;
}

// RX Messages
void ThrottleCmdMsg::encode(
  bool enctrl,
  float acc,
  float throttle_pedal_target,
  float speed_target)
{
  data.assign(DATA_LENGTH, 0);
  uint16_t acc_u16 = static_cast<uint16_t>(acc * 100.0);
  uint16_t throttle_pedal_target_u16 = static_cast<uint16_t>(throttle_pedal_target * 10.0);
  uint16_t speed_target_u16 = static_cast<uint16_t>(speed_target * 100.0);

  data[0] = (enctrl ? 0x01 : 0x00);
  data[1] = (acc_u16 & 0x3FC) >> 2;
  data[2] = (acc_u16 & 0x03) << 6;
  data[3] = (throttle_pedal_target_u16 & 0xFF00) >> 8;
  data[4] = throttle_pedal_target_u16 & 0xFF;
  data[5] = (speed_target_u16 & 0xFF0) >> 4;
  data[6] = (speed_target_u16 & 0x0F) << 4;
}

void BrakeCmdMsg::encode(
  bool enctrl,
  bool aeb_enctrl,
  float dec,
  float brake_pedal_target)
{
  data.assign(DATA_LENGTH, 0);
  uint16_t dec_u16 = static_cast<uint16_t>(dec * 100.0);
  uint16_t brake_pedal_target_u16 = static_cast<uint16_t>(brake_pedal_target * 10.0);

  data[0] = enctrl ? 0x01 : 0x00;
  data[0] |= aeb_enctrl ? 0x02 : 0x00;
  data[1] = (dec_u16 & 0x3FC) >> 2;
  data[2] = (dec_u16 & 0x03) << 6;
  data[3] = (brake_pedal_target_u16 & 0xFF00) >> 8;
  data[4] = brake_pedal_target_u16 & 0xFF;
}

void SteeringCmdMsg::encode(
  bool enctrl,
  float angle_speed,
  float angle_target)
{
  data.assign(DATA_LENGTH, 0);

  data[0] = enctrl ? 0x01 : 0x00;
  data[1] = static_cast<uint8_t>(radians_to_degrees(angle_speed));
  uint16_t angle_target_u16 = static_cast<uint16_t>(radians_to_degrees(angle_target) + 500);
  data[3] = (angle_target_u16 & 0xFF00) >> 8;
  data[4] = angle_target_u16 & 0x0FF;
}

void GearCmdMsg::encode(
  bool enctrl,
  uint8_t target)
{
  data.assign(DATA_LENGTH, 0);

  data[0] = enctrl ? 0x01 : 0x00;
  data[1] = target & 0x07;
}

void ParkCmdMsg::encode(
  bool enctrl,
  bool target)
{
  data.assign(DATA_LENGTH, 0);

  data[0] = enctrl ? 0x01 : 0x00;
  data[1] = target ? 0x01 : 0x00;
}

void VehicleModeCmdMsg::encode(
  uint8_t steer_mode_ctrl,
  bool auto_professional,
  uint8_t drive_mode_ctrl,
  uint8_t turn_light_ctrl,
  bool head_light_ctrl,
  bool vehicle_vin_req)
{
  data.assign(DATA_LENGTH, 0);

  data[0] = steer_mode_ctrl & 0x07;
  data[0] |= auto_professional ? 0x80 : 0x00;
  data[1] = drive_mode_ctrl & 0x07;
  data[2] = turn_light_ctrl & 0x03;
  data[2] |= head_light_ctrl ? 0x04 : 0x00;
  data[3] = vehicle_vin_req ? 0x01 : 0x00;
}

}  // namespace pacmod3
