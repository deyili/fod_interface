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

#ifndef FOD_INTERFACE__UTILS_HPP_
#define FOD_INTERFACE__UTILS_HPP_

#include <cmath>

template <typename VT>
inline double degrees_to_radians(VT degrees) {
  return degrees * M_PI / 180.0;
}

template <typename VT>
inline double radians_to_degrees(VT radians) {
  return radians * 180.0 / M_PI;
}

template <typename Container>
inline std::string toHexString(const Container& data) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');  // 设置填充字符为 '0'
    for (const auto& byte : data) {
        ss << std::setw(2) << static_cast<int>(byte);  // 输出每个字节
    }
    return ss.str();
}

// // 对最多16位有符号数进行符号扩展,bitCount表示有效位数.
// int16_t sign16ExtendNBit(uint16_t value, uint8_t bitCount) {
//   uint16_t mask = bitCount > 0 ? 1 << (bitCount - 1) : 0;
//   if (value & mask) { // 检查符号位（第bitCount位）
//     uint8_t pad1Count = 16 - bitCount;
//     return value | 0xF000; // 符号扩展，将高4位设为1
//   }
//   return value;
// }
// 对 12 位有符号数进行符号扩展
inline int16_t signExtend12Bit(uint16_t value) {
  if (value & 0x0800) { // 检查符号位（第12位）
    return value | 0xF000; // 符号扩展，将高4位设为1
  }
  return value;
}

template <typename VT>
inline uint32_t encodeValue(VT value, float scaleFactor, float offset = 0) {
  // 将下发的数据转化成CAN格式
  float val = (static_cast<float>(value) - offset)/ scaleFactor;
  // int val = value / scaleFactor - offset;
  return static_cast<uint32_t>(val);
}

template <typename VT>
inline float decodeValue(VT value, float scaleFactor, float offset = 0) {
  // 将上报的数据转化成实际数据。
  return static_cast<float>(value) * scaleFactor + offset;
  // return (value + offset) * scaleFactor;
}

#endif  // FOD_INTERFACE__UTILS_HPP_
