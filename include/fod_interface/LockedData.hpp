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

#ifndef FOD_INTERFACE__LOCKED_DATA_HPP_
#define FOD_INTERFACE__LOCKED_DATA_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace pacmod3
{

class LockedData
{
public:
  explicit LockedData(unsigned char data_length): _data(), _data_mut() {
    _data.assign(data_length, 0);
  };

  std::vector<unsigned char> getData() const {
    std::lock_guard<std::mutex> lck(_data_mut);
    return _data;
  };
  void setData(std::vector<unsigned char> && new_data){
    std::lock_guard<std::mutex> lck(_data_mut);
    _data = new_data;
  };

private:
  std::vector<unsigned char> _data;
  mutable std::mutex _data_mut;
};

}  // namespace pacmod3

#endif  // FOD_INTERFACE__LOCKED_DATA_HPP_
