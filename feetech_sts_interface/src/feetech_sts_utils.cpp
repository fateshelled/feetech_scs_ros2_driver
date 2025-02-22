// Copyright 2023 fateshelled.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "feetech_sts_interface/feetech_sts_utils.hpp"
#include <iostream>

namespace feetech_sts_interface
{

float STS3032::data2angle(int32_t data)
{
  return ((float)data * 360 / STS3032_ANGLE_360);
}

int32_t STS3032::angle2data(const float angle)
{
  return (int)(angle * STS3032_ANGLE_360 / 360);
}

}  // namespace feetech_sts_interface
