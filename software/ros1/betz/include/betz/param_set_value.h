/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef PARAM_SET_VALUE_H
#define PARAM_SET_VALUE_H

#include <stdint.h>
#include "betz/drive.h"
#include "betz/packet.h"

namespace betz {

class ParamSetValue : public Packet
{
public:
  ParamSetValue(
      const Drive& drive,
      const Param& param,
      const bool force_long_addr = false);
};

}  // namespace betz

#endif
