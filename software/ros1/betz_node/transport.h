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

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <stdint.h>

namespace betz {

class Transport
{
public:
  virtual bool send(const uint8_t *data, const uint32_t len) = 0;
  virtual int recv_nonblocking(uint8_t *data, const uint32_t max_len) = 0;
};

}  // namespace betz

#endif
