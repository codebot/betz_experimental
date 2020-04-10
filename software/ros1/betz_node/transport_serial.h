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

#ifndef TRANSPORT_SERIAL_H
#define TRANSPORT_SERIAL_H

#include <memory>

#include "transport.h"
#include "lightweightserial.h"

namespace betz {

class TransportSerial : public Transport
{
public:
  TransportSerial();

  bool send(const uint8_t *data, const uint32_t len) override;
  int recv_nonblocking(uint8_t *data, const uint32_t max_len) override;

  bool open_device(const std::string& device_name);

  std::unique_ptr<LightweightSerial> serial;
};

}  // namespace betz

#endif
