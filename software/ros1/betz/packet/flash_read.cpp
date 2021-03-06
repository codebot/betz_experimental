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

#include "betz/flash_read.h"
using betz::FlashRead;
using betz::Packet;

FlashRead::FlashRead(
    const Drive& drive,
    const uint32_t addr,
    const uint32_t len)
{
  flags = FLAG_SENTINEL | FLAG_ADDR_UUID;
  uuid = drive.uuid;

  payload.push_back(ID_FLASH_READ);
  append(addr);
  append(len);
}
