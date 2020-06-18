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

#include "betz/uuid.h"
using betz::UUID;
using std::string;

UUID::UUID()
{
}

UUID::UUID(const std::vector<uint8_t>& _bytes)
{
  bytes.resize(_bytes.size());
  s = string();
  for (size_t i = 0; i < _bytes.size(); i++)
  {
    bytes[i] = _bytes[i];

    char byte_buf[10] = {0};
    snprintf(
        byte_buf,
        sizeof(byte_buf),
        "%02x",
        static_cast<unsigned>(_bytes[i]));
    s += string(byte_buf);
    if (i % 4 == 3 && i != 11)
      s += ":";
  }
}

bool UUID::operator==(const UUID& rhs)
{
  for (size_t i = 0; i < rhs.bytes.size() && i < bytes.size(); i++)
    if (bytes[i] != rhs.bytes[i])
      return false;
  return true;
}

UUID& UUID::operator=(const UUID& rhs)
{
  bytes = rhs.bytes;
  s = rhs.s;
}

void UUID::clear()
{
  bytes.clear();
  s.clear();
}

bool UUID::is_valid() const
{
  return bytes.size() == UUID_LEN;
}

string UUID::generate_string()
{
  if (!s.empty())
    return s;

  for (size_t i = 0; i < bytes.size(); i++)
  {
    char byte_buf[10] = {0};
    snprintf(
        byte_buf,
        sizeof(byte_buf),
        "%02x",
        static_cast<unsigned>(bytes[i]));
    s += string(byte_buf);
    if (i % 4 == 3 && i != 11)
      s += ":";
  }
  return s;
}
