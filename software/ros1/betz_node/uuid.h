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

#ifndef BETZ_UUID_H
#define BETZ_UUID_H

// just a utility function to convert UUID's to string
#include <string>
#include <vector>
#include <cstdint>

namespace betz {

class UUID
{
public:
  UUID();
  UUID(const std::vector<uint8_t>& _bytes);

  std::vector<uint8_t> bytes;
  static const size_t UUID_LEN = 12;

  bool operator==(const UUID& rhs);
  UUID& operator=(const UUID& rhs);

  void clear();
  bool is_valid() const;
  std::string to_string() const { return s; };
  std::string generate_string();

private:
  std::string s;
};

}  // namespace betz

#endif
