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

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include "bus.h"
#include "packet/packet.h"
#include <ros/ros.h>

namespace Ui {
  class MainWindow;
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);

  virtual ~MainWindow();

public slots:
  void discover();
  void tick();

public:
  betz::Bus bus;
  ros::NodeHandle ros_node;

private:
  Ui::MainWindow *ui;
  void rx_packet(const betz::Packet& packet);
  void rx_discovery(const betz::Packet& packet);
  void rx_num_params(const betz::Packet& packet);
  void rx_param_name_value(const betz::Packet& packet);
  void set_selected_drive(const std::string& uuid);
  void param_changed(const int param_idx);

  std::string selected_uuid;
};

#endif
