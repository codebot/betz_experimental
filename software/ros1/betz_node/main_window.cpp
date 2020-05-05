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

#include "ui_mainwindow.h"
#include "main_window.h"
#include <QTimer>
#include <functional>

using betz::Packet;
using betz::Drive;


MainWindow::MainWindow(QWidget *parent)
: QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  connect(ui->actionDiscover, &QAction::triggered, [this]() { discover(); } );

  connect(
      ui->actionQuit,
      &QAction::triggered,
      QCoreApplication::instance(),
      &QCoreApplication::quit);

  bus.packet_listener =
      [this](const Packet& p) { this->rx_packet(p); };

  QTimer *timer = new QTimer(this);
  connect(timer, &QTimer::timeout, [this]() { tick(); });
  // todo: connect ui->drive_table cell clicked to opening a param table
  timer->start(10);
  // todo: start discovery as a one-shot timer after startup
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::discover()
{
  printf("discover()\n");
  bus.discovery_begin();
}

void MainWindow::tick()
{
  bus.spin_once();
}

void MainWindow::rx_discovery(const Packet& packet)
{
  ui->drive_table->setRowCount(bus.drives.size());
  for (size_t i = 0; i < bus.drives.size(); i++)
    ui->drive_table->setItem(
        i,
        1, 
        new QTableWidgetItem(
            QString::fromStdString(bus.drives[i]->uuid.to_string())));

  if (selected_uuid.empty() && !bus.drives.empty())
    selected_uuid = bus.drives[0]->uuid.to_string();
}

void MainWindow::rx_num_params(const betz::Packet& packet)
{
  if (selected_uuid == packet.uuid.to_string())
  {
    ui->param_table->setRowCount(
        bus.drive_by_uuid_str(selected_uuid)->params.size());
  }
}

void MainWindow::rx_param_name_value(const betz::Packet& packet)
{
  uint32_t param_idx = 0;
  memcpy(&param_idx, &packet.payload[1], sizeof(param_idx));
  if (selected_uuid == packet.uuid.to_string())
  {
    std::shared_ptr<Drive> drive = bus.drive_by_uuid_str(selected_uuid);
    if (!drive)
    {
      printf("WOAH couldn't find drive\n");
      return;
    }
    if (param_idx >= ui->param_table->rowCount())
    {
      printf("WOAH invalid param_idx\n");
      return;
    }
    if (param_idx >= static_cast<uint32_t>(drive->params.size()))
    {
      printf("WOAH param_idx not in drive!\n");
      return;
    }
    printf("drive %s param %d name [%s]\n",
        drive->uuid.to_string().c_str(),
        (int)param_idx,
        drive->params[param_idx].name.c_str());

    ui->param_table->setItem(
        param_idx,
        0,
        new QTableWidgetItem(
            QString::fromStdString(drive->params[param_idx].name)));
  }
}

void MainWindow::rx_packet(const Packet& packet)
{
  if (packet.payload.size() == 0)
    return;

  // dispatch packet to avoid absurd indentations and a mega-function here
  const uint8_t packet_id = packet.payload[0];
  switch (packet_id)
  {
    case Packet::ID_DISCOVERY:        rx_discovery(packet); break;
    case Packet::ID_NUM_PARAMS:       rx_num_params(packet); break;
    case Packet::ID_PARAM_NAME_VALUE: rx_param_name_value(packet); break;
    default: break;
  }
}
