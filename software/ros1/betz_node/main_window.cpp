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
#include "packet/param_set_value.h"

#include <QTimer>
#include <functional>
#include <memory>

using betz::Drive;
using betz::Packet;
using betz::Param;
using betz::ParamSetValue;


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

  connect(
      ui->param_table,
      &QTableWidget::cellChanged,
      [this](int row, int col) { this->param_changed(row); });

  ui->data_table->horizontalHeader()->setSectionResizeMode(
      0,
      QHeaderView::ResizeToContents);

  ui->data_table->horizontalHeader()->setSectionResizeMode(
      1,
      QHeaderView::ResizeToContents);

  bus.packet_listener =
      [this](const Packet& p) { this->rx_packet(p); };

  connect(
      ui->actionStream,
      &QAction::toggled,
      [this](bool checked) { this->stream = checked; } );

  stream_elapsed_timer.start();

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
  if (stream && stream_elapsed_timer.elapsed() > 500)
  {
    stream_elapsed_timer.restart();
    // todo: send "verbose state poll" packet
  }
}

void MainWindow::rx_discovery(const Packet& packet)
{
  ui->drive_table->setRowCount(bus.drives.size());
  for (size_t i = 0; i < bus.drives.size(); i++)
  {
    auto id_item = new QTableWidgetItem("");
    ui->drive_table->setItem(i, 0, id_item);
    id_item->setFlags(id_item->flags() & ~Qt::ItemIsEditable);

    auto uuid_item = new QTableWidgetItem(
        QString::fromStdString(bus.drives[i]->uuid.to_string()));
    uuid_item->setFlags(uuid_item->flags() & ~Qt::ItemIsEditable);

    ui->drive_table->setItem(i, 1, uuid_item);
  }

  if (!bus.drives.empty())
    set_selected_drive(bus.drives[0]->uuid.to_string());
}

void MainWindow::rx_num_params(const betz::Packet& packet)
{
  if (selected_uuid == packet.uuid.to_string())
  {
    const size_t num_params = 
        bus.drive_by_uuid_str(selected_uuid)->params.size();

    ui->param_table->setRowCount(num_params);

    ui->param_table->blockSignals(true);
    for (int i = 0; i < num_params; i++)
    {
      auto name_item = new QTableWidgetItem("?");
      name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
      ui->param_table->setItem(i, 0, name_item);

      auto value_item = new QTableWidgetItem("");
      ui->param_table->setItem(i, 1, value_item);
    }
    ui->param_table->blockSignals(false);
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

    ui->param_table->blockSignals(true);

    const Param& param = drive->params[param_idx];
    printf("drive %s (%d params) param %d type %d name [%s]\n",
        drive->uuid.to_string().c_str(),
        (int)drive->params.size(),
        (int)param_idx,
        static_cast<int>(param.type),
        param.name.c_str());

    ui->param_table->item(param_idx, 0)->setText(
        QString::fromStdString(param.name));

    auto value_item = ui->param_table->item(param_idx, 1);
    if (param.type == Param::Type::INT)
      value_item->setText(QString::number(param.i_value));
    else if (param.type == Param::Type::FLOAT)
      value_item->setText(QString::number(param.f_value));
    else
      ROS_ERROR("WOAH unknown type of param idx %d", (int)param_idx);

    ui->param_table->blockSignals(false);
  }

  // the 'id' parameter is magic, so we special-case it here.
  // always render it if we get it, no matter what drive is selected
  if (packet.payload[5] == static_cast<uint8_t>(Param::Type::INT) &&
      packet.payload[6] == static_cast<uint8_t>(Param::Storage::PERSISTENT) &&
      packet.payload[7] == 2 &&
      packet.payload[8] == 'i' &&
      packet.payload[9] == 'd' &&
      packet.payload.size() == 14)
  {
    int32_t id = 0;
    memcpy(&id, &packet.payload[10], 4);
    std::shared_ptr<Drive> drive = bus.drive_by_uuid_str(selected_uuid);
    for (size_t drive_idx = 0; drive_idx < bus.drives.size(); drive_idx++)
      if (bus.drives[drive_idx]->uuid == packet.uuid)
        ui->drive_table->item(drive_idx, 0)->setText(QString::number(id));
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

void MainWindow::set_selected_drive(const std::string& uuid_str)
{
  printf("set_selected_drive(%s)\n", uuid_str.c_str());
  selected_uuid = uuid_str;

  const QBrush selected = QBrush(QColor("#c0c0ff"));
  const QBrush white = QBrush(QColor("#ffffff"));

  const int num_rows = ui->drive_table->rowCount();  // avoid line wrap
  for (size_t i = 0; i < bus.drives.size() && i < num_rows; i++)
  {
    const std::string row_uuid = bus.drives[i]->uuid.to_string();

    QTableWidgetItem *id_item = ui->drive_table->item(i, 0);
    if (id_item)
      id_item->setBackground(selected_uuid == row_uuid ? selected : white);

    QTableWidgetItem *uuid_item = ui->drive_table->item(i, 1);
    if (uuid_item)
      uuid_item->setBackground(selected_uuid == row_uuid ? selected : white);
  }
}

void MainWindow::param_changed(const int param_idx)
{
  printf("param_changed(%d)\n", param_idx);
  auto drive = bus.drive_by_uuid_str(selected_uuid);
  if (!drive)
  {
    ROS_ERROR("woah! couldn't find drive for param");
    return;
  }
  if (param_idx >= static_cast<int>(drive->params.size()))
  {
    ROS_ERROR("woah! bogus row index for drive param");
    return;
  }

  auto value_item = ui->param_table->item(param_idx, 1);

  auto& param = drive->params[param_idx];
  printf("param name: [%s]\n", param.name.c_str());
  if (param.type == Param::Type::INT)
  {
    param.i_value = static_cast<int32_t>(value_item->text().toInt());
  }
  else if (param.type == Param::Type::FLOAT)
  {
    param.f_value = value_item->text().toFloat();
  }
  else
  {
    ROS_ERROR("woah! unknown param type");
    return;
  }
  bus.send_packet(std::make_unique<ParamSetValue>(*drive, param));
}
