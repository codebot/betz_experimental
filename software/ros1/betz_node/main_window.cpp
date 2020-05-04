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

  bus.packet_listener = std::bind(&MainWindow::packet_received, this, _1);

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
  //printf("tick\n");
  bus.spin_once();

  if (bus.enumeration_state == betz::Bus::EnumerationState::DISCOVERY)
  {
    if (bus.discovery_state == betz::Bus::DiscoveryState::COMPLETE)
    {
      printf("discovery complete\n");
      ui->drive_table->setRowCount(bus.drives.size());
      for (size_t i = 0; i < bus.drives.size(); i++)
        ui->drive_table->setItem(
            i,
            0, 
            new QTableWidgetItem(
                QString::fromStdString(bus.drives[i]->uuid_str)));
      bus.enumeration_begin();
    }
  }
}

void MainWindow::packet_received(const betz::Packet& packet)
{
  printf("MainWindow::packet_received()\n");
}
