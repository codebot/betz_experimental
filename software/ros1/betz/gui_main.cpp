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

#include <QApplication>
#include <QtWidgets>

#include "main_window.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "betz_gui");
  QApplication app(argc, argv);

  QCommandLineParser parser;
  parser.addHelpOption();
  parser.addPositionalArgument(
      "[transport_device]",
      "Transport device, like /dev/ttyUSB0");
  parser.process(QCoreApplication::arguments());

  if (parser.positionalArguments().isEmpty())
  {
    printf("Please specify the transport as a command-line argument.\n");
    printf("  examples:\n");
    printf("    betz_gui /dev/ttyUSB0\n");
    printf("    betz_gui multicast\n");
    return 1;
  }

  const std::string transport_name =
      parser.positionalArguments().at(0).toStdString();

  MainWindow mw;

  if (transport_name == "multicast")
  {
    printf("using multicast transport\n");
    mw.bus.use_multicast_transport();
  }
  else
  {
    printf("using serial transport\n");
    if (!mw.bus.use_serial_transport(transport_name))
    {
      printf(
          "unable to open requested serial device: [%s]\n",
          transport_name.c_str());
      return 1;
    }
  }

  mw.show();
  return app.exec();
}
