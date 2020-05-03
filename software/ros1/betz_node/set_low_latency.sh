#!/bin/bash
if [ "$#" -ne 1 ]; then
  echo "syntax: set_low_latency.sh DEVICENAME"
  echo ""
  echo "example: set_low_latency.sh ttyUSB0"
  exit 1
fi

fn=/sys/bus/usb-serial/devices/$1/latency_timer
if [ -f $fn ]; then
  echo 1 > $fn
else
  echo "device $1 does not seem to have a latency_timer file."
fi
