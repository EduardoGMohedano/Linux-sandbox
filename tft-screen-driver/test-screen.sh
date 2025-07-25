#!/bin/bash

echo "Modyfing text"
echo -n "Hello world!" > /dev/simple-tft
sleep 3
echo -n "New driver" > /dev/simple-tft
sleep 3
echo -n "TFT driver" > /dev/simple-tft
sleep 3

echo "Modyfing text color using ioctl"
./set_text_color 0 0 30
sleep 3
./set_text_color 0 60 0
sleep 3
./set_text_color 30 0 0
sleep 3

echo "Modyfing text move time interval using ioctl"
./set_time_interval 100
sleep 3
./set_time_interval 20
sleep 3
./set_time_interval 350