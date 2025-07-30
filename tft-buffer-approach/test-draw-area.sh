#!/bin/bash

echo "Modyfing Draw Area"
echo -n "x1:0,y1:0,x2:5,y2:0,x3:0,y3:5,x4:5,y4:5" > /dev/simple-tft
sleep 3
echo -n "x1:0,y1:0,x2:15,y2:0,x3:0,y3:15,x4:15,y4:15" > /dev/simple-tft
sleep 3
echo -n "x1:0,y1:0,x2:25,y2:0,x3:20,y3:25,x4:25,y4:25" > /dev/simple-tft
sleep 3
echo -n "x1:10,y1:10,x2:15,y2:10,x3:10,y3:15,x4:15,y4:15" > /dev/simple-tft
sleep 3
