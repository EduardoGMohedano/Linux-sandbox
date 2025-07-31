#!/bin/bash

echo "Create Draw Buffer Area"
echo -n "x1:0,y1:0,x2:7,y2:0,x3:0,y3:7,x4:7,y4:7" > /dev/simple-tft
echo -n "len=49,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30" > /dev/simple-tft
sleep 1

echo "Start moving rectangle..."
for i in {0..400..5}; do
    x0=$i
    y0=$i
    x1=$((i+7))
    y1=$y0
    x2=$x0
    y2=$((y0+7))
    x3=$((x0+7))
    y3=$((y0+7))
    window="x1:$x0,y1:$y0,x2:$x1,y2:$y1,x3:$x2,y3:$y2,x4:$x3,y4:$y3"
    echo -n "$window" > /dev/simple-tft
    sleep 0.2
done