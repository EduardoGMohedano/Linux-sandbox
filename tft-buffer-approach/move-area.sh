#!/bin/bash

echo "Start moving rectangle"

for i in {0..400..5}; do
    x0=$i
    y0=$i
    x1=$((i+5))
    y1=$y0
    x2=$x0
    y2=$((y0+5))
    x3=$((x0+5))
    y3=$((y0+5))
    window="x0:$x0,y1:$y0,x2:$x1,y2:$y1,x3:$x2,y3:$y2,x4:$x3,y4:$y3"
    echo -n "$window" > /dev/simple-tft
    # echo "$window"
    sleep 2
done
