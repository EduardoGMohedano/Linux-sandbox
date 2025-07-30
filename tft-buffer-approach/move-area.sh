#!/bin/bash

echo "Create Draw Buffer Area"
echo -n "x1:0,y1:0,x2:4,y2:0,x3:0,y3:4,x4:4,y4:4" > /dev/simple-tft
echo -n "len=25,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488,63488" > /dev/simple-tft
sleep 1

echo "Start moving rectangle..."
for i in {0..400..5}; do
    x0=$i
    y0=$i
    x1=$((i+4))
    y1=$y0
    x2=$x0
    y2=$((y0+4))
    x3=$((x0+4))
    y3=$((y0+4))
    window="x1:$x0,y1:$y0,x2:$x1,y2:$y1,x3:$x2,y3:$y2,x4:$x3,y4:$y3"
    echo -n "$window" > /dev/simple-tft
    # echo "$window"
    sleep 2
done
