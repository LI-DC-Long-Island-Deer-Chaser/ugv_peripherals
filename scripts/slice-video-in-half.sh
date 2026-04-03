#!/usr/bin/bash
ffmpeg -f v4l2 -i /dev/video0 -vf "crop=1920:540:0:0,format=yuv420p" -f v4l2 /dev/video2
