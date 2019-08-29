#!/bin/bash

usage ()
{
    echo "Usage:  server <viewer_ip> <viewer_port>"
}


if [ $# -ne 2 ]; then
    usage
    exit 0
fi

echo Configuring the camera...

# pixelformat=1 enables the in-camera H.264 encoder
v4l2-ctl --set-fmt-video=width=800,height=448,pixelformat=1
v4l2-ctl --set-parm=30

echo "Starting streaming server to $1:$2"

./capture -o | gst-launch -v -e filesrc location=/dev/fd/0 ! legacyh264parse ! queue ! rtph264pay pt=96 config-interval=1 ! queue ! tcpserversink host=$1 port=$2 protocol=1
