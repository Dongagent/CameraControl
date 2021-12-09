[toc]
# Requirement
Ubuntu 18.04
ffmpeg
Python 3.7.10
v4l-utils

anaconda
> sh ****.sh
nodejs
> conda install nodejs

sudo apt-get install v4l-utils

Change the path of Webcam everytime.
use ```v4l2-ctl --list-devices```

> v4l2-ctl -d /dev/video2 --set-ctrl focus_auto=1 # use auto focus
> v4l2-ctl -d /dev/video2 --get-ctrl focus_absolute # get the focus value after auto fucus
> v4l2-ctl -d /dev/video2 --set-ctrl focus_auto=0 # cancel auto fucus
