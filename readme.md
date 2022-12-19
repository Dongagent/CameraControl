[toc]
# Notes for developing

Current env: py37pyfeat1

## library version

pytorch (install your compatiable version first.)

py-feat  0.5.0

bayesian-optimization 1.4.2

rospkg 1.4.0



# Requirement
Ubuntu 20.04

ffmpeg

Python 3.8.15

v4l-utils

bayes_opt

py-feat=0.3.7

anaconda
> sh ****.sh
nodejs
> conda install nodejs

sudo apt-get install v4l-utils

Change the path of Webcam everytime.
use ```v4l2-ctl --list-devices``` to check available webcams on Ubuntu

> v4l2-ctl -d /dev/video2 --set-ctrl focus_auto=1 # use auto focus
> v4l2-ctl -d /dev/video2 --get-ctrl focus_absolute # get the focus value after auto focus
> v4l2-ctl -d /dev/video2 --set-ctrl focus_auto=0 # cancel auto focus

While connecting the USB cable, use ```ls -Al /dev/ttyUSB0``` to check whether the cable is connected.