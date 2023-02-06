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

Please check the access permission of ```/dev/ttyUSB0```, better to use ```chmod 777 /dev/ttyUSB0```

### Initialize the Nikola on my laptop

#### 1 connect cable

#### 2 run roscore on one shell, 
```roscore```

#### 3 open another shell, source first, then rosrun rc rc
```source ~/catkin_ws/devel/setup.zsh```
```rosrun rc rc```

#### 4 run your code
```conda activate py37pyfeat1```
```rosrun dongagent_package RCSystem.py```

Exp10
Target Anger
Exp structure: 10 initialization + 490 iteration, 500 in total
Start time: 15:14:51
End time: 15:49:26
Duration: 33m 33s

Target Happiness and Fear
Exp structure for each expression: 10 initialization + 490 iteration, 500 in total
Duration: 1h 5m 22s

Target disgust, sadness, surprise and neutral
Exp structure: 10 initialization + 490 iteration, 500 in total
Start time: 15:37:00
End time:
Duration: 