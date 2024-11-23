[toc]
# Notes for developing

Current env: py37pyfeat1
Python 3.7.12

## library version

pytorch (install your compatiable version first.)

py-feat  0.5.0 (resmasknet is changed, DON'T use it until varified.)
py-feat  0.3.7

bayesian-optimization 1.4.2
Try bayesian-optimization 2.0.0 with python 3.10

rospkg 1.4.0



# Requirement
Ubuntu 20.04

ffmpeg

Python 3.8.15 (?)

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

> v4l2-ctl -d /dev/video2 -V # check resolutions

Setup the webcam
use ```ffplay /dev/video2```

While connecting the USB cable, use ```ls -Al /dev/ttyUSB0``` to check whether the cable is connected.

Please check the access permission of ```/dev/ttyUSB0```, better to use ```sudo chmod 777 /dev/ttyUSB0```

### Initialize the Nikola on my laptop

#### 0 OPEN THE POWER!
```OPEN THE POWER!```

#### 1 connect cable
```sudo chmod 777 /dev/ttyUSB0```

#### 2 run roscore on one shell, 
```roscore```

#### 3 open another shell, source first, then rosrun rc rc
```source ~/catkin_ws/devel/setup.zsh```

```rosrun rc rc```

#### 3 modify the facebox
In jupyter

#### 4 run your code
```conda activate py37pyfeat1```
```rosrun dongagent_package RCSystem.py```

# Experiment Record
### Exp10

Target Anger  
Exp structure: 10 initialization + 490 iteration, 500 in total  
> Start time: 15:14:51  
> End time: 15:49:26  
> Duration: 33m 33s  

Target Happiness and Fear  
Exp structure for each expression: 10 initialization + 490 iteration, 500 in total  
Duration: 1h 5m 22s  

### Exp11
Env: py-feat 0.5.0  
Target anger, disgust, fear, happiness, sadness, surprise and neutral  
Exp structure: 10 initialization + 490 iteration, 500 in total  
> Start time: 14:30:30  
> End time: 17:15:19  
> Duration: 2h 44m 49s  

### Exp12
Env: py-feat 0.3.7  
Target anger, disgust, fear, happiness, sadness, surprise and neutral  
Exp structure: 10 initialization + 490 iteration, 500 in total  
Start time: 16:15:00  
End time: 19:34:00  

### Exp13 lighting
Env: py-feat 0.3.7, use new lighting system  
Target anger, disgust, fear, happiness, sadness, surprise  
Exp structure: 10 initialization + 490 iteration, 500 in total   
Duration: 1h 54m 25s  

### Exp14 try ranknet

### Exp15 try new pyfeat
Env: py-feat 0.6.1, use new lighting system  
RFE: 6 basic (anger, disgust, fear, happiness, sadness, surprise)  
Head direction pos(35): 95
Exp structure: 10 initialization + 490 iteration, 500 in total 

Test: 100

### Exp16 Use Py-Feat 0.6.1 for Nikola, with happy eye restriction

### Exp 17: record video, record prototype images

### Exp 18: new environment basic video, sigmoid function

### Exp 19: test the head movement, collect data again confirming details

### Exp 20: writing the Main Framework of Experiment

### Exp 21: serial port control

### Exp 27: New baseline
Kappa = 1.576
init_points = 20
n_iter = 270
use_probe = True

### Exp28: mixed v1
Kappa = 3.576
init_points = 20
n_iter = 70
use_probe = True

### Exp29: mixed v2
Kappa = 7.576
init_points = 20
n_iter = 70
use_probe = True
alpha=0.7

### Exp30: mixed v3
alpha=0.6
Kappa = 7.576
init_points = 20
n_iter = 70
use_probe = True
alpha=0.7

### SET UP WEB CAM

use ```v4l2-ctl -d /dev/video2 -c exposure_auto=1 -c white_balance_temperature_auto=0``` means manual, 3 means auto
<!-- ```v4l2-ctl -d /dev/video2 -c white_balance_temperature_auto=0``` -->

```v4l2-ctl -d /dev/video2 --set-ctrl=brightness=140 --set-ctrl=gain=20 --set-ctrl=exposure_absolute=333 --set-ctrl=white_balance_temperature=4336 -c zoom_absolute=150```
```v4l2-ctl -d /dev/video2 -c zoom_absolute=150```



v4l2-ctl -d /dev/video2 -L                       
                     brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=140
                       contrast 0x00980901 (int)    : min=0 max=255 step=1 default=128 value=128
                     saturation 0x00980902 (int)    : min=0 max=255 step=1 default=128 value=128
 white_balance_temperature_auto 0x0098090c (bool)   : default=1 value=1
                           gain 0x00980913 (int)    : min=0 max=255 step=1 default=0 value=20
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=2 value=2
				0: Disabled
				1: 50 Hz
				2: 60 Hz
      white_balance_temperature 0x0098091a (int)    : min=2000 max=6500 step=1 default=4000 value=4336 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=255 step=1 default=128 value=128
         backlight_compensation 0x0098091c (int)    : min=0 max=1 step=1 default=0 value=0
                  exposure_auto 0x009a0901 (menu)   : min=0 max=3 default=3 value=1
				1: Manual Mode
				3: Aperture Priority Mode
              exposure_absolute 0x009a0902 (int)    : min=3 max=2047 step=1 default=250 value=333
         exposure_auto_priority 0x009a0903 (bool)   : default=0 value=1
                   pan_absolute 0x009a0908 (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                  tilt_absolute 0x009a0909 (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                 focus_absolute 0x009a090a (int)    : min=0 max=250 step=5 default=0 value=0 flags=inactive
                     focus_auto 0x009a090c (bool)   : default=1 value=1
                  zoom_absolute 0x009a090d (int)    : min=100 max=500 step=1 default=100 value=100