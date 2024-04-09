#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This experiment was created using PsychoPy3 Experiment Builder (v2022.2.5),
    on 10月 30, 2023, at 22:57
If you publish work using this script the most relevant publication is:

    Peirce J, Gray JR, Simpson S, MacAskill M, Höchenberger R, Sogo H, Kastman E, Lindeløv JK. (2019) 
        PsychoPy2: Experiments in behavior made easy Behav Res 51: 195. 
        https://doi.org/10.3758/s13428-018-01193-y

"""

# --- Import packages ---
from psychopy import locale_setup
from psychopy import prefs
from psychopy import sound, gui, visual, core, data, event, logging, clock, colors, layout
from psychopy.constants import (NOT_STARTED, STARTED, PLAYING, PAUSED,
                                STOPPED, FINISHED, PRESSED, RELEASED, FOREVER)

import numpy as np  # whole numpy lib is available, prepend 'np.'
from numpy import (sin, cos, tan, log, log10, pi, average,
                   sqrt, std, deg2rad, rad2deg, linspace, asarray)
from numpy.random import random, randint, normal, shuffle, choice as randchoice
import os  # handy system and path functions
import sys  # to get file system encoding

import psychopy.iohub as io
from psychopy.hardware import keyboard

# Run 'Before Experiment' code from init_variable
# Stop idle behavior
import roslibpy
import json
import time
import socket
import threading

#  server setting
HOST = "nikola-control.ad180.riken.go.jp"
PORT1 = 21000
PORT2 = 20000
BUFSIZE = 4096

# create socket
client1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# connect server
client1.connect((HOST, PORT1))
client2.connect((HOST, PORT2))

# roslibpy server setting
client = roslibpy.Ros(host="nikola-master.local", port=9099)
client.run() # connect roslibpy 

# roslibpy topic 
dib_topic = roslibpy.Topic(client, '/do_idle_behavior', 'std_msgs/String')
tts_topic = roslibpy.Topic(client, '/tts', 'std_msgs/String')

# Volume
Master_Volume = 1.3

# roslibpy message
compoff_true = {"completely_off": True}   # stop idle behavior
compoff_false = {"completely_off": False} # start idle behavior

# init cmd
head_cmd0_1 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"
            
head_cmd0_2 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"
            
head_cmd1_1 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"
            
head_cmd1_2 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"
            
head_cmd3_1 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.9,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"

eye_cmd0_1 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"

eye_cmd0_2 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"

eye_cmd1_1 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"

eye_cmd1_2 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"
            
facs_cmd0 = "facs InnerBrow 0 \
             Outerbrow 0 \
             UpperLid -1 \
             LowerLid 1 \
             Cheek 0 \
             NoseWrinkler 0 \
             UpperLip 0 \
             LipCorner 0 \
             MouthCornerStickout 0 \
             MouthOpen 0 \
             JawDrop 0\n"
             
facs_cmd1 = "facs InnerBrow 0 \
             Outerbrow 0 \
             UpperLid 0 \
             LowerLid 0 \
             Cheek 0 \
             NoseWrinkler 0 \
             UpperLip 0 \
             LipCorner 0 \
             MouthCornerStickout 0 \
             MouthOpen 0 \
             JawDrop 0\n"
# Run 'Before Experiment' code from init_stim
stim_txt = ["ぼくはふつう", "ぼくはかなしい", "ぼくはつらい", "ぼくはうれしい", "ぼくはたのしい"]
stim_exp = ["facs InnerBrow 0 Outerbrow 0 UpperLid 0 LowerLid 0 Cheek 0 NoseWrinkler 0 UpperLip 0 LipCorner 0 MouthCornerStickout 0 MouthOpen 0 JawDrop 0\n", \
            "facs InnerBrow 1.0 Outerbrow 0.0 UpperLid -0.7 LowerLid 0 Cheek 0 NoseWrinkler 0.0 UpperLip 0 LipCorner -0.9 MouthCornerStickout 0 MouthOpen 0 JawDrop 0\n", \
            "facs InnerBrow 1.0 Outerbrow 0.0 UpperLid -0.8 LowerLid 0 Cheek 0 NoseWrinkler 0.0 UpperLip 0 LipCorner -0.9 MouthCornerStickout 0 MouthOpen 0 JawDrop 0\n", \
            "facs InnerBrow 1.0 Outerbrow 0.5 UpperLid 0.2 LowerLid 1 Cheek 1.0 NoseWrinkler 0 UpperLip 0 LipCorner 0.5 MouthCornerStickout 0.5 MouthOpen 0 JawDrop 0\n", \
            "facs InnerBrow 1.0 Outerbrow 0.5 UpperLid 0.3 LowerLid 1 Cheek 1.0 NoseWrinkler 0 UpperLip 0 LipCorner 0.5 MouthCornerStickout 0.5 MouthOpen 0 JawDrop 0\n"]
# Run 'Before Experiment' code from init_variable_ex
# Stop idle behavior
import roslibpy
import json
import time
import socket
import threading

#  server setting
HOST = "nikola-control.ad180.riken.go.jp"
PORT1 = 21000
PORT2 = 20000
BUFSIZE = 4096

# create socket
client1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# connect server
client1.connect((HOST, PORT1))
client2.connect((HOST, PORT2))

# roslibpy server setting
client = roslibpy.Ros(host="nikola-master.local", port=9099)
client.run() # connect roslibpy 

# roslibpy topic 
dib_topic = roslibpy.Topic(client, '/do_idle_behavior', 'std_msgs/String')
tts_topic = roslibpy.Topic(client, '/tts', 'std_msgs/String')

# roslibpy message
compoff_true = {"completely_off": True}   # stop idle behavior
compoff_false = {"completely_off": False} # start idle behavior

# init cmd
head_cmd0_1 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"
            
head_cmd0_2 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"
            
head_cmd1_1 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"
            
head_cmd1_2 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"
            
head_cmd3_1 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.9,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"

eye_cmd0_1 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"

eye_cmd0_2 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"

eye_cmd1_1 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"

eye_cmd1_2 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"
            
facs_cmd0 = "facs InnerBrow 0 \
             Outerbrow 0 \
             UpperLid -1 \
             LowerLid 1 \
             Cheek 0 \
             NoseWrinkler 0 \
             UpperLip 0 \
             LipCorner 0 \
             MouthCornerStickout 0 \
             MouthOpen 0 \
             JawDrop 0\n"
             
facs_cmd1 = "facs InnerBrow 0 \
             Outerbrow 0 \
             UpperLid 0 \
             LowerLid 0 \
             Cheek 0 \
             NoseWrinkler 0 \
             UpperLip 0 \
             LipCorner 0 \
             MouthCornerStickout 0 \
             MouthOpen 0 \
             JawDrop 0\n"
# Run 'Before Experiment' code from init_variable_ex
# Stop idle behavior
import roslibpy
import json
import time
import socket
import threading

#  server setting
HOST = "nikola-control.ad180.riken.go.jp"
PORT1 = 21000
PORT2 = 20000
BUFSIZE = 4096

# create socket
client1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# connect server
client1.connect((HOST, PORT1))
client2.connect((HOST, PORT2))

# roslibpy server setting
client = roslibpy.Ros(host="nikola-master.local", port=9099)
client.run() # connect roslibpy 

# roslibpy topic 
dib_topic = roslibpy.Topic(client, '/do_idle_behavior', 'std_msgs/String')
tts_topic = roslibpy.Topic(client, '/tts', 'std_msgs/String')

# roslibpy message
compoff_true = {"completely_off": True}   # stop idle behavior
compoff_false = {"completely_off": False} # start idle behavior

# init cmd
head_cmd0_1 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"
            
head_cmd0_2 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"
            
head_cmd1_1 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"
            
head_cmd1_2 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"
            
head_cmd3_1 = "HeadController={\"id\":\"HeadController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.9,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"

eye_cmd0_1 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"

eye_cmd0_2 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"

eye_cmd1_1 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":1.0}\n"

eye_cmd1_2 = "EyeController={\"id\":\"EyeController\", \
            \"motionTowardObject\":\"\", \
            \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \
            \"translateSpeed\":2.0}\n"
            
facs_cmd0 = "facs InnerBrow 0 \
             Outerbrow 0 \
             UpperLid -1 \
             LowerLid 1 \
             Cheek 0 \
             NoseWrinkler 0 \
             UpperLip 0 \
             LipCorner 0 \
             MouthCornerStickout 0 \
             MouthOpen 0 \
             JawDrop 0\n"
             
facs_cmd1 = "facs InnerBrow 0 \
             Outerbrow 0 \
             UpperLid 0 \
             LowerLid 0 \
             Cheek 0 \
             NoseWrinkler 0 \
             UpperLip 0 \
             LipCorner 0 \
             MouthCornerStickout 0 \
             MouthOpen 0 \
             JawDrop 0\n"


# Ensure that relative paths start from the same directory as this script
_thisDir = os.path.dirname(os.path.abspath(__file__))
os.chdir(_thisDir)
# Store info about the experiment session
psychopyVersion = '2022.2.5'
expName = 'Resume Gaze'  # from the Builder filename that created this script
expInfo = {
    'participant': f"{randint(0, 999999):06.0f}",
    'session': '001',
}
# --- Show participant info dialog --
dlg = gui.DlgFromDict(dictionary=expInfo, sortKeys=False, title=expName)
if dlg.OK == False:
    core.quit()  # user pressed cancel
expInfo['date'] = data.getDateStr()  # add a simple timestamp
expInfo['expName'] = expName
expInfo['psychopyVersion'] = psychopyVersion

# Data file name stem = absolute path + name; later add .psyexp, .csv, .log, etc
filename = _thisDir + os.sep + u'data/%s_%s_%s' % (expInfo['participant'], expName, expInfo['date'])

# An ExperimentHandler isn't essential but helps with data saving
thisExp = data.ExperimentHandler(name=expName, version='',
    extraInfo=expInfo, runtimeInfo=None,
    originPath='C:\\Users\\SATO\\OneDrive\\Coding\\Dr.Sato\\exp_202311\\ResumeMulti_lastrun.py',
    savePickle=True, saveWideText=True,
    dataFileName=filename)
# save a log file for detail verbose info
logFile = logging.LogFile(filename+'.log', level=logging.EXP)
logging.console.setLevel(logging.WARNING)  # this outputs to the screen, not a file

endExpNow = False  # flag for 'escape' or other condition => quit the exp
frameTolerance = 0.001  # how close to onset before 'same' frame

# Start Code - component code to be run after the window creation

# --- Setup the Window ---
win = visual.Window(
    size=[720, 480], fullscr=False, screen=0, 
    winType='pyglet', allowStencil=False,
    monitor='testMonitor', color=[0,0,0], colorSpace='rgb',
    blendMode='avg', useFBO=True, 
    units='height')
win.mouseVisible = True
# store frame rate of monitor if we can measure it
expInfo['frameRate'] = win.getActualFrameRate()
if expInfo['frameRate'] != None:
    frameDur = 1.0 / round(expInfo['frameRate'])
else:
    frameDur = 1.0 / 60.0  # could not measure, so guess
# --- Setup input devices ---
ioConfig = {}

# Setup iohub keyboard
ioConfig['Keyboard'] = dict(use_keymap='psychopy')

ioSession = '1'
if 'session' in expInfo:
    ioSession = str(expInfo['session'])
ioServer = io.launchHubServer(window=win, **ioConfig)
eyetracker = None

# create a default keyboard (e.g. to check for escape)
defaultKeyboard = keyboard.Keyboard(backend='iohub')

# --- Initialize components for Routine "Init" ---
init = visual.TextStim(win=win, name='init',
    text='実験プログラムを起動しています。\n暫くお待ちください。',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
# Run 'Begin Experiment' code from init_text
json_data_ikimasu = {
    "Command": "ttsPlaywithParam",
    "TtsID": 1023,
    "Text": "いきます。",
    "Volume": 1.0,
    "Rate": 1.0,
    "Pitch": 1.0,
    "Emphasis": 1.0,
    "Emotion":
    {
        "Joy": 0.0,
        "Angry": 0.0,
        "Sad": 0.0
    }
}

# --- Initialize components for Routine "Explanation" ---
explanation_1 = visual.TextStim(win=win, name='explanation_1',
    text='実験を行います。\nキーボードの1~9を使って、アンドロイドの\n表情に対する評価値を以下の2つの項目別に入力してください。\n・感情価（説明）\n・ヒトらしさ（説明）\n\n',
    font='Open Sans',
    pos=(0, 0.25), height=0.03, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
routine_end_1 = event.Mouse(win=win)
x, y = [None, None]
routine_end_1.mouseClock = core.Clock()
disp_keyresp = visual.TextStim(win=win, name='disp_keyresp',
    text=None,
    font='Open Sans',
    pos=(0, 0), height=0.03, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-2.0);
keyresp = keyboard.Keyboard()
explanation_2 = visual.TextStim(win=win, name='explanation_2',
    text='マウスの左クリックを押してを練習を始めます。',
    font='Open Sans',
    pos=(0, -0.25), height=0.02, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-5.0);

# --- Initialize components for Routine "Eyecatch" ---
text = visual.TextStim(win=win, name='text',
    text='準備しています．．．',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);

# --- Initialize components for Routine "Idleoff" ---

# --- Initialize components for Routine "Eyecatch_demo" ---
text_6 = visual.TextStim(win=win, name='text_6',
    text='デモ\nランダムな表情、会話内容、プロソディを計5パターンをご確認ください。',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);

# --- Initialize components for Routine "Demo_init" ---

# --- Initialize components for Routine "Demo_stim" ---

# --- Initialize components for Routine "Idleon" ---

# --- Initialize components for Routine "Idleoff" ---

# --- Initialize components for Routine "Start_Countdown" ---
count_sc = visual.TextStim(win=win, name='count_sc',
    text='',
    font='Open Sans',
    pos=(0, 0), height=0.1, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);

# --- Initialize components for Routine "Test_Eyecatch" ---
# Run 'Begin Experiment' code from code
trialCounter = 1
text_3 = visual.TextStim(win=win, name='text_3',
    text=None,
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);

# --- Initialize components for Routine "Exercise" ---

# --- Initialize components for Routine "Check_v" ---
text_4 = visual.TextStim(win=win, name='text_4',
    text='感情価の値を1~9のボタンで評価してください。',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
key_resp_2 = keyboard.Keyboard()

# --- Initialize components for Routine "Check_a" ---
text_9 = visual.TextStim(win=win, name='text_9',
    text='ヒトらしさの値を1~9のボタンで評価してください。',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
key_resp_3 = keyboard.Keyboard()

# --- Initialize components for Routine "Idleon" ---

# --- Initialize components for Routine "Standby" ---
text_5 = visual.TextStim(win=win, name='text_5',
    text='本番の実験を開始します。\n準備できたら左クリックを押してください。',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
mouse_4 = event.Mouse(win=win)
x, y = [None, None]
mouse_4.mouseClock = core.Clock()

# --- Initialize components for Routine "Idleoff" ---

# --- Initialize components for Routine "Start_Countdown" ---
count_sc = visual.TextStim(win=win, name='count_sc',
    text='',
    font='Open Sans',
    pos=(0, 0), height=0.1, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);

# --- Initialize components for Routine "Counter" ---
# Run 'Begin Experiment' code from break_counter
trialCounter = 1
nRestTrial = 28 # 28回目で休憩をはさむ
text_2 = visual.TextStim(win=win, name='text_2',
    text=None,
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);

# --- Initialize components for Routine "Idleon" ---

# --- Initialize components for Routine "Routine" ---
text_countdown = visual.TextStim(win=win, name='text_countdown',
    text='',
    font='Open Sans',
    pos=(0, 0), height=0.1, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
text_7 = visual.TextStim(win=win, name='text_7',
    text='マウスを左クリックして再開します。',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);
mouse_3 = event.Mouse(win=win)
x, y = [None, None]
mouse_3.mouseClock = core.Clock()

# --- Initialize components for Routine "Idleoff" ---

# --- Initialize components for Routine "Exercise" ---

# --- Initialize components for Routine "Check_v" ---
text_4 = visual.TextStim(win=win, name='text_4',
    text='感情価の値を1~9のボタンで評価してください。',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
key_resp_2 = keyboard.Keyboard()

# --- Initialize components for Routine "Check_a" ---
text_9 = visual.TextStim(win=win, name='text_9',
    text='ヒトらしさの値を1~9のボタンで評価してください。',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
key_resp_3 = keyboard.Keyboard()

# --- Initialize components for Routine "Idleon" ---

# --- Initialize components for Routine "finish" ---
text_8 = visual.TextStim(win=win, name='text_8',
    text='終了します',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
mouse_2 = event.Mouse(win=win)
x, y = [None, None]
mouse_2.mouseClock = core.Clock()

# Create some handy timers
globalClock = core.Clock()  # to track the time since experiment started
routineTimer = core.Clock()  # to track time remaining of each (possibly non-slip) routine 

# --- Prepare to start Routine "Init" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# keep track of which components have finished
InitComponents = [init]
for thisComponent in InitComponents:
    thisComponent.tStart = None
    thisComponent.tStop = None
    thisComponent.tStartRefresh = None
    thisComponent.tStopRefresh = None
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED
# reset timers
t = 0
_timeToFirstFrame = win.getFutureFlipTime(clock="now")
frameN = -1

# --- Run Routine "Init" ---
while continueRoutine and routineTimer.getTime() < 5.0:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *init* updates
    if init.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        init.frameNStart = frameN  # exact frame index
        init.tStart = t  # local t and not account for scr refresh
        init.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(init, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.timestampOnFlip(win, 'init.started')
        init.setAutoDraw(True)
    if init.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > init.tStartRefresh + 5.0-frameTolerance:
            # keep track of stop time/frame for later
            init.tStop = t  # not accounting for scr refresh
            init.frameNStop = frameN  # exact frame index
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'init.stopped')
            init.setAutoDraw(False)
    
    # check for quit (typically the Esc key)
    if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
        core.quit()
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineForceEnded = True
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in InitComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Init" ---
for thisComponent in InitComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# using non-slip timing so subtract the expected duration of this Routine (unless ended on request)
if routineForceEnded:
    routineTimer.reset()
else:
    routineTimer.addTime(-5.000000)

# --- Prepare to start Routine "Explanation" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# setup some python lists for storing info about the routine_end_1
routine_end_1.x = []
routine_end_1.y = []
routine_end_1.leftButton = []
routine_end_1.midButton = []
routine_end_1.rightButton = []
routine_end_1.time = []
gotValidClick = False  # until a click is received
keyresp.keys = []
keyresp.rt = []
_keyresp_allKeys = []
# Run 'Begin Routine' code from check_keyresp
displayText=''
text_=''
# keep track of which components have finished
ExplanationComponents = [explanation_1, routine_end_1, disp_keyresp, keyresp, explanation_2]
for thisComponent in ExplanationComponents:
    thisComponent.tStart = None
    thisComponent.tStop = None
    thisComponent.tStartRefresh = None
    thisComponent.tStopRefresh = None
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED
# reset timers
t = 0
_timeToFirstFrame = win.getFutureFlipTime(clock="now")
frameN = -1

# --- Run Routine "Explanation" ---
while continueRoutine:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *explanation_1* updates
    if explanation_1.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        explanation_1.frameNStart = frameN  # exact frame index
        explanation_1.tStart = t  # local t and not account for scr refresh
        explanation_1.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(explanation_1, 'tStartRefresh')  # time at next scr refresh
        explanation_1.setAutoDraw(True)
    # *routine_end_1* updates
    if routine_end_1.status == NOT_STARTED and t >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        routine_end_1.frameNStart = frameN  # exact frame index
        routine_end_1.tStart = t  # local t and not account for scr refresh
        routine_end_1.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(routine_end_1, 'tStartRefresh')  # time at next scr refresh
        routine_end_1.status = STARTED
        routine_end_1.mouseClock.reset()
        prevButtonState = routine_end_1.getPressed()  # if button is down already this ISN'T a new click
    if routine_end_1.status == STARTED:  # only update if started and not finished!
        buttons = routine_end_1.getPressed()
        if buttons != prevButtonState:  # button state changed?
            prevButtonState = buttons
            if sum(buttons) > 0:  # state changed to a new click
                x, y = routine_end_1.getPos()
                routine_end_1.x.append(x)
                routine_end_1.y.append(y)
                buttons = routine_end_1.getPressed()
                routine_end_1.leftButton.append(buttons[0])
                routine_end_1.midButton.append(buttons[1])
                routine_end_1.rightButton.append(buttons[2])
                routine_end_1.time.append(routine_end_1.mouseClock.getTime())
                
                continueRoutine = False  # abort routine on response
    
    # *disp_keyresp* updates
    if disp_keyresp.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        disp_keyresp.frameNStart = frameN  # exact frame index
        disp_keyresp.tStart = t  # local t and not account for scr refresh
        disp_keyresp.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(disp_keyresp, 'tStartRefresh')  # time at next scr refresh
        disp_keyresp.setAutoDraw(True)
    
    # *keyresp* updates
    if keyresp.status == NOT_STARTED and t >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        keyresp.frameNStart = frameN  # exact frame index
        keyresp.tStart = t  # local t and not account for scr refresh
        keyresp.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(keyresp, 'tStartRefresh')  # time at next scr refresh
        keyresp.status = STARTED
        # keyboard checking is just starting
        keyresp.clock.reset()  # now t=0
        keyresp.clearEvents(eventType='keyboard')
    if keyresp.status == STARTED:
        theseKeys = keyresp.getKeys(keyList=['a','right'], waitRelease=False)
        _keyresp_allKeys.extend(theseKeys)
        if len(_keyresp_allKeys):
            keyresp.keys = _keyresp_allKeys[-1].name  # just the last key pressed
            keyresp.rt = _keyresp_allKeys[-1].rt
    # Run 'Each Frame' code from check_keyresp
    keys = keyresp.getKeys()
    if keys:
        displayText = keys[-1].name
        if displayText == 'left' or displayText == 'lshift':
            text_ = 'L'
        elif displayText == 'right' or displayText == 'rshift':
            text_ = 'R'
        else:
            text_ = "ミスです。左シフトか右シフトボタンを押してください。"
        disp_keyresp.setText(text_)
    
    # *explanation_2* updates
    if explanation_2.status == NOT_STARTED and tThisFlip >= 3-frameTolerance:
        # keep track of start time/frame for later
        explanation_2.frameNStart = frameN  # exact frame index
        explanation_2.tStart = t  # local t and not account for scr refresh
        explanation_2.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(explanation_2, 'tStartRefresh')  # time at next scr refresh
        explanation_2.setAutoDraw(True)
    
    # check for quit (typically the Esc key)
    if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
        core.quit()
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineForceEnded = True
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in ExplanationComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Explanation" ---
for thisComponent in ExplanationComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# store data for thisExp (ExperimentHandler)
thisExp.addData('routine_end_1.x', routine_end_1.x)
thisExp.addData('routine_end_1.y', routine_end_1.y)
thisExp.addData('routine_end_1.leftButton', routine_end_1.leftButton)
thisExp.addData('routine_end_1.midButton', routine_end_1.midButton)
thisExp.addData('routine_end_1.rightButton', routine_end_1.rightButton)
thisExp.addData('routine_end_1.time', routine_end_1.time)
thisExp.nextEntry()
# check responses
if keyresp.keys in ['', [], None]:  # No response was made
    keyresp.keys = None
thisExp.addData('keyresp.keys',keyresp.keys)
if keyresp.keys != None:  # we had a response
    thisExp.addData('keyresp.rt', keyresp.rt)
thisExp.nextEntry()
# the Routine "Explanation" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "Eyecatch" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# keep track of which components have finished
EyecatchComponents = [text]
for thisComponent in EyecatchComponents:
    thisComponent.tStart = None
    thisComponent.tStop = None
    thisComponent.tStartRefresh = None
    thisComponent.tStopRefresh = None
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED
# reset timers
t = 0
_timeToFirstFrame = win.getFutureFlipTime(clock="now")
frameN = -1

# --- Run Routine "Eyecatch" ---
while continueRoutine and routineTimer.getTime() < 1.0:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *text* updates
    if text.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        text.frameNStart = frameN  # exact frame index
        text.tStart = t  # local t and not account for scr refresh
        text.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(text, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.timestampOnFlip(win, 'text.started')
        text.setAutoDraw(True)
    if text.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > text.tStartRefresh + 1-frameTolerance:
            # keep track of stop time/frame for later
            text.tStop = t  # not accounting for scr refresh
            text.frameNStop = frameN  # exact frame index
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'text.stopped')
            text.setAutoDraw(False)
    
    # check for quit (typically the Esc key)
    if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
        core.quit()
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineForceEnded = True
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in EyecatchComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Eyecatch" ---
for thisComponent in EyecatchComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# using non-slip timing so subtract the expected duration of this Routine (unless ended on request)
if routineForceEnded:
    routineTimer.reset()
else:
    routineTimer.addTime(-1.000000)

# set up handler to look after randomisation of conditions etc
trials_4 = data.TrialHandler(nReps=1.0, method='random', 
    extraInfo=expInfo, originPath=-1,
    trialList=[None],
    seed=None, name='trials_4')
thisExp.addLoop(trials_4)  # add the loop to the experiment
thisTrial_4 = trials_4.trialList[0]  # so we can initialise stimuli with some values
# abbreviate parameter names if possible (e.g. rgb = thisTrial_4.rgb)
if thisTrial_4 != None:
    for paramName in thisTrial_4:
        exec('{} = thisTrial_4[paramName]'.format(paramName))

for thisTrial_4 in trials_4:
    currentLoop = trials_4
    # abbreviate parameter names if possible (e.g. rgb = thisTrial_4.rgb)
    if thisTrial_4 != None:
        for paramName in thisTrial_4:
            exec('{} = thisTrial_4[paramName]'.format(paramName))
    
    # --- Prepare to start Routine "Idleoff" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from idle_off
    # stop idle behavior
    json_string = json.dumps(compoff_true)
    message = roslibpy.Message({'data': json_string})
    time.sleep(2)
    dib_topic.publish(message)
    time.sleep(1)
    dib_topic.publish(message)
    time.sleep(1)
    # keep track of which components have finished
    IdleoffComponents = []
    for thisComponent in IdleoffComponents:
        thisComponent.tStart = None
        thisComponent.tStop = None
        thisComponent.tStartRefresh = None
        thisComponent.tStopRefresh = None
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    # reset timers
    t = 0
    _timeToFirstFrame = win.getFutureFlipTime(clock="now")
    frameN = -1
    
    # --- Run Routine "Idleoff" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in IdleoffComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Idleoff" ---
    for thisComponent in IdleoffComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Idleoff" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Eyecatch_demo" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # keep track of which components have finished
    Eyecatch_demoComponents = [text_6]
    for thisComponent in Eyecatch_demoComponents:
        thisComponent.tStart = None
        thisComponent.tStop = None
        thisComponent.tStartRefresh = None
        thisComponent.tStopRefresh = None
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    # reset timers
    t = 0
    _timeToFirstFrame = win.getFutureFlipTime(clock="now")
    frameN = -1
    
    # --- Run Routine "Eyecatch_demo" ---
    while continueRoutine and routineTimer.getTime() < 1.0:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # *text_6* updates
        if text_6.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            text_6.frameNStart = frameN  # exact frame index
            text_6.tStart = t  # local t and not account for scr refresh
            text_6.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(text_6, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'text_6.started')
            text_6.setAutoDraw(True)
        if text_6.status == STARTED:
            # is it time to stop? (based on global clock, using actual start)
            if tThisFlipGlobal > text_6.tStartRefresh + 1.0-frameTolerance:
                # keep track of stop time/frame for later
                text_6.tStop = t  # not accounting for scr refresh
                text_6.frameNStop = frameN  # exact frame index
                # add timestamp to datafile
                thisExp.timestampOnFlip(win, 'text_6.stopped')
                text_6.setAutoDraw(False)
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in Eyecatch_demoComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Eyecatch_demo" ---
    for thisComponent in Eyecatch_demoComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # using non-slip timing so subtract the expected duration of this Routine (unless ended on request)
    if routineForceEnded:
        routineTimer.reset()
    else:
        routineTimer.addTime(-1.000000)
    
    # --- Prepare to start Routine "Demo_init" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from start_pose_demo
    client1.send(head_cmd1_1.encode("utf-8"))
    time.sleep(2)
    # Run 'Begin Routine' code from ikimasu_demo
    json_string = json.dumps(json_data_ikimasu)
    message = roslibpy.Message({'data': json_string})
    time.sleep(1)
    tts_topic.publish(message)
    time.sleep(2)
    # keep track of which components have finished
    Demo_initComponents = []
    for thisComponent in Demo_initComponents:
        thisComponent.tStart = None
        thisComponent.tStop = None
        thisComponent.tStartRefresh = None
        thisComponent.tStopRefresh = None
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    # reset timers
    t = 0
    _timeToFirstFrame = win.getFutureFlipTime(clock="now")
    frameN = -1
    
    # --- Run Routine "Demo_init" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in Demo_initComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Demo_init" ---
    for thisComponent in Demo_initComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Demo_init" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # set up handler to look after randomisation of conditions etc
    trials_3 = data.TrialHandler(nReps=1.0, method='sequential', 
        extraInfo=expInfo, originPath=-1,
        trialList=data.importConditions('xlsx/Random.xlsx'),
        seed=None, name='trials_3')
    thisExp.addLoop(trials_3)  # add the loop to the experiment
    thisTrial_3 = trials_3.trialList[0]  # so we can initialise stimuli with some values
    # abbreviate parameter names if possible (e.g. rgb = thisTrial_3.rgb)
    if thisTrial_3 != None:
        for paramName in thisTrial_3:
            exec('{} = thisTrial_3[paramName]'.format(paramName))
    
    for thisTrial_3 in trials_3:
        currentLoop = trials_3
        # abbreviate parameter names if possible (e.g. rgb = thisTrial_3.rgb)
        if thisTrial_3 != None:
            for paramName in thisTrial_3:
                exec('{} = thisTrial_3[paramName]'.format(paramName))
        
        # --- Prepare to start Routine "Demo_stim" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # Run 'Begin Routine' code from start_pose_demo_2
        client1.send(head_cmd1_1.encode("utf-8"))
        time.sleep(1)
        # Run 'Begin Routine' code from stim_demo
        num_01 = text
        num_02 = prosody
        num_03 = expression
        
        json_data_1 = {
            "Command": "ttsPlaywithParam",
            "TtsID": 1023,
            "Text": stim_txt[num_01],
            "Volume": 1.3,
            "Rate": 1.0,
            "Pitch": 1.0,
            "Emphasis": 1.0,
            "Emotion": {"Joy": 0.0, "Angry": 0.0, "Sad": 0.0}
        }
        json_data_2 = {
            "Command": "ttsPlaywithParam",
            "TtsID": 1023,
            "Text": stim_txt[num_01],
            "Volume": 1.3,
            "Rate": 1.0,
            "Pitch": 1.0,
            "Emphasis": 1.0,
            "Emotion": {"Joy": 0.0, "Angry": 0.0, "Sad": 0.6}
        }
        json_data_3 = {
            "Command": "ttsPlaywithParam",
            "TtsID": 1023,
            "Text": stim_txt[num_01],
            "Volume": 1.3,
            "Rate": 1.0,
            "Pitch": 1.0,
            "Emphasis": 1.0,
            "Emotion": {"Joy": 0.0, "Angry": 0.0, "Sad": 0.7}
        }
        json_data_4 = {
            "Command": "ttsPlaywithParam",
            "TtsID": 1023,
            "Text": stim_txt[num_01],
            "Volume": 1.3,
            "Rate": 1.0,
            "Pitch": 1.0,
            "Emphasis": 1.0,
            "Emotion": {"Joy": 0.6, "Angry": 0.0, "Sad": 0.0}
        }
        json_data_5 = {
            "Command": "ttsPlaywithParam",
            "TtsID": 1023,
            "Text": stim_txt[num_01],
            "Volume": 1.3,
            "Rate": 1.0,
            "Pitch": 1.0,
            "Emphasis": 1.0,
            "Emotion": {"Joy": 0.7, "Angry": 0.0, "Sad": 0.0}
        }
        
        if num_02 == 1:
            json_string = json.dumps(json_data_2)
        elif num_02 == 2:
            json_string = json.dumps(json_data_3)
        elif num_02 == 3:
            json_string = json.dumps(json_data_4)
        elif num_02 == 4:
            json_string = json.dumps(json_data_5)
        elif num_02 == 0:
            json_string = json.dumps(json_data_1)
        # Send messages
        message = roslibpy.Message({'data': json_string})
        
        cmd = stim_exp[num_03]
        time.sleep(2)
        
        def talk_play():
            tts_topic.publish(message)
        
        def face_play():
            client2.send(cmd.encode("utf-8"))
            
        def wait_():
            time.sleep(1)
            
        # スレッドを作成
        thread1 = threading.Thread(target=talk_play)
        thread2 = threading.Thread(target=face_play)
        thread3 = threading.Thread(target=wait_)
        
        # thread1を開始
        thread1.start()
        thread2.start()
        thread3.start()
        
        # スレッドが終了するのを待つ
        thread1.join()
        thread2.join()
        thread3.join()
        # Run 'Begin Routine' code from stim_end_demo
        time.sleep(1)
        
        client1.send(head_cmd1_1.encode("utf-8"))
        client1.send(eye_cmd1_2.encode("utf-8"))
        client2.send(facs_cmd1.encode("utf-8"))
        # keep track of which components have finished
        Demo_stimComponents = []
        for thisComponent in Demo_stimComponents:
            thisComponent.tStart = None
            thisComponent.tStop = None
            thisComponent.tStartRefresh = None
            thisComponent.tStopRefresh = None
            if hasattr(thisComponent, 'status'):
                thisComponent.status = NOT_STARTED
        # reset timers
        t = 0
        _timeToFirstFrame = win.getFutureFlipTime(clock="now")
        frameN = -1
        
        # --- Run Routine "Demo_stim" ---
        while continueRoutine:
            # get current time
            t = routineTimer.getTime()
            tThisFlip = win.getFutureFlipTime(clock=routineTimer)
            tThisFlipGlobal = win.getFutureFlipTime(clock=None)
            frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
            # update/draw components on each frame
            
            # check for quit (typically the Esc key)
            if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
                core.quit()
            
            # check if all components have finished
            if not continueRoutine:  # a component has requested a forced-end of Routine
                routineForceEnded = True
                break
            continueRoutine = False  # will revert to True if at least one component still running
            for thisComponent in Demo_stimComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
        
        # --- Ending Routine "Demo_stim" ---
        for thisComponent in Demo_stimComponents:
            if hasattr(thisComponent, "setAutoDraw"):
                thisComponent.setAutoDraw(False)
        # the Routine "Demo_stim" was not non-slip safe, so reset the non-slip timer
        routineTimer.reset()
        thisExp.nextEntry()
        
    # completed 1.0 repeats of 'trials_3'
    
    
    # --- Prepare to start Routine "Idleon" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from start_idle_2
    json_string = json.dumps(compoff_false)
    
    #topic = roslibpy.Topic(client, '/do_idle_behavior', 'std_msgs/String')
    message = roslibpy.Message({'data': json_string})
    time.sleep(1)
    dib_topic.publish(message)
    time.sleep(1)
    
    client2.send(facs_cmd1.encode("utf-8"))
    
    time.sleep(1)
    # keep track of which components have finished
    IdleonComponents = []
    for thisComponent in IdleonComponents:
        thisComponent.tStart = None
        thisComponent.tStop = None
        thisComponent.tStartRefresh = None
        thisComponent.tStopRefresh = None
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    # reset timers
    t = 0
    _timeToFirstFrame = win.getFutureFlipTime(clock="now")
    frameN = -1
    
    # --- Run Routine "Idleon" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in IdleonComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Idleon" ---
    for thisComponent in IdleonComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Idleon" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    thisExp.nextEntry()
    
# completed 1.0 repeats of 'trials_4'


# set up handler to look after randomisation of conditions etc
trials_5 = data.TrialHandler(nReps=1.0, method='random', 
    extraInfo=expInfo, originPath=-1,
    trialList=[None],
    seed=None, name='trials_5')
thisExp.addLoop(trials_5)  # add the loop to the experiment
thisTrial_5 = trials_5.trialList[0]  # so we can initialise stimuli with some values
# abbreviate parameter names if possible (e.g. rgb = thisTrial_5.rgb)
if thisTrial_5 != None:
    for paramName in thisTrial_5:
        exec('{} = thisTrial_5[paramName]'.format(paramName))

for thisTrial_5 in trials_5:
    currentLoop = trials_5
    # abbreviate parameter names if possible (e.g. rgb = thisTrial_5.rgb)
    if thisTrial_5 != None:
        for paramName in thisTrial_5:
            exec('{} = thisTrial_5[paramName]'.format(paramName))
    
    # --- Prepare to start Routine "Idleoff" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from idle_off
    # stop idle behavior
    json_string = json.dumps(compoff_true)
    message = roslibpy.Message({'data': json_string})
    time.sleep(2)
    dib_topic.publish(message)
    time.sleep(1)
    dib_topic.publish(message)
    time.sleep(1)
    # keep track of which components have finished
    IdleoffComponents = []
    for thisComponent in IdleoffComponents:
        thisComponent.tStart = None
        thisComponent.tStop = None
        thisComponent.tStartRefresh = None
        thisComponent.tStopRefresh = None
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    # reset timers
    t = 0
    _timeToFirstFrame = win.getFutureFlipTime(clock="now")
    frameN = -1
    
    # --- Run Routine "Idleoff" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in IdleoffComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Idleoff" ---
    for thisComponent in IdleoffComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Idleoff" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Start_Countdown" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # keep track of which components have finished
    Start_CountdownComponents = [count_sc]
    for thisComponent in Start_CountdownComponents:
        thisComponent.tStart = None
        thisComponent.tStop = None
        thisComponent.tStartRefresh = None
        thisComponent.tStopRefresh = None
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    # reset timers
    t = 0
    _timeToFirstFrame = win.getFutureFlipTime(clock="now")
    frameN = -1
    
    # --- Run Routine "Start_Countdown" ---
    while continueRoutine and routineTimer.getTime() < 5.0:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # *count_sc* updates
        if count_sc.status == NOT_STARTED and tThisFlip >= 0-frameTolerance:
            # keep track of start time/frame for later
            count_sc.frameNStart = frameN  # exact frame index
            count_sc.tStart = t  # local t and not account for scr refresh
            count_sc.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(count_sc, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'count_sc.started')
            count_sc.setAutoDraw(True)
        if count_sc.status == STARTED:
            # is it time to stop? (based on global clock, using actual start)
            if tThisFlipGlobal > count_sc.tStartRefresh + 5-frameTolerance:
                # keep track of stop time/frame for later
                count_sc.tStop = t  # not accounting for scr refresh
                count_sc.frameNStop = frameN  # exact frame index
                # add timestamp to datafile
                thisExp.timestampOnFlip(win, 'count_sc.stopped')
                count_sc.setAutoDraw(False)
        if count_sc.status == STARTED:  # only update if drawing
            count_sc.setText(str(5-int(t)), log=False)
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in Start_CountdownComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Start_Countdown" ---
    for thisComponent in Start_CountdownComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # using non-slip timing so subtract the expected duration of this Routine (unless ended on request)
    if routineForceEnded:
        routineTimer.reset()
    else:
        routineTimer.addTime(-5.000000)
    
    # set up handler to look after randomisation of conditions etc
    trials_2 = data.TrialHandler(nReps=1.0, method='sequential', 
        extraInfo=expInfo, originPath=-1,
        trialList=data.importConditions('xlsx/exercise.xlsx'),
        seed=None, name='trials_2')
    thisExp.addLoop(trials_2)  # add the loop to the experiment
    thisTrial_2 = trials_2.trialList[0]  # so we can initialise stimuli with some values
    # abbreviate parameter names if possible (e.g. rgb = thisTrial_2.rgb)
    if thisTrial_2 != None:
        for paramName in thisTrial_2:
            exec('{} = thisTrial_2[paramName]'.format(paramName))
    
    for thisTrial_2 in trials_2:
        currentLoop = trials_2
        # abbreviate parameter names if possible (e.g. rgb = thisTrial_2.rgb)
        if thisTrial_2 != None:
            for paramName in thisTrial_2:
                exec('{} = thisTrial_2[paramName]'.format(paramName))
        
        # --- Prepare to start Routine "Test_Eyecatch" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # Run 'Begin Routine' code from code
        text_3.setText(trialCounter)
        trialCounter = trialCounter + 1
        # keep track of which components have finished
        Test_EyecatchComponents = [text_3]
        for thisComponent in Test_EyecatchComponents:
            thisComponent.tStart = None
            thisComponent.tStop = None
            thisComponent.tStartRefresh = None
            thisComponent.tStopRefresh = None
            if hasattr(thisComponent, 'status'):
                thisComponent.status = NOT_STARTED
        # reset timers
        t = 0
        _timeToFirstFrame = win.getFutureFlipTime(clock="now")
        frameN = -1
        
        # --- Run Routine "Test_Eyecatch" ---
        while continueRoutine and routineTimer.getTime() < 1.0:
            # get current time
            t = routineTimer.getTime()
            tThisFlip = win.getFutureFlipTime(clock=routineTimer)
            tThisFlipGlobal = win.getFutureFlipTime(clock=None)
            frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
            # update/draw components on each frame
            
            # *text_3* updates
            if text_3.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                text_3.frameNStart = frameN  # exact frame index
                text_3.tStart = t  # local t and not account for scr refresh
                text_3.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(text_3, 'tStartRefresh')  # time at next scr refresh
                # add timestamp to datafile
                thisExp.timestampOnFlip(win, 'text_3.started')
                text_3.setAutoDraw(True)
            if text_3.status == STARTED:
                # is it time to stop? (based on global clock, using actual start)
                if tThisFlipGlobal > text_3.tStartRefresh + 1.0-frameTolerance:
                    # keep track of stop time/frame for later
                    text_3.tStop = t  # not accounting for scr refresh
                    text_3.frameNStop = frameN  # exact frame index
                    # add timestamp to datafile
                    thisExp.timestampOnFlip(win, 'text_3.stopped')
                    text_3.setAutoDraw(False)
            
            # check for quit (typically the Esc key)
            if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
                core.quit()
            
            # check if all components have finished
            if not continueRoutine:  # a component has requested a forced-end of Routine
                routineForceEnded = True
                break
            continueRoutine = False  # will revert to True if at least one component still running
            for thisComponent in Test_EyecatchComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
        
        # --- Ending Routine "Test_Eyecatch" ---
        for thisComponent in Test_EyecatchComponents:
            if hasattr(thisComponent, "setAutoDraw"):
                thisComponent.setAutoDraw(False)
        # using non-slip timing so subtract the expected duration of this Routine (unless ended on request)
        if routineForceEnded:
            routineTimer.reset()
        else:
            routineTimer.addTime(-1.000000)
        
        # --- Prepare to start Routine "Exercise" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # Run 'Begin Routine' code from idle_off_ex
        # stop idle behavior
        json_string = json.dumps(compoff_true)
        message = roslibpy.Message({'data': json_string})
        time.sleep(1)
        dib_topic.publish(message)
        time.sleep(1)
        
        # eyemove 2 initpose
        client2.send(facs_cmd0.encode("utf-8"))
        time.sleep(1)
        # Run 'Begin Routine' code from Init_pose_ex
        client1.send(head_cmd0_1.encode("utf-8"))
        client1.send(eye_cmd0_2.encode("utf-8"))
        # client2.send(facs_cmd0.encode("utf-8"))
        time.sleep(3)
        # Run 'Begin Routine' code from start_pose_ex
        client1.send(head_cmd1_1.encode("utf-8"))
        time.sleep(2)
        # Run 'Begin Routine' code from ikimasu_ex
        json_string = json.dumps(json_data_ikimasu)
        message = roslibpy.Message({'data': json_string})
        time.sleep(1)
        tts_topic.publish(message)
        time.sleep(2)
        # Run 'Begin Routine' code from ready_pose_ex
        client2.send(facs_cmd1.encode("utf-8"))
        time.sleep(1)
        
        #client1.send(head_cmd3_1.encode("utf-8"))
        client1.send(eye_cmd1_2.encode("utf-8"))
        time.sleep(1)
        #time.sleep(2)
        # Run 'Begin Routine' code from stim_ex
        num_01 = text
        num_02 = prosody
        num_03 = expression
        
        json_data_1 = {
            "Command": "ttsPlaywithParam",
            "TtsID": 1023,
            "Text": stim_txt[num_01],
            "Volume": 1.3,
            "Rate": 1.0,
            "Pitch": 1.0,
            "Emphasis": 1.0,
            "Emotion": {"Joy": 0.0, "Angry": 0.0, "Sad": 0.0}
        }
        json_data_2 = {
            "Command": "ttsPlaywithParam",
            "TtsID": 1023,
            "Text": stim_txt[num_01],
            "Volume": 1.3,
            "Rate": 1.0,
            "Pitch": 1.0,
            "Emphasis": 1.0,
            "Emotion": {"Joy": 0.0, "Angry": 0.0, "Sad": 0.6}
        }
        json_data_3 = {
            "Command": "ttsPlaywithParam",
            "TtsID": 1023,
            "Text": stim_txt[num_01],
            "Volume": 1.3,
            "Rate": 1.0,
            "Pitch": 1.0,
            "Emphasis": 1.0,
            "Emotion": {"Joy": 0.0, "Angry": 0.0, "Sad": 0.7}
        }
        json_data_4 = {
            "Command": "ttsPlaywithParam",
            "TtsID": 1023,
            "Text": stim_txt[num_01],
            "Volume": 1.3,
            "Rate": 1.0,
            "Pitch": 1.0,
            "Emphasis": 1.0,
            "Emotion": {"Joy": 0.6, "Angry": 0.0, "Sad": 0.0}
        }
        json_data_5 = {
            "Command": "ttsPlaywithParam",
            "TtsID": 1023,
            "Text": stim_txt[num_01],
            "Volume": 1.3,
            "Rate": 1.0,
            "Pitch": 1.0,
            "Emphasis": 1.0,
            "Emotion": {"Joy": 0.7, "Angry": 0.0, "Sad": 0.0}
        }
        
        if num_02 == 1:
            json_string = json.dumps(json_data_2)
        elif num_02 == 2:
            json_string = json.dumps(json_data_3)
        elif num_02 == 3:
            json_string = json.dumps(json_data_4)
        elif num_02 == 4:
            json_string = json.dumps(json_data_5)
        elif num_02 == 0:
            json_string = json.dumps(json_data_1)
        # Send messages
        message = roslibpy.Message({'data': json_string})
        
        cmd = stim_exp[num_03]
        time.sleep(2)
        
        def talk_play():
            tts_topic.publish(message)
        
        def face_play():
            client2.send(cmd.encode("utf-8"))
            
        def wait_():
            time.sleep(1)
            
        # make thread
        thread1 = threading.Thread(target=talk_play)
        thread2 = threading.Thread(target=face_play)
        thread3 = threading.Thread(target=wait_)
        
        # start thread
        thread1.start()
        thread2.start()
        thread3.start()
        
        # wait to finish threads
        thread1.join()
        thread2.join()
        thread3.join()
        # Run 'Begin Routine' code from stim_end_ex
        client1.send(head_cmd1_1.encode("utf-8"))
        client1.send(eye_cmd1_2.encode("utf-8"))
        #client2.send(facs_cmd1.encode("utf-8"))
        
        time.sleep(2)
        # Run 'Begin Routine' code from Init_pose_ex_2
        client1.send(head_cmd0_1.encode("utf-8"))
        client2.send(facs_cmd0.encode("utf-8"))
        client1.send(eye_cmd0_2.encode("utf-8"))
        time.sleep(3)
        # keep track of which components have finished
        ExerciseComponents = []
        for thisComponent in ExerciseComponents:
            thisComponent.tStart = None
            thisComponent.tStop = None
            thisComponent.tStartRefresh = None
            thisComponent.tStopRefresh = None
            if hasattr(thisComponent, 'status'):
                thisComponent.status = NOT_STARTED
        # reset timers
        t = 0
        _timeToFirstFrame = win.getFutureFlipTime(clock="now")
        frameN = -1
        
        # --- Run Routine "Exercise" ---
        while continueRoutine:
            # get current time
            t = routineTimer.getTime()
            tThisFlip = win.getFutureFlipTime(clock=routineTimer)
            tThisFlipGlobal = win.getFutureFlipTime(clock=None)
            frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
            # update/draw components on each frame
            
            # check for quit (typically the Esc key)
            if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
                core.quit()
            
            # check if all components have finished
            if not continueRoutine:  # a component has requested a forced-end of Routine
                routineForceEnded = True
                break
            continueRoutine = False  # will revert to True if at least one component still running
            for thisComponent in ExerciseComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
        
        # --- Ending Routine "Exercise" ---
        for thisComponent in ExerciseComponents:
            if hasattr(thisComponent, "setAutoDraw"):
                thisComponent.setAutoDraw(False)
        # the Routine "Exercise" was not non-slip safe, so reset the non-slip timer
        routineTimer.reset()
        
        # --- Prepare to start Routine "Check_v" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        key_resp_2.keys = []
        key_resp_2.rt = []
        _key_resp_2_allKeys = []
        # keep track of which components have finished
        Check_vComponents = [text_4, key_resp_2]
        for thisComponent in Check_vComponents:
            thisComponent.tStart = None
            thisComponent.tStop = None
            thisComponent.tStartRefresh = None
            thisComponent.tStopRefresh = None
            if hasattr(thisComponent, 'status'):
                thisComponent.status = NOT_STARTED
        # reset timers
        t = 0
        _timeToFirstFrame = win.getFutureFlipTime(clock="now")
        frameN = -1
        
        # --- Run Routine "Check_v" ---
        while continueRoutine:
            # get current time
            t = routineTimer.getTime()
            tThisFlip = win.getFutureFlipTime(clock=routineTimer)
            tThisFlipGlobal = win.getFutureFlipTime(clock=None)
            frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
            # update/draw components on each frame
            
            # *text_4* updates
            if text_4.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                text_4.frameNStart = frameN  # exact frame index
                text_4.tStart = t  # local t and not account for scr refresh
                text_4.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(text_4, 'tStartRefresh')  # time at next scr refresh
                text_4.setAutoDraw(True)
            
            # *key_resp_2* updates
            if key_resp_2.status == NOT_STARTED and t >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                key_resp_2.frameNStart = frameN  # exact frame index
                key_resp_2.tStart = t  # local t and not account for scr refresh
                key_resp_2.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(key_resp_2, 'tStartRefresh')  # time at next scr refresh
                key_resp_2.status = STARTED
                # keyboard checking is just starting
                key_resp_2.clock.reset()  # now t=0
                key_resp_2.clearEvents(eventType='keyboard')
            if key_resp_2.status == STARTED:
                theseKeys = key_resp_2.getKeys(keyList=['1','2','3','4','5','6','7','8','9', 'escape'], waitRelease=False)
                _key_resp_2_allKeys.extend(theseKeys)
                if len(_key_resp_2_allKeys):
                    key_resp_2.keys = _key_resp_2_allKeys[-1].name  # just the last key pressed
                    key_resp_2.rt = _key_resp_2_allKeys[-1].rt
                    # a response ends the routine
                    continueRoutine = False
            
            # check for quit (typically the Esc key)
            if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
                core.quit()
            
            # check if all components have finished
            if not continueRoutine:  # a component has requested a forced-end of Routine
                routineForceEnded = True
                break
            continueRoutine = False  # will revert to True if at least one component still running
            for thisComponent in Check_vComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
        
        # --- Ending Routine "Check_v" ---
        for thisComponent in Check_vComponents:
            if hasattr(thisComponent, "setAutoDraw"):
                thisComponent.setAutoDraw(False)
        # check responses
        if key_resp_2.keys in ['', [], None]:  # No response was made
            key_resp_2.keys = None
        trials_2.addData('key_resp_2.keys',key_resp_2.keys)
        if key_resp_2.keys != None:  # we had a response
            trials_2.addData('key_resp_2.rt', key_resp_2.rt)
        # the Routine "Check_v" was not non-slip safe, so reset the non-slip timer
        routineTimer.reset()
        
        # --- Prepare to start Routine "Check_a" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        key_resp_3.keys = []
        key_resp_3.rt = []
        _key_resp_3_allKeys = []
        # keep track of which components have finished
        Check_aComponents = [text_9, key_resp_3]
        for thisComponent in Check_aComponents:
            thisComponent.tStart = None
            thisComponent.tStop = None
            thisComponent.tStartRefresh = None
            thisComponent.tStopRefresh = None
            if hasattr(thisComponent, 'status'):
                thisComponent.status = NOT_STARTED
        # reset timers
        t = 0
        _timeToFirstFrame = win.getFutureFlipTime(clock="now")
        frameN = -1
        
        # --- Run Routine "Check_a" ---
        while continueRoutine:
            # get current time
            t = routineTimer.getTime()
            tThisFlip = win.getFutureFlipTime(clock=routineTimer)
            tThisFlipGlobal = win.getFutureFlipTime(clock=None)
            frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
            # update/draw components on each frame
            
            # *text_9* updates
            if text_9.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                text_9.frameNStart = frameN  # exact frame index
                text_9.tStart = t  # local t and not account for scr refresh
                text_9.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(text_9, 'tStartRefresh')  # time at next scr refresh
                text_9.setAutoDraw(True)
            
            # *key_resp_3* updates
            if key_resp_3.status == NOT_STARTED and t >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                key_resp_3.frameNStart = frameN  # exact frame index
                key_resp_3.tStart = t  # local t and not account for scr refresh
                key_resp_3.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(key_resp_3, 'tStartRefresh')  # time at next scr refresh
                key_resp_3.status = STARTED
                # keyboard checking is just starting
                key_resp_3.clock.reset()  # now t=0
                key_resp_3.clearEvents(eventType='keyboard')
            if key_resp_3.status == STARTED:
                theseKeys = key_resp_3.getKeys(keyList=['1','2','3','4','5','6','7','8','9', 'escape'], waitRelease=False)
                _key_resp_3_allKeys.extend(theseKeys)
                if len(_key_resp_3_allKeys):
                    key_resp_3.keys = _key_resp_3_allKeys[-1].name  # just the last key pressed
                    key_resp_3.rt = _key_resp_3_allKeys[-1].rt
                    # a response ends the routine
                    continueRoutine = False
            
            # check for quit (typically the Esc key)
            if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
                core.quit()
            
            # check if all components have finished
            if not continueRoutine:  # a component has requested a forced-end of Routine
                routineForceEnded = True
                break
            continueRoutine = False  # will revert to True if at least one component still running
            for thisComponent in Check_aComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
        
        # --- Ending Routine "Check_a" ---
        for thisComponent in Check_aComponents:
            if hasattr(thisComponent, "setAutoDraw"):
                thisComponent.setAutoDraw(False)
        # check responses
        if key_resp_3.keys in ['', [], None]:  # No response was made
            key_resp_3.keys = None
        trials_2.addData('key_resp_3.keys',key_resp_3.keys)
        if key_resp_3.keys != None:  # we had a response
            trials_2.addData('key_resp_3.rt', key_resp_3.rt)
        # the Routine "Check_a" was not non-slip safe, so reset the non-slip timer
        routineTimer.reset()
        thisExp.nextEntry()
        
    # completed 1.0 repeats of 'trials_2'
    
    
    # --- Prepare to start Routine "Idleon" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from start_idle_2
    json_string = json.dumps(compoff_false)
    
    #topic = roslibpy.Topic(client, '/do_idle_behavior', 'std_msgs/String')
    message = roslibpy.Message({'data': json_string})
    time.sleep(1)
    dib_topic.publish(message)
    time.sleep(1)
    
    client2.send(facs_cmd1.encode("utf-8"))
    
    time.sleep(1)
    # keep track of which components have finished
    IdleonComponents = []
    for thisComponent in IdleonComponents:
        thisComponent.tStart = None
        thisComponent.tStop = None
        thisComponent.tStartRefresh = None
        thisComponent.tStopRefresh = None
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    # reset timers
    t = 0
    _timeToFirstFrame = win.getFutureFlipTime(clock="now")
    frameN = -1
    
    # --- Run Routine "Idleon" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in IdleonComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Idleon" ---
    for thisComponent in IdleonComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Idleon" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    thisExp.nextEntry()
    
# completed 1.0 repeats of 'trials_5'


# --- Prepare to start Routine "Standby" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# setup some python lists for storing info about the mouse_4
mouse_4.x = []
mouse_4.y = []
mouse_4.leftButton = []
mouse_4.midButton = []
mouse_4.rightButton = []
mouse_4.time = []
gotValidClick = False  # until a click is received
# keep track of which components have finished
StandbyComponents = [text_5, mouse_4]
for thisComponent in StandbyComponents:
    thisComponent.tStart = None
    thisComponent.tStop = None
    thisComponent.tStartRefresh = None
    thisComponent.tStopRefresh = None
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED
# reset timers
t = 0
_timeToFirstFrame = win.getFutureFlipTime(clock="now")
frameN = -1

# --- Run Routine "Standby" ---
while continueRoutine:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *text_5* updates
    if text_5.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        text_5.frameNStart = frameN  # exact frame index
        text_5.tStart = t  # local t and not account for scr refresh
        text_5.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(text_5, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.timestampOnFlip(win, 'text_5.started')
        text_5.setAutoDraw(True)
    # *mouse_4* updates
    if mouse_4.status == NOT_STARTED and t >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        mouse_4.frameNStart = frameN  # exact frame index
        mouse_4.tStart = t  # local t and not account for scr refresh
        mouse_4.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(mouse_4, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.addData('mouse_4.started', t)
        mouse_4.status = STARTED
        mouse_4.mouseClock.reset()
        prevButtonState = mouse_4.getPressed()  # if button is down already this ISN'T a new click
    if mouse_4.status == STARTED:  # only update if started and not finished!
        buttons = mouse_4.getPressed()
        if buttons != prevButtonState:  # button state changed?
            prevButtonState = buttons
            if sum(buttons) > 0:  # state changed to a new click
                x, y = mouse_4.getPos()
                mouse_4.x.append(x)
                mouse_4.y.append(y)
                buttons = mouse_4.getPressed()
                mouse_4.leftButton.append(buttons[0])
                mouse_4.midButton.append(buttons[1])
                mouse_4.rightButton.append(buttons[2])
                mouse_4.time.append(mouse_4.mouseClock.getTime())
                
                continueRoutine = False  # abort routine on response
    
    # check for quit (typically the Esc key)
    if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
        core.quit()
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineForceEnded = True
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in StandbyComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Standby" ---
for thisComponent in StandbyComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# store data for thisExp (ExperimentHandler)
thisExp.addData('mouse_4.x', mouse_4.x)
thisExp.addData('mouse_4.y', mouse_4.y)
thisExp.addData('mouse_4.leftButton', mouse_4.leftButton)
thisExp.addData('mouse_4.midButton', mouse_4.midButton)
thisExp.addData('mouse_4.rightButton', mouse_4.rightButton)
thisExp.addData('mouse_4.time', mouse_4.time)
thisExp.nextEntry()
# the Routine "Standby" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "Idleoff" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# Run 'Begin Routine' code from idle_off
# stop idle behavior
json_string = json.dumps(compoff_true)
message = roslibpy.Message({'data': json_string})
time.sleep(2)
dib_topic.publish(message)
time.sleep(1)
dib_topic.publish(message)
time.sleep(1)
# keep track of which components have finished
IdleoffComponents = []
for thisComponent in IdleoffComponents:
    thisComponent.tStart = None
    thisComponent.tStop = None
    thisComponent.tStartRefresh = None
    thisComponent.tStopRefresh = None
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED
# reset timers
t = 0
_timeToFirstFrame = win.getFutureFlipTime(clock="now")
frameN = -1

# --- Run Routine "Idleoff" ---
while continueRoutine:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # check for quit (typically the Esc key)
    if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
        core.quit()
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineForceEnded = True
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in IdleoffComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Idleoff" ---
for thisComponent in IdleoffComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# the Routine "Idleoff" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "Start_Countdown" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# keep track of which components have finished
Start_CountdownComponents = [count_sc]
for thisComponent in Start_CountdownComponents:
    thisComponent.tStart = None
    thisComponent.tStop = None
    thisComponent.tStartRefresh = None
    thisComponent.tStopRefresh = None
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED
# reset timers
t = 0
_timeToFirstFrame = win.getFutureFlipTime(clock="now")
frameN = -1

# --- Run Routine "Start_Countdown" ---
while continueRoutine and routineTimer.getTime() < 5.0:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *count_sc* updates
    if count_sc.status == NOT_STARTED and tThisFlip >= 0-frameTolerance:
        # keep track of start time/frame for later
        count_sc.frameNStart = frameN  # exact frame index
        count_sc.tStart = t  # local t and not account for scr refresh
        count_sc.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(count_sc, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.timestampOnFlip(win, 'count_sc.started')
        count_sc.setAutoDraw(True)
    if count_sc.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > count_sc.tStartRefresh + 5-frameTolerance:
            # keep track of stop time/frame for later
            count_sc.tStop = t  # not accounting for scr refresh
            count_sc.frameNStop = frameN  # exact frame index
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'count_sc.stopped')
            count_sc.setAutoDraw(False)
    if count_sc.status == STARTED:  # only update if drawing
        count_sc.setText(str(5-int(t)), log=False)
    
    # check for quit (typically the Esc key)
    if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
        core.quit()
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineForceEnded = True
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in Start_CountdownComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Start_Countdown" ---
for thisComponent in Start_CountdownComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# using non-slip timing so subtract the expected duration of this Routine (unless ended on request)
if routineForceEnded:
    routineTimer.reset()
else:
    routineTimer.addTime(-5.000000)

# set up handler to look after randomisation of conditions etc
trials = data.TrialHandler(nReps=1.0, method='random', 
    extraInfo=expInfo, originPath=-1,
    trialList=data.importConditions('xlsx/ResumeMulti.xlsx'),
    seed=None, name='trials')
thisExp.addLoop(trials)  # add the loop to the experiment
thisTrial = trials.trialList[0]  # so we can initialise stimuli with some values
# abbreviate parameter names if possible (e.g. rgb = thisTrial.rgb)
if thisTrial != None:
    for paramName in thisTrial:
        exec('{} = thisTrial[paramName]'.format(paramName))

for thisTrial in trials:
    currentLoop = trials
    # abbreviate parameter names if possible (e.g. rgb = thisTrial.rgb)
    if thisTrial != None:
        for paramName in thisTrial:
            exec('{} = thisTrial[paramName]'.format(paramName))
    
    # --- Prepare to start Routine "Counter" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from break_counter
    if trialCounter % nRestTrial == 0:
        isRest = 1
    else:
        isRest = 0
    
    text_2.setText(trialCounter)
    trialCounter = trialCounter + 1
    
    # keep track of which components have finished
    CounterComponents = [text_2]
    for thisComponent in CounterComponents:
        thisComponent.tStart = None
        thisComponent.tStop = None
        thisComponent.tStartRefresh = None
        thisComponent.tStopRefresh = None
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    # reset timers
    t = 0
    _timeToFirstFrame = win.getFutureFlipTime(clock="now")
    frameN = -1
    
    # --- Run Routine "Counter" ---
    while continueRoutine and routineTimer.getTime() < 1.0:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # *text_2* updates
        if text_2.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            text_2.frameNStart = frameN  # exact frame index
            text_2.tStart = t  # local t and not account for scr refresh
            text_2.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(text_2, 'tStartRefresh')  # time at next scr refresh
            text_2.setAutoDraw(True)
        if text_2.status == STARTED:
            # is it time to stop? (based on global clock, using actual start)
            if tThisFlipGlobal > text_2.tStartRefresh + 1-frameTolerance:
                # keep track of stop time/frame for later
                text_2.tStop = t  # not accounting for scr refresh
                text_2.frameNStop = frameN  # exact frame index
                text_2.setAutoDraw(False)
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in CounterComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Counter" ---
    for thisComponent in CounterComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # using non-slip timing so subtract the expected duration of this Routine (unless ended on request)
    if routineForceEnded:
        routineTimer.reset()
    else:
        routineTimer.addTime(-1.000000)
    
    # set up handler to look after randomisation of conditions etc
    break_ = data.TrialHandler(nReps=isRest, method='random', 
        extraInfo=expInfo, originPath=-1,
        trialList=[None],
        seed=None, name='break_')
    thisExp.addLoop(break_)  # add the loop to the experiment
    thisBreak_ = break_.trialList[0]  # so we can initialise stimuli with some values
    # abbreviate parameter names if possible (e.g. rgb = thisBreak_.rgb)
    if thisBreak_ != None:
        for paramName in thisBreak_:
            exec('{} = thisBreak_[paramName]'.format(paramName))
    
    for thisBreak_ in break_:
        currentLoop = break_
        # abbreviate parameter names if possible (e.g. rgb = thisBreak_.rgb)
        if thisBreak_ != None:
            for paramName in thisBreak_:
                exec('{} = thisBreak_[paramName]'.format(paramName))
        
        # --- Prepare to start Routine "Idleon" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # Run 'Begin Routine' code from start_idle_2
        json_string = json.dumps(compoff_false)
        
        #topic = roslibpy.Topic(client, '/do_idle_behavior', 'std_msgs/String')
        message = roslibpy.Message({'data': json_string})
        time.sleep(1)
        dib_topic.publish(message)
        time.sleep(1)
        
        client2.send(facs_cmd1.encode("utf-8"))
        
        time.sleep(1)
        # keep track of which components have finished
        IdleonComponents = []
        for thisComponent in IdleonComponents:
            thisComponent.tStart = None
            thisComponent.tStop = None
            thisComponent.tStartRefresh = None
            thisComponent.tStopRefresh = None
            if hasattr(thisComponent, 'status'):
                thisComponent.status = NOT_STARTED
        # reset timers
        t = 0
        _timeToFirstFrame = win.getFutureFlipTime(clock="now")
        frameN = -1
        
        # --- Run Routine "Idleon" ---
        while continueRoutine:
            # get current time
            t = routineTimer.getTime()
            tThisFlip = win.getFutureFlipTime(clock=routineTimer)
            tThisFlipGlobal = win.getFutureFlipTime(clock=None)
            frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
            # update/draw components on each frame
            
            # check for quit (typically the Esc key)
            if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
                core.quit()
            
            # check if all components have finished
            if not continueRoutine:  # a component has requested a forced-end of Routine
                routineForceEnded = True
                break
            continueRoutine = False  # will revert to True if at least one component still running
            for thisComponent in IdleonComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
        
        # --- Ending Routine "Idleon" ---
        for thisComponent in IdleonComponents:
            if hasattr(thisComponent, "setAutoDraw"):
                thisComponent.setAutoDraw(False)
        # the Routine "Idleon" was not non-slip safe, so reset the non-slip timer
        routineTimer.reset()
        
        # --- Prepare to start Routine "Routine" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # setup some python lists for storing info about the mouse_3
        mouse_3.x = []
        mouse_3.y = []
        mouse_3.leftButton = []
        mouse_3.midButton = []
        mouse_3.rightButton = []
        mouse_3.time = []
        gotValidClick = False  # until a click is received
        # keep track of which components have finished
        RoutineComponents = [text_countdown, text_7, mouse_3]
        for thisComponent in RoutineComponents:
            thisComponent.tStart = None
            thisComponent.tStop = None
            thisComponent.tStartRefresh = None
            thisComponent.tStopRefresh = None
            if hasattr(thisComponent, 'status'):
                thisComponent.status = NOT_STARTED
        # reset timers
        t = 0
        _timeToFirstFrame = win.getFutureFlipTime(clock="now")
        frameN = -1
        
        # --- Run Routine "Routine" ---
        while continueRoutine:
            # get current time
            t = routineTimer.getTime()
            tThisFlip = win.getFutureFlipTime(clock=routineTimer)
            tThisFlipGlobal = win.getFutureFlipTime(clock=None)
            frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
            # update/draw components on each frame
            
            # *text_countdown* updates
            if text_countdown.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                text_countdown.frameNStart = frameN  # exact frame index
                text_countdown.tStart = t  # local t and not account for scr refresh
                text_countdown.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(text_countdown, 'tStartRefresh')  # time at next scr refresh
                # add timestamp to datafile
                thisExp.timestampOnFlip(win, 'text_countdown.started')
                text_countdown.setAutoDraw(True)
            if text_countdown.status == STARTED:
                # is it time to stop? (based on global clock, using actual start)
                if tThisFlipGlobal > text_countdown.tStartRefresh + 300-frameTolerance:
                    # keep track of stop time/frame for later
                    text_countdown.tStop = t  # not accounting for scr refresh
                    text_countdown.frameNStop = frameN  # exact frame index
                    # add timestamp to datafile
                    thisExp.timestampOnFlip(win, 'text_countdown.stopped')
                    text_countdown.setAutoDraw(False)
            if text_countdown.status == STARTED:  # only update if drawing
                text_countdown.setText(str(300-int(t)), log=False)
            
            # *text_7* updates
            if text_7.status == NOT_STARTED and tThisFlip >= 300.0-frameTolerance:
                # keep track of start time/frame for later
                text_7.frameNStart = frameN  # exact frame index
                text_7.tStart = t  # local t and not account for scr refresh
                text_7.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(text_7, 'tStartRefresh')  # time at next scr refresh
                # add timestamp to datafile
                thisExp.timestampOnFlip(win, 'text_7.started')
                text_7.setAutoDraw(True)
            # *mouse_3* updates
            if mouse_3.status == NOT_STARTED and t >= 0-frameTolerance:
                # keep track of start time/frame for later
                mouse_3.frameNStart = frameN  # exact frame index
                mouse_3.tStart = t  # local t and not account for scr refresh
                mouse_3.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(mouse_3, 'tStartRefresh')  # time at next scr refresh
                # add timestamp to datafile
                thisExp.addData('mouse_3.started', t)
                mouse_3.status = STARTED
                mouse_3.mouseClock.reset()
                prevButtonState = mouse_3.getPressed()  # if button is down already this ISN'T a new click
            if mouse_3.status == STARTED:  # only update if started and not finished!
                buttons = mouse_3.getPressed()
                if buttons != prevButtonState:  # button state changed?
                    prevButtonState = buttons
                    if sum(buttons) > 0:  # state changed to a new click
                        x, y = mouse_3.getPos()
                        mouse_3.x.append(x)
                        mouse_3.y.append(y)
                        buttons = mouse_3.getPressed()
                        mouse_3.leftButton.append(buttons[0])
                        mouse_3.midButton.append(buttons[1])
                        mouse_3.rightButton.append(buttons[2])
                        mouse_3.time.append(mouse_3.mouseClock.getTime())
                        
                        continueRoutine = False  # abort routine on response
            
            # check for quit (typically the Esc key)
            if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
                core.quit()
            
            # check if all components have finished
            if not continueRoutine:  # a component has requested a forced-end of Routine
                routineForceEnded = True
                break
            continueRoutine = False  # will revert to True if at least one component still running
            for thisComponent in RoutineComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
        
        # --- Ending Routine "Routine" ---
        for thisComponent in RoutineComponents:
            if hasattr(thisComponent, "setAutoDraw"):
                thisComponent.setAutoDraw(False)
        # store data for break_ (TrialHandler)
        break_.addData('mouse_3.x', mouse_3.x)
        break_.addData('mouse_3.y', mouse_3.y)
        break_.addData('mouse_3.leftButton', mouse_3.leftButton)
        break_.addData('mouse_3.midButton', mouse_3.midButton)
        break_.addData('mouse_3.rightButton', mouse_3.rightButton)
        break_.addData('mouse_3.time', mouse_3.time)
        # the Routine "Routine" was not non-slip safe, so reset the non-slip timer
        routineTimer.reset()
        
        # --- Prepare to start Routine "Idleoff" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # Run 'Begin Routine' code from idle_off
        # stop idle behavior
        json_string = json.dumps(compoff_true)
        message = roslibpy.Message({'data': json_string})
        time.sleep(2)
        dib_topic.publish(message)
        time.sleep(1)
        dib_topic.publish(message)
        time.sleep(1)
        # keep track of which components have finished
        IdleoffComponents = []
        for thisComponent in IdleoffComponents:
            thisComponent.tStart = None
            thisComponent.tStop = None
            thisComponent.tStartRefresh = None
            thisComponent.tStopRefresh = None
            if hasattr(thisComponent, 'status'):
                thisComponent.status = NOT_STARTED
        # reset timers
        t = 0
        _timeToFirstFrame = win.getFutureFlipTime(clock="now")
        frameN = -1
        
        # --- Run Routine "Idleoff" ---
        while continueRoutine:
            # get current time
            t = routineTimer.getTime()
            tThisFlip = win.getFutureFlipTime(clock=routineTimer)
            tThisFlipGlobal = win.getFutureFlipTime(clock=None)
            frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
            # update/draw components on each frame
            
            # check for quit (typically the Esc key)
            if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
                core.quit()
            
            # check if all components have finished
            if not continueRoutine:  # a component has requested a forced-end of Routine
                routineForceEnded = True
                break
            continueRoutine = False  # will revert to True if at least one component still running
            for thisComponent in IdleoffComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
        
        # --- Ending Routine "Idleoff" ---
        for thisComponent in IdleoffComponents:
            if hasattr(thisComponent, "setAutoDraw"):
                thisComponent.setAutoDraw(False)
        # the Routine "Idleoff" was not non-slip safe, so reset the non-slip timer
        routineTimer.reset()
        thisExp.nextEntry()
        
    # completed isRest repeats of 'break_'
    
    
    # --- Prepare to start Routine "Exercise" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from idle_off_ex
    # stop idle behavior
    json_string = json.dumps(compoff_true)
    message = roslibpy.Message({'data': json_string})
    time.sleep(1)
    dib_topic.publish(message)
    time.sleep(1)
    
    # eyemove 2 initpose
    client2.send(facs_cmd0.encode("utf-8"))
    time.sleep(1)
    # Run 'Begin Routine' code from Init_pose_ex
    client1.send(head_cmd0_1.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    # client2.send(facs_cmd0.encode("utf-8"))
    time.sleep(3)
    # Run 'Begin Routine' code from start_pose_ex
    client1.send(head_cmd1_1.encode("utf-8"))
    time.sleep(2)
    # Run 'Begin Routine' code from ikimasu_ex
    json_string = json.dumps(json_data_ikimasu)
    message = roslibpy.Message({'data': json_string})
    time.sleep(1)
    tts_topic.publish(message)
    time.sleep(2)
    # Run 'Begin Routine' code from ready_pose_ex
    client2.send(facs_cmd1.encode("utf-8"))
    time.sleep(1)
    
    #client1.send(head_cmd3_1.encode("utf-8"))
    client1.send(eye_cmd1_2.encode("utf-8"))
    time.sleep(1)
    #time.sleep(2)
    # Run 'Begin Routine' code from stim_ex
    num_01 = text
    num_02 = prosody
    num_03 = expression
    
    json_data_1 = {
        "Command": "ttsPlaywithParam",
        "TtsID": 1023,
        "Text": stim_txt[num_01],
        "Volume": 1.3,
        "Rate": 1.0,
        "Pitch": 1.0,
        "Emphasis": 1.0,
        "Emotion": {"Joy": 0.0, "Angry": 0.0, "Sad": 0.0}
    }
    json_data_2 = {
        "Command": "ttsPlaywithParam",
        "TtsID": 1023,
        "Text": stim_txt[num_01],
        "Volume": 1.3,
        "Rate": 1.0,
        "Pitch": 1.0,
        "Emphasis": 1.0,
        "Emotion": {"Joy": 0.0, "Angry": 0.0, "Sad": 0.6}
    }
    json_data_3 = {
        "Command": "ttsPlaywithParam",
        "TtsID": 1023,
        "Text": stim_txt[num_01],
        "Volume": 1.3,
        "Rate": 1.0,
        "Pitch": 1.0,
        "Emphasis": 1.0,
        "Emotion": {"Joy": 0.0, "Angry": 0.0, "Sad": 0.7}
    }
    json_data_4 = {
        "Command": "ttsPlaywithParam",
        "TtsID": 1023,
        "Text": stim_txt[num_01],
        "Volume": 1.3,
        "Rate": 1.0,
        "Pitch": 1.0,
        "Emphasis": 1.0,
        "Emotion": {"Joy": 0.6, "Angry": 0.0, "Sad": 0.0}
    }
    json_data_5 = {
        "Command": "ttsPlaywithParam",
        "TtsID": 1023,
        "Text": stim_txt[num_01],
        "Volume": 1.3,
        "Rate": 1.0,
        "Pitch": 1.0,
        "Emphasis": 1.0,
        "Emotion": {"Joy": 0.7, "Angry": 0.0, "Sad": 0.0}
    }
    
    if num_02 == 1:
        json_string = json.dumps(json_data_2)
    elif num_02 == 2:
        json_string = json.dumps(json_data_3)
    elif num_02 == 3:
        json_string = json.dumps(json_data_4)
    elif num_02 == 4:
        json_string = json.dumps(json_data_5)
    elif num_02 == 0:
        json_string = json.dumps(json_data_1)
    # Send messages
    message = roslibpy.Message({'data': json_string})
    
    cmd = stim_exp[num_03]
    time.sleep(2)
    
    def talk_play():
        tts_topic.publish(message)
    
    def face_play():
        client2.send(cmd.encode("utf-8"))
        
    def wait_():
        time.sleep(1)
        
    # make thread
    thread1 = threading.Thread(target=talk_play)
    thread2 = threading.Thread(target=face_play)
    thread3 = threading.Thread(target=wait_)
    
    # start thread
    thread1.start()
    thread2.start()
    thread3.start()
    
    # wait to finish threads
    thread1.join()
    thread2.join()
    thread3.join()
    # Run 'Begin Routine' code from stim_end_ex
    client1.send(head_cmd1_1.encode("utf-8"))
    client1.send(eye_cmd1_2.encode("utf-8"))
    #client2.send(facs_cmd1.encode("utf-8"))
    
    time.sleep(2)
    # Run 'Begin Routine' code from Init_pose_ex_2
    client1.send(head_cmd0_1.encode("utf-8"))
    client2.send(facs_cmd0.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    time.sleep(3)
    # keep track of which components have finished
    ExerciseComponents = []
    for thisComponent in ExerciseComponents:
        thisComponent.tStart = None
        thisComponent.tStop = None
        thisComponent.tStartRefresh = None
        thisComponent.tStopRefresh = None
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    # reset timers
    t = 0
    _timeToFirstFrame = win.getFutureFlipTime(clock="now")
    frameN = -1
    
    # --- Run Routine "Exercise" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in ExerciseComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Exercise" ---
    for thisComponent in ExerciseComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Exercise" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Check_v" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    key_resp_2.keys = []
    key_resp_2.rt = []
    _key_resp_2_allKeys = []
    # keep track of which components have finished
    Check_vComponents = [text_4, key_resp_2]
    for thisComponent in Check_vComponents:
        thisComponent.tStart = None
        thisComponent.tStop = None
        thisComponent.tStartRefresh = None
        thisComponent.tStopRefresh = None
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    # reset timers
    t = 0
    _timeToFirstFrame = win.getFutureFlipTime(clock="now")
    frameN = -1
    
    # --- Run Routine "Check_v" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # *text_4* updates
        if text_4.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            text_4.frameNStart = frameN  # exact frame index
            text_4.tStart = t  # local t and not account for scr refresh
            text_4.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(text_4, 'tStartRefresh')  # time at next scr refresh
            text_4.setAutoDraw(True)
        
        # *key_resp_2* updates
        if key_resp_2.status == NOT_STARTED and t >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            key_resp_2.frameNStart = frameN  # exact frame index
            key_resp_2.tStart = t  # local t and not account for scr refresh
            key_resp_2.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(key_resp_2, 'tStartRefresh')  # time at next scr refresh
            key_resp_2.status = STARTED
            # keyboard checking is just starting
            key_resp_2.clock.reset()  # now t=0
            key_resp_2.clearEvents(eventType='keyboard')
        if key_resp_2.status == STARTED:
            theseKeys = key_resp_2.getKeys(keyList=['1','2','3','4','5','6','7','8','9', 'escape'], waitRelease=False)
            _key_resp_2_allKeys.extend(theseKeys)
            if len(_key_resp_2_allKeys):
                key_resp_2.keys = _key_resp_2_allKeys[-1].name  # just the last key pressed
                key_resp_2.rt = _key_resp_2_allKeys[-1].rt
                # a response ends the routine
                continueRoutine = False
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in Check_vComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Check_v" ---
    for thisComponent in Check_vComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # check responses
    if key_resp_2.keys in ['', [], None]:  # No response was made
        key_resp_2.keys = None
    trials.addData('key_resp_2.keys',key_resp_2.keys)
    if key_resp_2.keys != None:  # we had a response
        trials.addData('key_resp_2.rt', key_resp_2.rt)
    # the Routine "Check_v" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Check_a" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    key_resp_3.keys = []
    key_resp_3.rt = []
    _key_resp_3_allKeys = []
    # keep track of which components have finished
    Check_aComponents = [text_9, key_resp_3]
    for thisComponent in Check_aComponents:
        thisComponent.tStart = None
        thisComponent.tStop = None
        thisComponent.tStartRefresh = None
        thisComponent.tStopRefresh = None
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    # reset timers
    t = 0
    _timeToFirstFrame = win.getFutureFlipTime(clock="now")
    frameN = -1
    
    # --- Run Routine "Check_a" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # *text_9* updates
        if text_9.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            text_9.frameNStart = frameN  # exact frame index
            text_9.tStart = t  # local t and not account for scr refresh
            text_9.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(text_9, 'tStartRefresh')  # time at next scr refresh
            text_9.setAutoDraw(True)
        
        # *key_resp_3* updates
        if key_resp_3.status == NOT_STARTED and t >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            key_resp_3.frameNStart = frameN  # exact frame index
            key_resp_3.tStart = t  # local t and not account for scr refresh
            key_resp_3.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(key_resp_3, 'tStartRefresh')  # time at next scr refresh
            key_resp_3.status = STARTED
            # keyboard checking is just starting
            key_resp_3.clock.reset()  # now t=0
            key_resp_3.clearEvents(eventType='keyboard')
        if key_resp_3.status == STARTED:
            theseKeys = key_resp_3.getKeys(keyList=['1','2','3','4','5','6','7','8','9', 'escape'], waitRelease=False)
            _key_resp_3_allKeys.extend(theseKeys)
            if len(_key_resp_3_allKeys):
                key_resp_3.keys = _key_resp_3_allKeys[-1].name  # just the last key pressed
                key_resp_3.rt = _key_resp_3_allKeys[-1].rt
                # a response ends the routine
                continueRoutine = False
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in Check_aComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Check_a" ---
    for thisComponent in Check_aComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # check responses
    if key_resp_3.keys in ['', [], None]:  # No response was made
        key_resp_3.keys = None
    trials.addData('key_resp_3.keys',key_resp_3.keys)
    if key_resp_3.keys != None:  # we had a response
        trials.addData('key_resp_3.rt', key_resp_3.rt)
    # the Routine "Check_a" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    thisExp.nextEntry()
    
# completed 1.0 repeats of 'trials'


# --- Prepare to start Routine "Idleon" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# Run 'Begin Routine' code from start_idle_2
json_string = json.dumps(compoff_false)

#topic = roslibpy.Topic(client, '/do_idle_behavior', 'std_msgs/String')
message = roslibpy.Message({'data': json_string})
time.sleep(1)
dib_topic.publish(message)
time.sleep(1)

client2.send(facs_cmd1.encode("utf-8"))

time.sleep(1)
# keep track of which components have finished
IdleonComponents = []
for thisComponent in IdleonComponents:
    thisComponent.tStart = None
    thisComponent.tStop = None
    thisComponent.tStartRefresh = None
    thisComponent.tStopRefresh = None
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED
# reset timers
t = 0
_timeToFirstFrame = win.getFutureFlipTime(clock="now")
frameN = -1

# --- Run Routine "Idleon" ---
while continueRoutine:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # check for quit (typically the Esc key)
    if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
        core.quit()
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineForceEnded = True
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in IdleonComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Idleon" ---
for thisComponent in IdleonComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# the Routine "Idleon" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "finish" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# setup some python lists for storing info about the mouse_2
mouse_2.x = []
mouse_2.y = []
mouse_2.leftButton = []
mouse_2.midButton = []
mouse_2.rightButton = []
mouse_2.time = []
gotValidClick = False  # until a click is received
# keep track of which components have finished
finishComponents = [text_8, mouse_2]
for thisComponent in finishComponents:
    thisComponent.tStart = None
    thisComponent.tStop = None
    thisComponent.tStartRefresh = None
    thisComponent.tStopRefresh = None
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED
# reset timers
t = 0
_timeToFirstFrame = win.getFutureFlipTime(clock="now")
frameN = -1

# --- Run Routine "finish" ---
while continueRoutine:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *text_8* updates
    if text_8.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        text_8.frameNStart = frameN  # exact frame index
        text_8.tStart = t  # local t and not account for scr refresh
        text_8.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(text_8, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.timestampOnFlip(win, 'text_8.started')
        text_8.setAutoDraw(True)
    if text_8.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > text_8.tStartRefresh + 10-frameTolerance:
            # keep track of stop time/frame for later
            text_8.tStop = t  # not accounting for scr refresh
            text_8.frameNStop = frameN  # exact frame index
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'text_8.stopped')
            text_8.setAutoDraw(False)
    # *mouse_2* updates
    if mouse_2.status == NOT_STARTED and t >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        mouse_2.frameNStart = frameN  # exact frame index
        mouse_2.tStart = t  # local t and not account for scr refresh
        mouse_2.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(mouse_2, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.addData('mouse_2.started', t)
        mouse_2.status = STARTED
        mouse_2.mouseClock.reset()
        prevButtonState = mouse_2.getPressed()  # if button is down already this ISN'T a new click
    if mouse_2.status == STARTED:  # only update if started and not finished!
        buttons = mouse_2.getPressed()
        if buttons != prevButtonState:  # button state changed?
            prevButtonState = buttons
            if sum(buttons) > 0:  # state changed to a new click
                x, y = mouse_2.getPos()
                mouse_2.x.append(x)
                mouse_2.y.append(y)
                buttons = mouse_2.getPressed()
                mouse_2.leftButton.append(buttons[0])
                mouse_2.midButton.append(buttons[1])
                mouse_2.rightButton.append(buttons[2])
                mouse_2.time.append(mouse_2.mouseClock.getTime())
                
                continueRoutine = False  # abort routine on response
    
    # check for quit (typically the Esc key)
    if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
        core.quit()
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineForceEnded = True
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in finishComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "finish" ---
for thisComponent in finishComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# store data for thisExp (ExperimentHandler)
thisExp.addData('mouse_2.x', mouse_2.x)
thisExp.addData('mouse_2.y', mouse_2.y)
thisExp.addData('mouse_2.leftButton', mouse_2.leftButton)
thisExp.addData('mouse_2.midButton', mouse_2.midButton)
thisExp.addData('mouse_2.rightButton', mouse_2.rightButton)
thisExp.addData('mouse_2.time', mouse_2.time)
thisExp.nextEntry()
# the Routine "finish" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()
# Run 'End Experiment' code from init_variable
# Define JSON data
json_data = {
    "completely_off": False
}
json_string = json.dumps(json_data)

# Define topic and message types
topic = roslibpy.Topic(client, '/do_idle_behavior', 'std_msgs/String')

# Send messages
message = roslibpy.Message({'data': json_string})
# time.sleep(1)
topic.publish(message)
# time.sleep(1)

# Terminate connection
client.terminate()

# socket client close
client1.close()
client2.close()
# Run 'End Experiment' code from init_variable_ex
# Define JSON data
json_data = {
    "completely_off": False
}
json_string = json.dumps(json_data)

# Define topic and message types
topic = roslibpy.Topic(client, '/do_idle_behavior', 'std_msgs/String')

# Send messages
message = roslibpy.Message({'data': json_string})
# time.sleep(1)
topic.publish(message)
# time.sleep(1)

# Terminate connection
client.terminate()

# socket client close
client1.close()
client2.close()
# Run 'End Experiment' code from init_variable_ex
# Define JSON data
json_data = {
    "completely_off": False
}
json_string = json.dumps(json_data)

# Define topic and message types
topic = roslibpy.Topic(client, '/do_idle_behavior', 'std_msgs/String')

# Send messages
message = roslibpy.Message({'data': json_string})
# time.sleep(1)
topic.publish(message)
# time.sleep(1)

# Terminate connection
client.terminate()

# socket client close
client1.close()
client2.close()

# --- End experiment ---
# Flip one final time so any remaining win.callOnFlip() 
# and win.timeOnFlip() tasks get executed before quitting
win.flip()

# these shouldn't be strictly necessary (should auto-save)
thisExp.saveAsWideText(filename+'.csv', delim='auto')
thisExp.saveAsPickle(filename)
logging.flush()
# make sure everything is closed down
if eyetracker:
    eyetracker.setConnectionState(False)
thisExp.abort()  # or data files will save again on exit
win.close()
core.quit()
