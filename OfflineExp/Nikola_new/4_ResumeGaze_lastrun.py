#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This experiment was created using PsychoPy3 Experiment Builder (v2022.2.5),
    on 12月 05, 2023, at 13:56
If you publish work using this script the most relevant publication is:

    Peirce J, Gray JR, Simpson S, MacAskill M, Höchenberger R, Sogo H, Kastman E, Lindeløv JK. (2019) 
        PsychoPy2: Experiments in behavior made easy Behav Res 51: 195. 
        https://doi.org/10.3758/s13428-018-01193-y

"""

# --- Import packages ---
from psychopy import locale_setup
from psychopy import prefs
from psychopy import sound, gui, visual, core, data, event, logging, clock, colors, layout, hardware
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

# Run 'Before Experiment' code from init_var
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

json_idleoff = json.dumps(compoff_true)
json_idleon = json.dumps(compoff_false)

message_idleoff = roslibpy.Message({'data': json_idleoff})
message_idleon = roslibpy.Message({'data': json_idleon})
# Run 'Before Experiment' code from init_cmd
# HEAD
head_cmd0_1 = "HeadController={\"id\":\"HeadController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
  \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \"translateSpeed\":1.0}\n"
head_cmd0_2 = "HeadController={\"id\":\"HeadController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
  \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \"translateSpeed\":2.0}\n"            
head_cmd0_4 = "HeadController={\"id\":\"HeadController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
  \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \"translateSpeed\":4.0}\n"

head_cmd1_1 = "HeadController={\"id\":\"HeadController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
  \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \"translateSpeed\":1.0}\n"
head_cmd1_2 = "HeadController={\"id\":\"HeadController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
  \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \"translateSpeed\":2.0}\n"
head_cmd1_4 = "HeadController={\"id\":\"HeadController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
  \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \"translateSpeed\":4.0}\n"

head_cmd_rm = "HeadController={\"id\":\"HeadController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
  \"targetPoint\":{\"x\":0.0,\"y\":0.1,\"z\":1.5}, \"translateSpeed\":2.0}\n"

# EYE
eye_cmd0_1 = "EyeController={\"id\":\"EyeController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \"translateSpeed\":1.0}\n"

eye_cmd0_2 = "EyeController={\"id\":\"EyeController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \"translateSpeed\":2.0}\n"
            
eye_cmd0_4 = "EyeController={\"id\":\"EyeController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \"translateSpeed\":4.0}\n"
            
eye_cmd0_8 = "EyeController={\"id\":\"EyeController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":0.5,\"z\":1.5}, \"translateSpeed\":8.0}\n"

eye_cmd1_1 = "EyeController={\"id\":\"EyeController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \"translateSpeed\":1.0}\n"

eye_cmd1_2 = "EyeController={\"id\":\"EyeController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \"translateSpeed\":2.0}\n"
            
eye_cmd1_4 = "EyeController={\"id\":\"EyeController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \"translateSpeed\":4.0}\n"
            
eye_cmd1_8 = "EyeController={\"id\":\"EyeController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \"translateSpeed\":8.0}\n"

# FACS
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
# Run 'Before Experiment' code from init_arduino
import serial

ser = serial.Serial("COM3", 9600)
# Run 'Before Experiment' code from stim_test
headcmd = ["HeadController={\"id\":\"HeadController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5},\"translateSpeed\":2}\n", \
            "HeadController={\"id\":\"HeadController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":0.7,\"y\":0.7,\"z\":1.5},\"translateSpeed\":2}\n", \
            "HeadController={\"id\":\"HeadController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":-0.7,\"y\":0.7,\"z\":1.5},\"translateSpeed\":2}\n"]

eyecmd = ["EyeController={\"id\":\"EyeController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5},\"translateSpeed\":8}\n", \
            "EyeController={\"id\":\"EyeController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":0.7,\"y\":0.7,\"z\":1.5},\"translateSpeed\":8}\n", \
            "EyeController={\"id\":\"EyeController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":-0.7,\"y\":0.7,\"z\":1.5},\"translateSpeed\":8}\n"]
try:  # to use the Cedrus response box
   import pyxid2 as pyxid
except ImportError:
   import pyxid
cedrusBox_0 = None
for n in range(10):  # doesn't always work first time!
    try:
        devices = pyxid.get_xid_devices()
        core.wait(0.1)
        cedrusBox_0 = devices[0]
        cedrusBox_0.clock = core.Clock()
        break  # found the device so can break the loop
    except Exception:
        pass
if not cedrusBox_0:
    logging.error('could not find a Cedrus device.')
    core.quit()
# Run 'Before Experiment' code from stim_test
headcmd = ["HeadController={\"id\":\"HeadController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5},\"translateSpeed\":2}\n", \
            "HeadController={\"id\":\"HeadController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":0.7,\"y\":0.7,\"z\":1.5},\"translateSpeed\":2}\n", \
            "HeadController={\"id\":\"HeadController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":-0.7,\"y\":0.7,\"z\":1.5},\"translateSpeed\":2}\n"]

eyecmd = ["EyeController={\"id\":\"EyeController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5},\"translateSpeed\":8}\n", \
            "EyeController={\"id\":\"EyeController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":0.7,\"y\":0.7,\"z\":1.5},\"translateSpeed\":8}\n", \
            "EyeController={\"id\":\"EyeController\",\"motionTowardObject\":\"\",\"targetMotionMode\":2,\"targetPoint\":{\"x\":-0.7,\"y\":0.7,\"z\":1.5},\"translateSpeed\":8}\n"]


# Ensure that relative paths start from the same directory as this script
_thisDir = os.path.dirname(os.path.abspath(__file__))
os.chdir(_thisDir)
# Store info about the experiment session
psychopyVersion = '2022.2.5'
expName = 'Resume Gaze'  # from the Builder filename that created this script
expInfo = {
    'participant': f"{randint(0, 999999):06.0f}",
    'subj': '001',
}
# --- Show participant info dialog --
dlg = gui.DlgFromDict(dictionary=expInfo, sortKeys=False, title=expName)
if dlg.OK == False:
    core.quit()  # user pressed cancel
expInfo['date'] = data.getDateStr()  # add a simple timestamp
expInfo['expName'] = expName
expInfo['psychopyVersion'] = psychopyVersion

# Data file name stem = absolute path + name; later add .psyexp, .csv, .log, etc
filename = _thisDir + os.sep + u'data/%s_%s_%s_%s' % (expInfo['subj'], '4-Gaze', expInfo['participant'], expInfo['date'])

# An ExperimentHandler isn't essential but helps with data saving
thisExp = data.ExperimentHandler(name=expName, version='',
    extraInfo=expInfo, runtimeInfo=None,
    originPath='C:\\Users\\SATO\\OneDrive\\Coding\\Dr.Sato\\exp_202311\\4_ResumeGaze_lastrun.py',
    savePickle=True, saveWideText=True,
    dataFileName=filename)
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
init_msg = visual.TextStim(win=win, name='init_msg',
    text='実験プログラムを起動しています\n暫くお待ちください',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
# Run 'Begin Experiment' code from init_talk
# Volume
Master_Volume = 1.3

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
json_ikimasu = json.dumps(json_data_ikimasu)
message_ikimasu = roslibpy.Message({'data': json_ikimasu})

json_data_tuduki = {
    "Command": "ttsPlaywithParam",
    "TtsID": 1023,
    "Text": "さいごのひょうじょうをえらんでください。",
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
json_tuduki = json.dumps(json_data_tuduki)
message_tuduki = roslibpy.Message({'data': json_tuduki})

json_data_ichi = {
    "Command": "ttsPlaywithParam",
    "TtsID": 1023,
    "Text": "いち",
    "Volume": Master_Volume,
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

json_data_ni = {
    "Command": "ttsPlaywithParam",
    "TtsID": 1023,
    "Text": "に",
    "Volume": Master_Volume,
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

json_data_san = {
    "Command": "ttsPlaywithParam",
    "TtsID": 1023,
    "Text": "さん",
    "Volume": Master_Volume,
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

# --- Initialize components for Routine "Outline" ---
outline_1 = visual.TextStim(win=win, name='outline_1',
    text='- 実験の説明 -',
    font='Open Sans',
    pos=(0, 0.4), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
outline_2 = visual.TextStim(win=win, name='outline_2',
    text='最初に動作テストを各種行います。\n左右どちらかのLEDランプが光るので、どちら側が光ったか\n対応する反応ボックスの左右のボタンをできるだけ素早く\n正確に押して答えてください。\n\n各施行はNikolaが「行きます」と言って始まります\nこの時、Nikolaの目を見るようにしてください\nまた、Nikolaは時々右を向いたり、左を向いたりするのですが\nこれはどちらのランプが点灯するか手掛かりになっているわけでは\nないので気にしないでください',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);
outline_3 = visual.TextStim(win=win, name='outline_3',
    text='「Enterキー」を押して進みます',
    font='Open Sans',
    pos=(0, -0.4), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-2.0);
key_resp = keyboard.Keyboard()

# --- Initialize components for Routine "Exer_Eyecatch" ---
exer_msg = visual.TextStim(win=win, name='exer_msg',
    text='評定（パッド入力）の練習をします\nLEDが左右どちらかで点灯するので\nボックスの左右ボタンで選択してください\n\n「Enterキー」を押してを次に進みます',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
key_resp_3 = keyboard.Keyboard()

# --- Initialize components for Routine "Idleoff" ---

# --- Initialize components for Routine "Countdown_5" ---
count_sc = visual.TextStim(win=win, name='count_sc',
    text='',
    font='Open Sans',
    pos=(0, 0), height=0.1, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);

# --- Initialize components for Routine "Test" ---

# --- Initialize components for Routine "Check" ---
check_l_msg = visual.TextStim(win=win, name='check_l_msg',
    text='右か左、どちらからLEDが点灯したか\nボックスの左と右ボタンで答えてください。',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
led = cedrusBox_0
key_resp_4 = keyboard.Keyboard()

# --- Initialize components for Routine "Test_End" ---

# --- Initialize components for Routine "Exer_CounterReset" ---

# --- Initialize components for Routine "Test_Standby" ---
standby_msg = visual.TextStim(win=win, name='standby_msg',
    text='練習は終わりです\n\n本番の実験を開始します\n本番は合計96パターンの表出を評定いただきます\n連続で32回実施したあと、休憩を挟み\n残り32回ずつ休憩ををとりながら実施します\n間違ってもそのまま進行してしまうので\nできるだけ素早く、正確に評定をお願いします\n\n準備できたら「Enterキー」を押してください',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
key_resp_2 = keyboard.Keyboard()

# --- Initialize components for Routine "Idleoff" ---

# --- Initialize components for Routine "Countdown_5" ---
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
nRestTrial = 33 # 32回の倍数で休憩をはさむ
text_2 = visual.TextStim(win=win, name='text_2',
    text=None,
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);

# --- Initialize components for Routine "idleon" ---

# --- Initialize components for Routine "Routine" ---
countd_msg = visual.TextStim(win=win, name='countd_msg',
    text='',
    font='Open Sans',
    pos=(0, 0), height=0.1, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
recess = visual.TextStim(win=win, name='recess',
    text='「Enterキー」を押して再開します。\n(途中で「Enterキー」を押した場合\n タイマーを待たずに開始します。)',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);
key_resp_5 = keyboard.Keyboard()

# --- Initialize components for Routine "Idleoff" ---

# --- Initialize components for Routine "Test" ---

# --- Initialize components for Routine "Check" ---
check_l_msg = visual.TextStim(win=win, name='check_l_msg',
    text='右か左、どちらからLEDが点灯したか\nボックスの左と右ボタンで答えてください。',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
led = cedrusBox_0
key_resp_4 = keyboard.Keyboard()

# --- Initialize components for Routine "Test_End" ---

# --- Initialize components for Routine "idleon" ---

# --- Initialize components for Routine "Fin" ---
fin_msg = visual.TextStim(win=win, name='fin_msg',
    text='終了します',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
key_resp_6 = keyboard.Keyboard()

# Create some handy timers
globalClock = core.Clock()  # to track the time since experiment started
routineTimer = core.Clock()  # to track time remaining of each (possibly non-slip) routine 

# --- Prepare to start Routine "Init" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# keep track of which components have finished
InitComponents = [init_msg]
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
    
    # *init_msg* updates
    if init_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        init_msg.frameNStart = frameN  # exact frame index
        init_msg.tStart = t  # local t and not account for scr refresh
        init_msg.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(init_msg, 'tStartRefresh')  # time at next scr refresh
        init_msg.setAutoDraw(True)
    if init_msg.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > init_msg.tStartRefresh + 5.0-frameTolerance:
            # keep track of stop time/frame for later
            init_msg.tStop = t  # not accounting for scr refresh
            init_msg.frameNStop = frameN  # exact frame index
            init_msg.setAutoDraw(False)
    
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

# --- Prepare to start Routine "Outline" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
key_resp.keys = []
key_resp.rt = []
_key_resp_allKeys = []
# keep track of which components have finished
OutlineComponents = [outline_1, outline_2, outline_3, key_resp]
for thisComponent in OutlineComponents:
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

# --- Run Routine "Outline" ---
while continueRoutine:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *outline_1* updates
    if outline_1.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        outline_1.frameNStart = frameN  # exact frame index
        outline_1.tStart = t  # local t and not account for scr refresh
        outline_1.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(outline_1, 'tStartRefresh')  # time at next scr refresh
        outline_1.setAutoDraw(True)
    
    # *outline_2* updates
    if outline_2.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        outline_2.frameNStart = frameN  # exact frame index
        outline_2.tStart = t  # local t and not account for scr refresh
        outline_2.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(outline_2, 'tStartRefresh')  # time at next scr refresh
        outline_2.setAutoDraw(True)
    
    # *outline_3* updates
    if outline_3.status == NOT_STARTED and tThisFlip >= 3-frameTolerance:
        # keep track of start time/frame for later
        outline_3.frameNStart = frameN  # exact frame index
        outline_3.tStart = t  # local t and not account for scr refresh
        outline_3.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(outline_3, 'tStartRefresh')  # time at next scr refresh
        outline_3.setAutoDraw(True)
    
    # *key_resp* updates
    if key_resp.status == NOT_STARTED and t >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        key_resp.frameNStart = frameN  # exact frame index
        key_resp.tStart = t  # local t and not account for scr refresh
        key_resp.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(key_resp, 'tStartRefresh')  # time at next scr refresh
        key_resp.status = STARTED
        # keyboard checking is just starting
        key_resp.clock.reset()  # now t=0
        key_resp.clearEvents(eventType='keyboard')
    if key_resp.status == STARTED:
        theseKeys = key_resp.getKeys(keyList=['return'], waitRelease=False)
        _key_resp_allKeys.extend(theseKeys)
        if len(_key_resp_allKeys):
            key_resp.keys = _key_resp_allKeys[-1].name  # just the last key pressed
            key_resp.rt = _key_resp_allKeys[-1].rt
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
    for thisComponent in OutlineComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Outline" ---
for thisComponent in OutlineComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# the Routine "Outline" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "Exer_Eyecatch" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
key_resp_3.keys = []
key_resp_3.rt = []
_key_resp_3_allKeys = []
# keep track of which components have finished
Exer_EyecatchComponents = [exer_msg, key_resp_3]
for thisComponent in Exer_EyecatchComponents:
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

# --- Run Routine "Exer_Eyecatch" ---
while continueRoutine:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *exer_msg* updates
    if exer_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        exer_msg.frameNStart = frameN  # exact frame index
        exer_msg.tStart = t  # local t and not account for scr refresh
        exer_msg.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(exer_msg, 'tStartRefresh')  # time at next scr refresh
        exer_msg.setAutoDraw(True)
    
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
        theseKeys = key_resp_3.getKeys(keyList=['return'], waitRelease=False)
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
    for thisComponent in Exer_EyecatchComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Exer_Eyecatch" ---
for thisComponent in Exer_EyecatchComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# the Routine "Exer_Eyecatch" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "Idleoff" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# Run 'Begin Routine' code from idle_off
# stop idle behavior
dib_topic.publish(message_idleoff)
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

# --- Prepare to start Routine "Countdown_5" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# keep track of which components have finished
Countdown_5Components = [count_sc]
for thisComponent in Countdown_5Components:
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

# --- Run Routine "Countdown_5" ---
while continueRoutine and routineTimer.getTime() < 5.0:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *count_sc* updates
    if count_sc.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        count_sc.frameNStart = frameN  # exact frame index
        count_sc.tStart = t  # local t and not account for scr refresh
        count_sc.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(count_sc, 'tStartRefresh')  # time at next scr refresh
        count_sc.setAutoDraw(True)
    if count_sc.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > count_sc.tStartRefresh + 5-frameTolerance:
            # keep track of stop time/frame for later
            count_sc.tStop = t  # not accounting for scr refresh
            count_sc.frameNStop = frameN  # exact frame index
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
    for thisComponent in Countdown_5Components:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Countdown_5" ---
for thisComponent in Countdown_5Components:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# using non-slip timing so subtract the expected duration of this Routine (unless ended on request)
if routineForceEnded:
    routineTimer.reset()
else:
    routineTimer.addTime(-5.000000)

# set up handler to look after randomisation of conditions etc
exer_trials = data.TrialHandler(nReps=1.0, method='random', 
    extraInfo=expInfo, originPath=-1,
    trialList=data.importConditions('xlsx/ResumeGaze.xlsx'),
    seed=None, name='exer_trials')
thisExp.addLoop(exer_trials)  # add the loop to the experiment
thisExer_trial = exer_trials.trialList[0]  # so we can initialise stimuli with some values
# abbreviate parameter names if possible (e.g. rgb = thisExer_trial.rgb)
if thisExer_trial != None:
    for paramName in thisExer_trial:
        exec('{} = thisExer_trial[paramName]'.format(paramName))

for thisExer_trial in exer_trials:
    currentLoop = exer_trials
    # abbreviate parameter names if possible (e.g. rgb = thisExer_trial.rgb)
    if thisExer_trial != None:
        for paramName in thisExer_trial:
            exec('{} = thisExer_trial[paramName]'.format(paramName))
    
    # --- Prepare to start Routine "Test" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from p_init_test
    # head and eye moves to init position
    client1.send(head_cmd0_2.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    client2.send(facs_cmd0.encode("utf-8"))
    time.sleep(1)
    # Run 'Begin Routine' code from p_ready_test
    # face forward
    client1.send(head_cmd1_2.encode("utf-8"))
    time.sleep(1)
    # Run 'Begin Routine' code from t_go_test
    tts_topic.publish(message_ikimasu)
    time.sleep(1.5)
    # Run 'Begin Routine' code from p_start_test
    client2.send(facs_cmd1.encode("utf-8"))
    client1.send(eye_cmd1_2.encode("utf-8"))
    time.sleep(1)
    # Run 'Begin Routine' code from stim_test
    num_01 = modality
    num_02 = modality_lr
    num_03 = movepoint
    num_04 = movepoint_lr
    
    def stim_():
        if num_01 == 1:
            if num_02 == 1:
                ser.write(b"1")
            elif num_02 == 2:
                ser.write(b"2")
        elif num_01 == 2:
            if num_02 == 1:
                ser.write(b"3")     
            elif num_02 == 2:
                ser.write(b"4")    
    
    def nikola_head():
        if num_03 == 1:
            if num_04 == 1:
                gaze_cmd1 = headcmd[1]
            elif num_04 == 2:
                gaze_cmd1 = headcmd[2]
        elif num_03 == 2:
            if num_04 == 2:
                gaze_cmd1 = headcmd[0]
            elif num_04 == 2:
                gaze_cmd1 = headcmd[0]
        client1.send(gaze_cmd1.encode("utf-8"))
        
    def nikola_eye():
        if num_03 == 1:
            if num_04 == 1:
                gaze_cmd2 = eyecmd[1]
            elif num_04 == 2:
                gaze_cmd2 = eyecmd[2]   
        elif num_03 == 2:
            if num_04 == 1:
                gaze_cmd2 = eyecmd[1]
            elif num_04 == 2:
                gaze_cmd2 = eyecmd[2]
    
        client1.send(gaze_cmd2.encode("utf-8"))
        
    def wait_1():
        time.sleep(0.6)#2.2
    
    # Make thread
    thread1 = threading.Thread(target=stim_)
    thread2 = threading.Thread(target=nikola_head)
    thread3 = threading.Thread(target=nikola_eye)
    thread4 = threading.Thread(target=wait_1)
    
    # Start thread:1
    thread2.start()
    thread3.start()
    thread4.start()
    
    # wait 600ms
    time.sleep(0.6)
    
    # Start thread:2
    thread1.start()
    
    # Wait thread
    thread1.join()
    thread2.join()
    thread3.join()
    thread4.join()
    # keep track of which components have finished
    TestComponents = []
    for thisComponent in TestComponents:
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
    
    # --- Run Routine "Test" ---
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
        for thisComponent in TestComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Test" ---
    for thisComponent in TestComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Test" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Check" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    led.keys = []  # to store response values
    led.rt = []
    led.status = None
    key_resp_4.keys = []
    key_resp_4.rt = []
    _key_resp_4_allKeys = []
    # keep track of which components have finished
    CheckComponents = [check_l_msg, led, key_resp_4]
    for thisComponent in CheckComponents:
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
    
    # --- Run Routine "Check" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # *check_l_msg* updates
        if check_l_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            check_l_msg.frameNStart = frameN  # exact frame index
            check_l_msg.tStart = t  # local t and not account for scr refresh
            check_l_msg.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(check_l_msg, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'check_l_msg.started')
            check_l_msg.setAutoDraw(True)
        # *led* updates
        if led.status == NOT_STARTED and t >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            led.frameNStart = frameN  # exact frame index
            led.tStart = t  # local t and not account for scr refresh
            led.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(led, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.addData('led.started', t)
            led.status = STARTED
            led.clock.reset()  # now t=0
            # clear led responses (in a loop - the Cedrus own function doesn't work well)
            led.poll_for_response()
            while len(led.response_queue):
                led.clear_response_queue()
                led.poll_for_response() #often there are more resps waiting!
        if led.status == STARTED:
            theseKeys=[]
            theseRTs=[]
            # check for key presses
            led.poll_for_response()
            while len(led.response_queue):
                evt = led.get_next_response()
                if evt['key'] not in [1, 3]:
                    continue  # we don't care about this key
                if evt['pressed']:
                  theseKeys.append(evt['key'])
                  theseRTs.append(led.clock.getTime())
                led.poll_for_response()
            led.clear_response_queue()  # don't process again
            if len(theseKeys) > 0:  # at least one key was pressed
                if led.keys == []:  # then this is first keypress
                    led.keys = theseKeys[0]  # the first key pressed
                    led.rt = theseRTs[0]
                    # a response ends the routine
                    continueRoutine = False
        
        # *key_resp_4* updates
        waitOnFlip = False
        if key_resp_4.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            key_resp_4.frameNStart = frameN  # exact frame index
            key_resp_4.tStart = t  # local t and not account for scr refresh
            key_resp_4.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(key_resp_4, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'key_resp_4.started')
            key_resp_4.status = STARTED
            # keyboard checking is just starting
            waitOnFlip = True
            win.callOnFlip(key_resp_4.clock.reset)  # t=0 on next screen flip
            win.callOnFlip(key_resp_4.clearEvents, eventType='keyboard')  # clear events on next screen flip
        if key_resp_4.status == STARTED and not waitOnFlip:
            theseKeys = key_resp_4.getKeys(keyList=['left','right'], waitRelease=False)
            _key_resp_4_allKeys.extend(theseKeys)
            if len(_key_resp_4_allKeys):
                key_resp_4.keys = _key_resp_4_allKeys[-1].name  # just the last key pressed
                key_resp_4.rt = _key_resp_4_allKeys[-1].rt
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
        for thisComponent in CheckComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Check" ---
    for thisComponent in CheckComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # check responses
    if led.keys in ['', [], None]:  # No response was made
        led.keys = None
    exer_trials.addData('led.keys',led.keys)
    if led.keys != None:  # we had a response
        exer_trials.addData('led.rt', led.rt)
    # check responses
    if key_resp_4.keys in ['', [], None]:  # No response was made
        key_resp_4.keys = None
    exer_trials.addData('key_resp_4.keys',key_resp_4.keys)
    if key_resp_4.keys != None:  # we had a response
        exer_trials.addData('key_resp_4.rt', key_resp_4.rt)
    # the Routine "Check" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Test_End" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from _stim_test_2
    client1.send(head_cmd1_1.encode("utf-8"))
    client1.send(eye_cmd1_2.encode("utf-8"))
    
    time.sleep(1)
    # Run 'Begin Routine' code from _p_init_test_2
    client1.send(head_cmd0_2.encode("utf-8"))
    client2.send(facs_cmd0.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    time.sleep(1.5)
    # keep track of which components have finished
    Test_EndComponents = []
    for thisComponent in Test_EndComponents:
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
    
    # --- Run Routine "Test_End" ---
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
        for thisComponent in Test_EndComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Test_End" ---
    for thisComponent in Test_EndComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Test_End" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    thisExp.nextEntry()
    
# completed 1.0 repeats of 'exer_trials'


# --- Prepare to start Routine "Exer_CounterReset" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# Run 'Begin Routine' code from countreset
trialCounter = 1
# keep track of which components have finished
Exer_CounterResetComponents = []
for thisComponent in Exer_CounterResetComponents:
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

# --- Run Routine "Exer_CounterReset" ---
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
    for thisComponent in Exer_CounterResetComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Exer_CounterReset" ---
for thisComponent in Exer_CounterResetComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# the Routine "Exer_CounterReset" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "Test_Standby" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
key_resp_2.keys = []
key_resp_2.rt = []
_key_resp_2_allKeys = []
# keep track of which components have finished
Test_StandbyComponents = [standby_msg, key_resp_2]
for thisComponent in Test_StandbyComponents:
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

# --- Run Routine "Test_Standby" ---
while continueRoutine:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *standby_msg* updates
    if standby_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        standby_msg.frameNStart = frameN  # exact frame index
        standby_msg.tStart = t  # local t and not account for scr refresh
        standby_msg.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(standby_msg, 'tStartRefresh')  # time at next scr refresh
        standby_msg.setAutoDraw(True)
    
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
        theseKeys = key_resp_2.getKeys(keyList=['return'], waitRelease=False)
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
    for thisComponent in Test_StandbyComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Test_Standby" ---
for thisComponent in Test_StandbyComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# the Routine "Test_Standby" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "Idleoff" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# Run 'Begin Routine' code from idle_off
# stop idle behavior
dib_topic.publish(message_idleoff)
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

# --- Prepare to start Routine "Countdown_5" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# keep track of which components have finished
Countdown_5Components = [count_sc]
for thisComponent in Countdown_5Components:
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

# --- Run Routine "Countdown_5" ---
while continueRoutine and routineTimer.getTime() < 5.0:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *count_sc* updates
    if count_sc.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        count_sc.frameNStart = frameN  # exact frame index
        count_sc.tStart = t  # local t and not account for scr refresh
        count_sc.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(count_sc, 'tStartRefresh')  # time at next scr refresh
        count_sc.setAutoDraw(True)
    if count_sc.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > count_sc.tStartRefresh + 5-frameTolerance:
            # keep track of stop time/frame for later
            count_sc.tStop = t  # not accounting for scr refresh
            count_sc.frameNStop = frameN  # exact frame index
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
    for thisComponent in Countdown_5Components:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Countdown_5" ---
for thisComponent in Countdown_5Components:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# using non-slip timing so subtract the expected duration of this Routine (unless ended on request)
if routineForceEnded:
    routineTimer.reset()
else:
    routineTimer.addTime(-5.000000)

# set up handler to look after randomisation of conditions etc
test_trials = data.TrialHandler(nReps=12.0, method='random', 
    extraInfo=expInfo, originPath=-1,
    trialList=data.importConditions('xlsx/ResumeGaze.xlsx'),
    seed=None, name='test_trials')
thisExp.addLoop(test_trials)  # add the loop to the experiment
thisTest_trial = test_trials.trialList[0]  # so we can initialise stimuli with some values
# abbreviate parameter names if possible (e.g. rgb = thisTest_trial.rgb)
if thisTest_trial != None:
    for paramName in thisTest_trial:
        exec('{} = thisTest_trial[paramName]'.format(paramName))

for thisTest_trial in test_trials:
    currentLoop = test_trials
    # abbreviate parameter names if possible (e.g. rgb = thisTest_trial.rgb)
    if thisTest_trial != None:
        for paramName in thisTest_trial:
            exec('{} = thisTest_trial[paramName]'.format(paramName))
    
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
        
        # --- Prepare to start Routine "idleon" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # Run 'Begin Routine' code from start_idle
        dib_topic.publish(message_idleon)
        time.sleep(1)
        
        client2.send(facs_cmd1.encode("utf-8"))
        time.sleep(1)
        # keep track of which components have finished
        idleonComponents = []
        for thisComponent in idleonComponents:
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
        
        # --- Run Routine "idleon" ---
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
            for thisComponent in idleonComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
        
        # --- Ending Routine "idleon" ---
        for thisComponent in idleonComponents:
            if hasattr(thisComponent, "setAutoDraw"):
                thisComponent.setAutoDraw(False)
        # the Routine "idleon" was not non-slip safe, so reset the non-slip timer
        routineTimer.reset()
        
        # --- Prepare to start Routine "Routine" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        key_resp_5.keys = []
        key_resp_5.rt = []
        _key_resp_5_allKeys = []
        # keep track of which components have finished
        RoutineComponents = [countd_msg, recess, key_resp_5]
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
            
            # *countd_msg* updates
            if countd_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                countd_msg.frameNStart = frameN  # exact frame index
                countd_msg.tStart = t  # local t and not account for scr refresh
                countd_msg.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(countd_msg, 'tStartRefresh')  # time at next scr refresh
                countd_msg.setAutoDraw(True)
            if countd_msg.status == STARTED:
                # is it time to stop? (based on global clock, using actual start)
                if tThisFlipGlobal > countd_msg.tStartRefresh + 300-frameTolerance:
                    # keep track of stop time/frame for later
                    countd_msg.tStop = t  # not accounting for scr refresh
                    countd_msg.frameNStop = frameN  # exact frame index
                    countd_msg.setAutoDraw(False)
            if countd_msg.status == STARTED:  # only update if drawing
                countd_msg.setText(str(300-int(t)), log=False)
            
            # *recess* updates
            if recess.status == NOT_STARTED and tThisFlip >= 0-frameTolerance:
                # keep track of start time/frame for later
                recess.frameNStart = frameN  # exact frame index
                recess.tStart = t  # local t and not account for scr refresh
                recess.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(recess, 'tStartRefresh')  # time at next scr refresh
                # add timestamp to datafile
                thisExp.timestampOnFlip(win, 'recess.started')
                recess.setAutoDraw(True)
            
            # *key_resp_5* updates
            if key_resp_5.status == NOT_STARTED and t >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                key_resp_5.frameNStart = frameN  # exact frame index
                key_resp_5.tStart = t  # local t and not account for scr refresh
                key_resp_5.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(key_resp_5, 'tStartRefresh')  # time at next scr refresh
                key_resp_5.status = STARTED
                # keyboard checking is just starting
                key_resp_5.clock.reset()  # now t=0
                key_resp_5.clearEvents(eventType='keyboard')
            if key_resp_5.status == STARTED:
                theseKeys = key_resp_5.getKeys(keyList=['return'], waitRelease=False)
                _key_resp_5_allKeys.extend(theseKeys)
                if len(_key_resp_5_allKeys):
                    key_resp_5.keys = _key_resp_5_allKeys[-1].name  # just the last key pressed
                    key_resp_5.rt = _key_resp_5_allKeys[-1].rt
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
        # the Routine "Routine" was not non-slip safe, so reset the non-slip timer
        routineTimer.reset()
        
        # --- Prepare to start Routine "Idleoff" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # Run 'Begin Routine' code from idle_off
        # stop idle behavior
        dib_topic.publish(message_idleoff)
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
    
    
    # --- Prepare to start Routine "Test" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from p_init_test
    # head and eye moves to init position
    client1.send(head_cmd0_2.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    client2.send(facs_cmd0.encode("utf-8"))
    time.sleep(1)
    # Run 'Begin Routine' code from p_ready_test
    # face forward
    client1.send(head_cmd1_2.encode("utf-8"))
    time.sleep(1)
    # Run 'Begin Routine' code from t_go_test
    tts_topic.publish(message_ikimasu)
    time.sleep(1.5)
    # Run 'Begin Routine' code from p_start_test
    client2.send(facs_cmd1.encode("utf-8"))
    client1.send(eye_cmd1_2.encode("utf-8"))
    time.sleep(1)
    # Run 'Begin Routine' code from stim_test
    num_01 = modality
    num_02 = modality_lr
    num_03 = movepoint
    num_04 = movepoint_lr
    
    def stim_():
        if num_01 == 1:
            if num_02 == 1:
                ser.write(b"1")
            elif num_02 == 2:
                ser.write(b"2")
        elif num_01 == 2:
            if num_02 == 1:
                ser.write(b"3")     
            elif num_02 == 2:
                ser.write(b"4")    
    
    def nikola_head():
        if num_03 == 1:
            if num_04 == 1:
                gaze_cmd1 = headcmd[1]
            elif num_04 == 2:
                gaze_cmd1 = headcmd[2]
        elif num_03 == 2:
            if num_04 == 2:
                gaze_cmd1 = headcmd[0]
            elif num_04 == 2:
                gaze_cmd1 = headcmd[0]
        client1.send(gaze_cmd1.encode("utf-8"))
        
    def nikola_eye():
        if num_03 == 1:
            if num_04 == 1:
                gaze_cmd2 = eyecmd[1]
            elif num_04 == 2:
                gaze_cmd2 = eyecmd[2]   
        elif num_03 == 2:
            if num_04 == 1:
                gaze_cmd2 = eyecmd[1]
            elif num_04 == 2:
                gaze_cmd2 = eyecmd[2]
    
        client1.send(gaze_cmd2.encode("utf-8"))
        
    def wait_1():
        time.sleep(0.6)#2.2
    
    # Make thread
    thread1 = threading.Thread(target=stim_)
    thread2 = threading.Thread(target=nikola_head)
    thread3 = threading.Thread(target=nikola_eye)
    thread4 = threading.Thread(target=wait_1)
    
    # Start thread:1
    thread2.start()
    thread3.start()
    thread4.start()
    
    # wait 600ms
    time.sleep(0.6)
    
    # Start thread:2
    thread1.start()
    
    # Wait thread
    thread1.join()
    thread2.join()
    thread3.join()
    thread4.join()
    # keep track of which components have finished
    TestComponents = []
    for thisComponent in TestComponents:
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
    
    # --- Run Routine "Test" ---
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
        for thisComponent in TestComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Test" ---
    for thisComponent in TestComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Test" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Check" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    led.keys = []  # to store response values
    led.rt = []
    led.status = None
    key_resp_4.keys = []
    key_resp_4.rt = []
    _key_resp_4_allKeys = []
    # keep track of which components have finished
    CheckComponents = [check_l_msg, led, key_resp_4]
    for thisComponent in CheckComponents:
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
    
    # --- Run Routine "Check" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # *check_l_msg* updates
        if check_l_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            check_l_msg.frameNStart = frameN  # exact frame index
            check_l_msg.tStart = t  # local t and not account for scr refresh
            check_l_msg.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(check_l_msg, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'check_l_msg.started')
            check_l_msg.setAutoDraw(True)
        # *led* updates
        if led.status == NOT_STARTED and t >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            led.frameNStart = frameN  # exact frame index
            led.tStart = t  # local t and not account for scr refresh
            led.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(led, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.addData('led.started', t)
            led.status = STARTED
            led.clock.reset()  # now t=0
            # clear led responses (in a loop - the Cedrus own function doesn't work well)
            led.poll_for_response()
            while len(led.response_queue):
                led.clear_response_queue()
                led.poll_for_response() #often there are more resps waiting!
        if led.status == STARTED:
            theseKeys=[]
            theseRTs=[]
            # check for key presses
            led.poll_for_response()
            while len(led.response_queue):
                evt = led.get_next_response()
                if evt['key'] not in [1, 3]:
                    continue  # we don't care about this key
                if evt['pressed']:
                  theseKeys.append(evt['key'])
                  theseRTs.append(led.clock.getTime())
                led.poll_for_response()
            led.clear_response_queue()  # don't process again
            if len(theseKeys) > 0:  # at least one key was pressed
                if led.keys == []:  # then this is first keypress
                    led.keys = theseKeys[0]  # the first key pressed
                    led.rt = theseRTs[0]
                    # a response ends the routine
                    continueRoutine = False
        
        # *key_resp_4* updates
        waitOnFlip = False
        if key_resp_4.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            key_resp_4.frameNStart = frameN  # exact frame index
            key_resp_4.tStart = t  # local t and not account for scr refresh
            key_resp_4.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(key_resp_4, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'key_resp_4.started')
            key_resp_4.status = STARTED
            # keyboard checking is just starting
            waitOnFlip = True
            win.callOnFlip(key_resp_4.clock.reset)  # t=0 on next screen flip
            win.callOnFlip(key_resp_4.clearEvents, eventType='keyboard')  # clear events on next screen flip
        if key_resp_4.status == STARTED and not waitOnFlip:
            theseKeys = key_resp_4.getKeys(keyList=['left','right'], waitRelease=False)
            _key_resp_4_allKeys.extend(theseKeys)
            if len(_key_resp_4_allKeys):
                key_resp_4.keys = _key_resp_4_allKeys[-1].name  # just the last key pressed
                key_resp_4.rt = _key_resp_4_allKeys[-1].rt
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
        for thisComponent in CheckComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Check" ---
    for thisComponent in CheckComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # check responses
    if led.keys in ['', [], None]:  # No response was made
        led.keys = None
    test_trials.addData('led.keys',led.keys)
    if led.keys != None:  # we had a response
        test_trials.addData('led.rt', led.rt)
    # check responses
    if key_resp_4.keys in ['', [], None]:  # No response was made
        key_resp_4.keys = None
    test_trials.addData('key_resp_4.keys',key_resp_4.keys)
    if key_resp_4.keys != None:  # we had a response
        test_trials.addData('key_resp_4.rt', key_resp_4.rt)
    # the Routine "Check" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Test_End" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from _stim_test_2
    client1.send(head_cmd1_1.encode("utf-8"))
    client1.send(eye_cmd1_2.encode("utf-8"))
    
    time.sleep(1)
    # Run 'Begin Routine' code from _p_init_test_2
    client1.send(head_cmd0_2.encode("utf-8"))
    client2.send(facs_cmd0.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    time.sleep(1.5)
    # keep track of which components have finished
    Test_EndComponents = []
    for thisComponent in Test_EndComponents:
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
    
    # --- Run Routine "Test_End" ---
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
        for thisComponent in Test_EndComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Test_End" ---
    for thisComponent in Test_EndComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Test_End" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    thisExp.nextEntry()
    
# completed 12.0 repeats of 'test_trials'


# --- Prepare to start Routine "idleon" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# Run 'Begin Routine' code from start_idle
dib_topic.publish(message_idleon)
time.sleep(1)

client2.send(facs_cmd1.encode("utf-8"))
time.sleep(1)
# keep track of which components have finished
idleonComponents = []
for thisComponent in idleonComponents:
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

# --- Run Routine "idleon" ---
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
    for thisComponent in idleonComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "idleon" ---
for thisComponent in idleonComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# the Routine "idleon" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "Fin" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
key_resp_6.keys = []
key_resp_6.rt = []
_key_resp_6_allKeys = []
# keep track of which components have finished
FinComponents = [fin_msg, key_resp_6]
for thisComponent in FinComponents:
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

# --- Run Routine "Fin" ---
while continueRoutine:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *fin_msg* updates
    if fin_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        fin_msg.frameNStart = frameN  # exact frame index
        fin_msg.tStart = t  # local t and not account for scr refresh
        fin_msg.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(fin_msg, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.timestampOnFlip(win, 'fin_msg.started')
        fin_msg.setAutoDraw(True)
    if fin_msg.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > fin_msg.tStartRefresh + 10-frameTolerance:
            # keep track of stop time/frame for later
            fin_msg.tStop = t  # not accounting for scr refresh
            fin_msg.frameNStop = frameN  # exact frame index
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'fin_msg.stopped')
            fin_msg.setAutoDraw(False)
    
    # *key_resp_6* updates
    if key_resp_6.status == NOT_STARTED and t >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        key_resp_6.frameNStart = frameN  # exact frame index
        key_resp_6.tStart = t  # local t and not account for scr refresh
        key_resp_6.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(key_resp_6, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.addData('key_resp_6.started', t)
        key_resp_6.status = STARTED
        # keyboard checking is just starting
        key_resp_6.clock.reset()  # now t=0
        key_resp_6.clearEvents(eventType='keyboard')
    if key_resp_6.status == STARTED:
        theseKeys = key_resp_6.getKeys(keyList=['return'], waitRelease=False)
        _key_resp_6_allKeys.extend(theseKeys)
        if len(_key_resp_6_allKeys):
            key_resp_6.keys = _key_resp_6_allKeys[-1].name  # just the last key pressed
            key_resp_6.rt = _key_resp_6_allKeys[-1].rt
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
    for thisComponent in FinComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Fin" ---
for thisComponent in FinComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# the Routine "Fin" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()
# Run 'End Experiment' code from init_var
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
# Run 'End Experiment' code from init_arduino
ser.close()

# --- End experiment ---
# Flip one final time so any remaining win.callOnFlip() 
# and win.timeOnFlip() tasks get executed before quitting
win.flip()

# these shouldn't be strictly necessary (should auto-save)
thisExp.saveAsWideText(filename+'.csv', delim='auto')
thisExp.saveAsPickle(filename)
# make sure everything is closed down
if eyetracker:
    eyetracker.setConnectionState(False)
thisExp.abort()  # or data files will save again on exit
win.close()
core.quit()
