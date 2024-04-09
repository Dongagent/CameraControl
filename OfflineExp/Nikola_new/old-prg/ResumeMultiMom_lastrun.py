#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This experiment was created using PsychoPy3 Experiment Builder (v2022.2.5),
    on 10月 27, 2023, at 00:09
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

# Run 'Before Experiment' code from init_variable_2
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
peakframe_ = 100.0 # 8 or 10

# facs init value
iBrow   = 0.0 #innerBrow
oBrow   = 0.0 #outerbrow
uLid    = 0.0 #upperLid
lLid    = 0.0 #lowerLid
cheek   = 0.0 #cheek
nWrink  = 0.0 #noseWrinkler
uLip    = 0.0 #upperLip
lCorner = 0.0 #lipCorner
mCS     = 0.0 #mouthCornerStickout
oMouth  = 0.0 #mouthOpen
jDrop  = 0.0 #jawDrop

# positive value
iBrow_p    = 1.0 #innerBrow
oBrow_p    = 0.5 #outerbrow
uLid_p     = 0.3 #upperLid
lLid_p     = 1.0 #lowerLid
cheek_p    = 1.0 #cheek
nWrink_p   = 0.0 #noseWrinkler
uLip_p     = 0.0 #upperLip
lCorner_p  = 0.5 #lipCorner
mCS_p      = 0.5 #mouthCornerStickout
oMouth_p   = 0.0 #mouthOpen
jDrop_p    = 0.0 #jawDrop

# negative value
iBrow_n   = 1.0 #innerBrow
oBrow_n   = 0.0 #outerbrow
uLid_n    = -0.8 #upperLid
lLid_n    = 0.0 #lowerLid
cheek_n   = 0.0 #cheek
nWrink_n  = 0.0 #noseWrinkler
uLip_n    = 0.0 #upperLip
lCorner_n = -0.9 #lipCorner
mCS_n     = 0.0 #mouthCornerStickout
oMouth_n  = 0.0 #mouthOpen
jDrop_n   = 0.0 #jawDrop
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
            \"translateSpeed\":2.0}\n"

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
# Run 'Before Experiment' code from idle_off__test
# stop idle behavior
json_string = json.dumps(compoff_true)
message = roslibpy.Message({'data': json_string})
time.sleep(1)
dib_topic.publish(message)
time.sleep(1)

# eyemove 2 initpose
client2.send(facs_cmd0.encode("utf-8"))
time.sleep(1)
# Run 'Before Experiment' code from ichi
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
# Run 'Before Experiment' code from ni
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
# Run 'Before Experiment' code from san
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
    originPath='C:\\Users\\SATO\\OneDrive\\Coding\\Dr.Sato\\202310_PsychoPy\\ResumeMultiMom_lastrun.py',
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
# Run 'Begin Experiment' code from init_stim
iBrow_p_1   = iBrow_p   / peakframe_
oBrow_p_1   = oBrow_p   / peakframe_
uLid_p_1    = uLid_p    / peakframe_
lLid_p_1    = lLid_p    / peakframe_
cheek_p_1   = cheek_p   / peakframe_
nWrink_p_1  = nWrink_p  / peakframe_
uLip_p_1    = uLip_p    / peakframe_
lCorner_p_1 = lCorner_p / peakframe_
mCS_p_1     = mCS_p     / peakframe_
oMouth_p_1  = oMouth_p  / peakframe_
jDrop_p_1   = jDrop_p   / peakframe_

iBrow_n_1   = iBrow_n   / peakframe_
oBrow_n_1   = oBrow_n   / peakframe_
uLid_n_1    = uLid_n    / peakframe_
lLid_n_1    = lLid_n    / peakframe_
cheek_n_1   = cheek_n   / peakframe_
nWrink_n_1  = nWrink_n  / peakframe_
uLip_n_1    = uLip_n    / peakframe_
lCorner_n_1 = lCorner_n / peakframe_
mCS_n_1     = mCS_n     / peakframe_
oMouth_n_1  = oMouth_n  / peakframe_
jDrop_n_1   = jDrop_n   / peakframe_

# --- Initialize components for Routine "Explanation" ---
explanaton_1 = visual.TextStim(win=win, name='explanaton_1',
    text='実験を行います。\nキーボードの1~9を使って、アンドロイドの\n表情に対する評価値を以下の2つの項目別に入力してください。\n・感情価（説明）\n・ヒトらしさ（説明）',
    font='Open Sans',
    pos=(0, 0.25), height=0.04, wrapWidth=None, ori=0.0, 
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
    text='マウスの左クリックを押して実験を始めます。',
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
text_2 = visual.TextStim(win=win, name='text_2',
    text=None,
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);

# --- Initialize components for Routine "Test_init" ---

# --- Initialize components for Routine "Test_stim" ---

# --- Initialize components for Routine "Test_fin" ---

# --- Initialize components for Routine "Emo" ---
# Run 'Begin Experiment' code from emo_1
# FACS Piece Value
# NEGATIVE
iBrow_n_piece   = iBrow_n   / peakframe_ #innerBrow
oBrow_n_piece   = oBrow_n   / peakframe_ #outerbrow
uLid_n_piece    = uLid_n    / peakframe_ #upperLid
lLid_n_piece    = lLid_n    / peakframe_ #lowerLid
cheek_n_piece   = cheek_n   / peakframe_ #cheek
nWrink_n_piece  = 0 #nWrink_n  / peakframe_ #noseWrinkler
uLip_n_piece    = 0 #uLip_n    / peakframe_ #upperLip
lCorner_n_piece = lCorner_n / peakframe_ #lipCorner
mCS_n_piece     = mCS_n     / peakframe_ #mouthCornerStickout
oMouth_n_piece  = 0 #oMouth_n  / peakframe_ #mouthOpen
jDrop_n_piece   = 0 #jDrop_n   / peakframe_ #jawDrop

# POSITIVE
iBrow_p_piece   = iBrow_p   / peakframe_ #innerBrow
oBrow_p_piece   = oBrow_p   / peakframe_ #outerbrow
uLid_p_piece    = uLid_p    / peakframe_ #upperLid
lLid_p_piece    = lLid_p    / peakframe_ #lowerLid
cheek_p_piece   = cheek_p   / peakframe_ #cheek
nWrink_p_piece  = 0 #nWrink_p  / peakframe_ #noseWrinkler
uLip_p_piece    = 0 #uLip_p    / peakframe_ #upperLip
lCorner_p_piece = lCorner_p / peakframe_ #lipCorner
mCS_p_piece     = mCS_p     / peakframe_ #mouthCornerStickout
oMouth_p_piece  = 0 #oMouth_p  / peakframe_ #mouthOpen
jDrop_p_piece   = 0 #jDrop_p   / peakframe_ #jawDrop

# --- Initialize components for Routine "Test_fin" ---

# --- Initialize components for Routine "Check" ---
text_4 = visual.TextStim(win=win, name='text_4',
    text='Nikolaの表出した表情の中で最も近い番号をキーボードの１～３で選択してください。',
    font='Open Sans',
    pos=(0, 0.15), height=0.02, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
key_resp_2 = keyboard.Keyboard()

# --- Initialize components for Routine "Idleon" ---

# --- Initialize components for Routine "Finish" ---
shuryo = visual.TextStim(win=win, name='shuryo',
    text='終了します',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
routine_end_2 = event.Mouse(win=win)
x, y = [None, None]
routine_end_2.mouseClock = core.Clock()

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
ExplanationComponents = [explanaton_1, routine_end_1, disp_keyresp, keyresp, explanation_2]
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
    
    # *explanaton_1* updates
    if explanaton_1.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        explanaton_1.frameNStart = frameN  # exact frame index
        explanaton_1.tStart = t  # local t and not account for scr refresh
        explanaton_1.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(explanaton_1, 'tStartRefresh')  # time at next scr refresh
        explanaton_1.setAutoDraw(True)
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
            text_ = "左シフトか右シフトボタンを押してください。"
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
    if count_sc.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
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
trials_2 = data.TrialHandler(nReps=1.0, method='random', 
    extraInfo=expInfo, originPath=-1,
    trialList=data.importConditions('ResumeRepMom .xlsx'),
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
    text_2.setText(trialCounter)
    trialCounter = trialCounter + 1
    # keep track of which components have finished
    Test_EyecatchComponents = [text_2]
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
        
        # *text_2* updates
        if text_2.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            text_2.frameNStart = frameN  # exact frame index
            text_2.tStart = t  # local t and not account for scr refresh
            text_2.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(text_2, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'text_2.started')
            text_2.setAutoDraw(True)
        if text_2.status == STARTED:
            # is it time to stop? (based on global clock, using actual start)
            if tThisFlipGlobal > text_2.tStartRefresh + 1.0-frameTolerance:
                # keep track of stop time/frame for later
                text_2.tStop = t  # not accounting for scr refresh
                text_2.frameNStop = frameN  # exact frame index
                # add timestamp to datafile
                thisExp.timestampOnFlip(win, 'text_2.stopped')
                text_2.setAutoDraw(False)
        
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
    
    # --- Prepare to start Routine "Test_init" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from Init_pose_test
    client1.send(head_cmd0_1.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    # client2.send(facs_cmd0.encode("utf-8"))
    time.sleep(3)
    # Run 'Begin Routine' code from start_pose_test
    client1.send(head_cmd1_1.encode("utf-8"))
    time.sleep(2)
    # Run 'Begin Routine' code from ikimasu_test
    json_string = json.dumps(json_data_ikimasu)
    message = roslibpy.Message({'data': json_string})
    time.sleep(1)
    tts_topic.publish(message)
    time.sleep(2)
    # Run 'Begin Routine' code from ready_pose_test
    client2.send(facs_cmd1.encode("utf-8"))
    time.sleep(1)
    
    client1.send(eye_cmd1_2.encode("utf-8"))
    time.sleep(1)
    # keep track of which components have finished
    Test_initComponents = []
    for thisComponent in Test_initComponents:
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
    
    # --- Run Routine "Test_init" ---
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
        for thisComponent in Test_initComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Test_init" ---
    for thisComponent in Test_initComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Test_init" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Test_stim" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from stim_test
    num_01 = method
    num_02 = expression
    num_03 = level_
    
    print(num_01, num_02, num_03)
    
    counter_ = 0
    ratio_ = 0
    
    if num_03 == 1: # 60%
            ratio_ = 60
    elif num_03 == 2: # 90%
            ratio_ = 90
            
    # STATIC
    if num_01 == 2:
        # POSITIVE
        if num_02 == 2:
            # calc positive pose
            iBrow   = iBrow_p
            oBrow   = oBrow_p
            uLid    = uLid_p
            lLid    = lLid_p
            cheek   = cheek_p
            nWrink  = nWrink_p
            uLip    = uLip_p
            lCorner = lCorner_p
            mCS     = mCS_p
            oMouth  = oMouth_p
            jDrop   = jDrop_p
        # NEGATIVE
        elif num_02 == 1:
            # calc negative pose
            iBrow   = iBrow_n
            oBrow   = oBrow_n
            uLid    = uLid_n
            lLid    = lLid_n
            cheek   = cheek_n
            nWrink  = nWrink_n
            uLip    = uLip_n
            lCorner = lCorner_n
            mCS     = mCS_n
            oMouth  = oMouth_n
            jDrop   = jDrop_n
    # keep track of which components have finished
    Test_stimComponents = []
    for thisComponent in Test_stimComponents:
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
    
    # --- Run Routine "Test_stim" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        # Run 'Each Frame' code from stim_test
        print(counter_)
        # DYNAMIC
        if num_01 == 1:
            # POSITIVE
            if num_02 == 2:
                # calc positive pose
                iBrow   = iBrow_p_1   * counter_
                oBrow   = oBrow_p_1   * counter_
                uLid    = uLid_p_1    * counter_
                lLid    = lLid_p_1    * counter_
                cheek   = cheek_p_1   * counter_
                nWrink  = 0 #nWrink_p_1  * counter_
                uLip    = 0 #uLip_p_1    * counter_
                lCorner = lCorner_p_1 * counter_
                mCS     = mCS_p_1     * counter_
                oMouth  = 0 #oMouth_p_1  * counter_
                jDrop   = 0 #jDrop_p_1   * counter_
                
            # NEGATIVE
            elif num_02 == 1:
                # calc positive pose
                iBrow   = iBrow_n_1   * counter_
                oBrow   = 0 #oBrow_n_1   * counter_
                uLid    = uLid_n_1    * counter_
                lLid    = 0 #lLid_n_1    * counter_
                cheek   = 0 #cheek_n_1   * counter_
                nWrink  = 0 #nWrink_n_1  * counter_
                uLip    = 0 #uLip_n_1    * counter_
                lCorner = lCorner_n_1 * counter_
                mCS     = 0 #mCS_n_1     * counter_
                oMouth  = 0 #oMouth_n_1  * counter_
                jDrop   = 0 #jDrop_n_1   * counter_
        
        if ratio_ >= counter_:
            cmd = "facs InnerBrow "       + str(iBrow) + \
                  " Outerbrow "           + str(oBrow) + \
                  " UpperLid "            + str(uLid) + \
                  " LowerLid "            + str(lLid) + \
                  " Cheek "               + str(cheek) + \
                  " NoseWrinkler "        + str(0) + \
                  " UpperLip "            + str(0) + \
                  " LipCorner "           + str(lCorner) + \
                  " MouthCornerStickout " + str(mCS) + \
                  " MouthOpen "           + str(0) + \
                  " JawDrop "             + str(0) + "\n"
            client2.send(cmd.encode("utf-8"))
        else:
            cmd = "facs InnerBrow "       + str(0) + \
                  " Outerbrow "           + str(0) + \
                  " UpperLid "            + str(0) + \
                  " LowerLid "            + str(0) + \
                  " Cheek "               + str(0) + \
                  " NoseWrinkler "        + str(0) + \
                  " UpperLip "            + str(0) + \
                  " LipCorner "           + str(0) + \
                  " MouthCornerStickout " + str(0) + \
                  " MouthOpen "           + str(0) + \
                  " JawDrop "             + str(0) + "\n"
            client2.send(cmd.encode("utf-8"))
            time.sleep(0.5)
            #core.quit() # finish routine
            
        counter_ = counter_ + 1
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in Test_stimComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Test_stim" ---
    for thisComponent in Test_stimComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Test_stim" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Test_fin" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from stim_end_test
    client1.send(head_cmd1_1.encode("utf-8"))
    client1.send(eye_cmd1_2.encode("utf-8"))
    #client2.send(facs_cmd1.encode("utf-8"))
    
    time.sleep(2)
    # Run 'Begin Routine' code from Init_pose_test_2
    client1.send(head_cmd0_1.encode("utf-8"))
    client2.send(facs_cmd0.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    time.sleep(3)
    # keep track of which components have finished
    Test_finComponents = []
    for thisComponent in Test_finComponents:
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
    
    # --- Run Routine "Test_fin" ---
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
        for thisComponent in Test_finComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Test_fin" ---
    for thisComponent in Test_finComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Test_fin" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Emo" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from start_pose_ichi
    client2.send(facs_cmd1.encode("utf-8"))
    time.sleep(1)
    
    client1.send(head_cmd1_1.encode("utf-8"))
    time.sleep(2)
    # Run 'Begin Routine' code from ichi
    json_string = json.dumps(json_data_ichi)
    message = roslibpy.Message({'data': json_string})
    time.sleep(1)
    
    tts_topic.publish(message)
    time.sleep(1)
    # Run 'Begin Routine' code from emo_1
    num_02 = expression
    num_03 = level_
    
    magnification_1 = 5.5
    magnification_2 = 8.5
    
    # NEGATIVE
    if num_02 == 1:
        if num_03 == 1: # 60%
            iBrow   = iBrow_n_piece   * magnification_1
            oBrow   = oBrow_n_piece   * magnification_1
            uLid    = uLid_n_piece    * magnification_1
            lLid    = lLid_n_piece    * magnification_1
            cheek   = cheek_n_piece   * magnification_1
            nWrink  = 0 #nWrink_n_piece  * magnification_1
            uLip    = 0 #uLip_n_piece    * magnification_1
            lCorner = lCorner_n_piece * magnification_1
            mCS     = mCS_n_piece     * magnification_1
            oMouth  = 0 #oMouth_n_piece  * magnification_1
            jDrop   = 0 #jDrop_n_piece   * magnification_1
        elif num_03 == 2: # 90%
            iBrow   = iBrow_n_piece   * magnification_2
            oBrow   = oBrow_n_piece   * magnification_2
            uLid    = uLid_n_piece    * magnification_2
            lLid    = lLid_n_piece    * magnification_2
            cheek   = cheek_n_piece   * magnification_2
            nWrink  = 0 #nWrink_n_piece  * magnification_2
            uLip    = 0 #uLip_n_piece    * magnification_2
            lCorner = lCorner_n_piece * magnification_2
            mCS     = mCS_n_piece     * magnification_2
            oMouth  = 0 #oMouth_n_piece  * magnification_2
            jDrop   = 0 #jDrop_n_piece   * magnification_2
    # POSITIVE
    elif num_02 == 2:
        if num_03 == 1: # 60%
            iBrow   = iBrow_p_piece   * magnification_1
            oBrow   = oBrow_p_piece   * magnification_1
            uLid    = uLid_p_piece    * magnification_1
            lLid    = lLid_p_piece    * magnification_1
            cheek   = cheek_p_piece   * magnification_1
            nWrink  = 0 #nWrink_p_piece  * magnification_1
            uLip    = 0 #uLip_p_piece    * magnification_1
            lCorner = lCorner_p_piece * magnification_1
            mCS     = mCS_p_piece     * magnification_1
            oMouth  = 0 #oMouth_p_piece  * magnification_1
            jDrop   = 0 #jDrop_p_piece   * magnification_1
        elif num_03 == 2: # 90%
            iBrow   = iBrow_p_piece   * magnification_2
            oBrow   = oBrow_p_piece   * magnification_2
            uLid    = uLid_p_piece    * magnification_2
            lLid    = lLid_p_piece    * magnification_2
            cheek   = cheek_p_piece   * magnification_2
            nWrink  = 0 #nWrink_p_piece  * magnification_2
            uLip    = 0 #uLip_p_piece    * magnification_2
            lCorner = lCorner_p_piece * magnification_2
            mCS     = mCS_p_piece     * magnification_2
            oMouth  = 0 #oMouth_p_piece  * magnification_2
            jDrop   = 0 #jDrop_p_piece   * magnification_2
            
    cmd = "facs InnerBrow " + str(iBrow) + \
              " Outerbrow " + str(oBrow) + \
              " UpperLid " + str(uLid) + \
              " LowerLid " + str(lLid) + \
              " Cheek " + str(cheek) + \
              " NoseWrinkler " + str(nWrink) + \
              " UpperLip " + str(uLip) + \
              " LipCorner " + str(lCorner) + \
              " MouthCornerStickout " + str(mCS) + \
              " MouthOpen " + str(oMouth) + \
              " JawDrop " + str(jDrop) + "\n"
    time.sleep(1)
    
    client2.send(cmd.encode("utf-8"))
    time.sleep(0.5)
    
    #core.quit() # finish routine
    # Run 'Begin Routine' code from start_pose_ni
    client2.send(facs_cmd1.encode("utf-8"))
    time.sleep(1)
    
    client1.send(head_cmd1_1.encode("utf-8"))
    time.sleep(2)
    # Run 'Begin Routine' code from ni
    json_string = json.dumps(json_data_ni)
    message = roslibpy.Message({'data': json_string})
    time.sleep(1)
    tts_topic.publish(message)
    time.sleep(1)
    # Run 'Begin Routine' code from emo_2
    num_02 = expression
    num_03 = level_
    
    magnification_1 = 6
    magnification_2 = 9
    
    # NEGATIVE
    if num_02 == 1:
        if num_03 == 1: # 60%
            iBrow   = iBrow_n_piece   * magnification_1
            oBrow   = oBrow_n_piece   * magnification_1
            uLid    = uLid_n_piece    * magnification_1
            lLid    = lLid_n_piece    * magnification_1
            cheek   = cheek_n_piece   * magnification_1
            nWrink  = 0 #nWrink_n_piece  * magnification_1
            uLip    = 0 #uLip_n_piece    * magnification_1
            lCorner = lCorner_n_piece * magnification_1
            mCS     = mCS_n_piece     * magnification_1
            oMouth  = 0 #oMouth_n_piece  * magnification_1
            jDrop   = 0 #jDrop_n_piece   * magnification_1
        elif num_03 == 2: # 90%
            iBrow   = iBrow_n_piece   * magnification_2
            oBrow   = oBrow_n_piece   * magnification_2
            uLid    = uLid_n_piece    * magnification_2
            lLid    = lLid_n_piece    * magnification_2
            cheek   = cheek_n_piece   * magnification_2
            nWrink  = 0 #nWrink_n_piece  * magnification_2
            uLip    = 0 #uLip_n_piece    * magnification_2
            lCorner = lCorner_n_piece * magnification_2
            mCS     = mCS_n_piece     * magnification_2
            oMouth  = 0 #oMouth_n_piece  * magnification_2
            jDrop   = 0 #jDrop_n_piece   * magnification_2
    # POSITIVE
    elif num_02 == 2:
        if num_03 == 1: # 60%
            iBrow   = iBrow_p_piece   * magnification_1
            oBrow   = oBrow_p_piece   * magnification_1
            uLid    = uLid_p_piece    * magnification_1
            lLid    = lLid_p_piece    * magnification_1
            cheek   = cheek_p_piece   * magnification_1
            nWrink  = 0 #nWrink_p_piece  * magnification_1
            uLip    = 0 #uLip_p_piece    * magnification_1
            lCorner = lCorner_p_piece * magnification_1
            mCS     = mCS_p_piece     * magnification_1
            oMouth  = 0 #oMouth_p_piece  * magnification_1
            jDrop   = 0 #jDrop_p_piece   * magnification_1
        elif num_03 == 2: # 90%
            iBrow   = iBrow_p_piece   * magnification_2
            oBrow   = oBrow_p_piece   * magnification_2
            uLid    = uLid_p_piece    * magnification_2
            lLid    = lLid_p_piece    * magnification_2
            cheek   = cheek_p_piece   * magnification_2
            nWrink  = 0 #nWrink_p_piece  * magnification_2
            uLip    = 0 #uLip_p_piece    * magnification_2
            lCorner = lCorner_p_piece * magnification_2
            mCS     = mCS_p_piece     * magnification_2
            oMouth  = 0 #oMouth_p_piece  * magnification_2
            jDrop   = 0 #jDrop_p_piece   * magnification_2
            
    cmd = "facs InnerBrow " + str(iBrow) + \
              " Outerbrow " + str(oBrow) + \
              " UpperLid " + str(uLid) + \
              " LowerLid " + str(lLid) + \
              " Cheek " + str(cheek) + \
              " NoseWrinkler " + str(nWrink) + \
              " UpperLip " + str(uLip) + \
              " LipCorner " + str(lCorner) + \
              " MouthCornerStickout " + str(mCS) + \
              " MouthOpen " + str(oMouth) + \
              " JawDrop " + str(jDrop) + "\n"
    time.sleep(1)
    
    client2.send(cmd.encode("utf-8"))
    time.sleep(0.5)
    
    #core.quit() # finish routine
    # Run 'Begin Routine' code from start_pose_san
    client2.send(facs_cmd1.encode("utf-8"))
    time.sleep(1)
    
    client1.send(head_cmd1_1.encode("utf-8"))
    time.sleep(2)
    # Run 'Begin Routine' code from san
    json_string = json.dumps(json_data_san)
    message = roslibpy.Message({'data': json_string})
    time.sleep(1)
    tts_topic.publish(message)
    time.sleep(1)
    # Run 'Begin Routine' code from emo_3
    num_02 = expression
    num_03 = level_
    
    magnification_1 = 5.5
    magnification_2 = 8.5
    
    # NEGATIVE
    if num_02 == 1:
        if num_03 == 1: # 60%
            iBrow   = iBrow_n_piece   * magnification_1
            oBrow   = oBrow_n_piece   * magnification_1
            uLid    = uLid_n_piece    * magnification_1
            lLid    = lLid_n_piece    * magnification_1
            cheek   = cheek_n_piece   * magnification_1
            nWrink  = 0 #nWrink_n_piece  * magnification_1
            uLip    = 0 #uLip_n_piece    * magnification_1
            lCorner = lCorner_n_piece * magnification_1
            mCS     = mCS_n_piece     * magnification_1
            oMouth  = 0 #oMouth_n_piece  * magnification_1
            jDrop   = 0 #jDrop_n_piece   * magnification_1
        elif num_03 == 2: # 90%
            iBrow   = iBrow_n_piece   * magnification_2
            oBrow   = oBrow_n_piece   * magnification_2
            uLid    = uLid_n_piece    * magnification_2
            lLid    = lLid_n_piece    * magnification_2
            cheek   = cheek_n_piece   * magnification_2
            nWrink  = 0 #nWrink_n_piece  * magnification_2
            uLip    = 0 #uLip_n_piece    * magnification_2
            lCorner = lCorner_n_piece * magnification_2
            mCS     = mCS_n_piece     * magnification_2
            oMouth  = 0 #oMouth_n_piece  * magnification_2
            jDrop   = 0 #jDrop_n_piece   * magnification_2
    # POSITIVE
    elif num_02 == 2:
        if num_03 == 1: # 60%
            iBrow   = iBrow_p_piece   * magnification_1
            oBrow   = oBrow_p_piece   * magnification_1
            uLid    = uLid_p_piece    * magnification_1
            lLid    = lLid_p_piece    * magnification_1
            cheek   = cheek_p_piece   * magnification_1
            nWrink  = 0 #nWrink_p_piece  * magnification_1
            uLip    = 0 #uLip_p_piece    * magnification_1
            lCorner = lCorner_p_piece * magnification_1
            mCS     = mCS_p_piece     * magnification_1
            oMouth  = 0 #oMouth_p_piece  * magnification_1
            jDrop   = 0 #jDrop_p_piece   * magnification_1
        elif num_03 == 2: # 90%
            iBrow   = iBrow_p_piece   * magnification_2
            oBrow   = oBrow_p_piece   * magnification_2
            uLid    = uLid_p_piece    * magnification_2
            lLid    = lLid_p_piece    * magnification_2
            cheek   = cheek_p_piece   * magnification_2
            nWrink  = 0 #nWrink_p_piece  * magnification_2
            uLip    = 0 #uLip_p_piece    * magnification_2
            lCorner = lCorner_p_piece * magnification_2
            mCS     = mCS_p_piece     * magnification_2
            oMouth  = 0 #oMouth_p_piece  * magnification_2
            jDrop   = 0 #jDrop_p_piece   * magnification_2
            
    cmd = "facs InnerBrow " + str(iBrow) + \
              " Outerbrow " + str(oBrow) + \
              " UpperLid " + str(uLid) + \
              " LowerLid " + str(lLid) + \
              " Cheek " + str(cheek) + \
              " NoseWrinkler " + str(nWrink) + \
              " UpperLip " + str(uLip) + \
              " LipCorner " + str(lCorner) + \
              " MouthCornerStickout " + str(mCS) + \
              " MouthOpen " + str(oMouth) + \
              " JawDrop " + str(jDrop) + "\n"
    time.sleep(1)
    
    client2.send(cmd.encode("utf-8"))
    time.sleep(0.5)
    
    #core.quit() # finish routine
    # Run 'Begin Routine' code from start_pose_end
    client2.send(facs_cmd1.encode("utf-8"))
    time.sleep(1)
    
    client1.send(head_cmd1_1.encode("utf-8"))
    time.sleep(2)
    # keep track of which components have finished
    EmoComponents = []
    for thisComponent in EmoComponents:
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
    
    # --- Run Routine "Emo" ---
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
        for thisComponent in EmoComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Emo" ---
    for thisComponent in EmoComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Emo" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Test_fin" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from stim_end_test
    client1.send(head_cmd1_1.encode("utf-8"))
    client1.send(eye_cmd1_2.encode("utf-8"))
    #client2.send(facs_cmd1.encode("utf-8"))
    
    time.sleep(2)
    # Run 'Begin Routine' code from Init_pose_test_2
    client1.send(head_cmd0_1.encode("utf-8"))
    client2.send(facs_cmd0.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    time.sleep(3)
    # keep track of which components have finished
    Test_finComponents = []
    for thisComponent in Test_finComponents:
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
    
    # --- Run Routine "Test_fin" ---
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
        for thisComponent in Test_finComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Test_fin" ---
    for thisComponent in Test_finComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Test_fin" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Check" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    key_resp_2.keys = []
    key_resp_2.rt = []
    _key_resp_2_allKeys = []
    # keep track of which components have finished
    CheckComponents = [text_4, key_resp_2]
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
            theseKeys = key_resp_2.getKeys(keyList=['1','2','3'], waitRelease=False)
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
    if key_resp_2.keys in ['', [], None]:  # No response was made
        key_resp_2.keys = None
    trials_2.addData('key_resp_2.keys',key_resp_2.keys)
    if key_resp_2.keys != None:  # we had a response
        trials_2.addData('key_resp_2.rt', key_resp_2.rt)
    # the Routine "Check" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    thisExp.nextEntry()
    
# completed 1.0 repeats of 'trials_2'


# --- Prepare to start Routine "Idleon" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# Run 'Begin Routine' code from start_idle
json_string = json.dumps(compoff_false)

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

# --- Prepare to start Routine "Finish" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# setup some python lists for storing info about the routine_end_2
routine_end_2.x = []
routine_end_2.y = []
routine_end_2.leftButton = []
routine_end_2.midButton = []
routine_end_2.rightButton = []
routine_end_2.time = []
gotValidClick = False  # until a click is received
# keep track of which components have finished
FinishComponents = [shuryo, routine_end_2]
for thisComponent in FinishComponents:
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

# --- Run Routine "Finish" ---
while continueRoutine:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *shuryo* updates
    if shuryo.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        shuryo.frameNStart = frameN  # exact frame index
        shuryo.tStart = t  # local t and not account for scr refresh
        shuryo.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(shuryo, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.timestampOnFlip(win, 'shuryo.started')
        shuryo.setAutoDraw(True)
    if shuryo.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > shuryo.tStartRefresh + 10-frameTolerance:
            # keep track of stop time/frame for later
            shuryo.tStop = t  # not accounting for scr refresh
            shuryo.frameNStop = frameN  # exact frame index
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'shuryo.stopped')
            shuryo.setAutoDraw(False)
    # *routine_end_2* updates
    if routine_end_2.status == NOT_STARTED and t >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        routine_end_2.frameNStart = frameN  # exact frame index
        routine_end_2.tStart = t  # local t and not account for scr refresh
        routine_end_2.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(routine_end_2, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.addData('routine_end_2.started', t)
        routine_end_2.status = STARTED
        routine_end_2.mouseClock.reset()
        prevButtonState = routine_end_2.getPressed()  # if button is down already this ISN'T a new click
    if routine_end_2.status == STARTED:  # only update if started and not finished!
        buttons = routine_end_2.getPressed()
        if buttons != prevButtonState:  # button state changed?
            prevButtonState = buttons
            if sum(buttons) > 0:  # state changed to a new click
                x, y = routine_end_2.getPos()
                routine_end_2.x.append(x)
                routine_end_2.y.append(y)
                buttons = routine_end_2.getPressed()
                routine_end_2.leftButton.append(buttons[0])
                routine_end_2.midButton.append(buttons[1])
                routine_end_2.rightButton.append(buttons[2])
                routine_end_2.time.append(routine_end_2.mouseClock.getTime())
                
                continueRoutine = False  # abort routine on response
    
    # check for quit (typically the Esc key)
    if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
        core.quit()
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineForceEnded = True
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in FinishComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Finish" ---
for thisComponent in FinishComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# store data for thisExp (ExperimentHandler)
thisExp.addData('routine_end_2.x', routine_end_2.x)
thisExp.addData('routine_end_2.y', routine_end_2.y)
thisExp.addData('routine_end_2.leftButton', routine_end_2.leftButton)
thisExp.addData('routine_end_2.midButton', routine_end_2.midButton)
thisExp.addData('routine_end_2.rightButton', routine_end_2.rightButton)
thisExp.addData('routine_end_2.time', routine_end_2.time)
thisExp.nextEntry()
# the Routine "Finish" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()
# Run 'End Experiment' code from init_variable_2
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
