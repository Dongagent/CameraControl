#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This experiment was created using PsychoPy3 Experiment Builder (v2022.2.5),
    on 11月 06, 2023, at 14:23
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

eye_cmd1_1 = "EyeController={\"id\":\"EyeController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \"translateSpeed\":1.0}\n"

eye_cmd1_2 = "EyeController={\"id\":\"EyeController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \"translateSpeed\":2.0}\n"
            
eye_cmd1_4 = "EyeController={\"id\":\"EyeController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
            \"targetPoint\":{\"x\":0.0,\"y\":1.2,\"z\":1.5}, \"translateSpeed\":4.0}\n"

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
filename = _thisDir + os.sep + u'data/%s_%s_%s_%s' % ('RepMom', expInfo['subj'], expInfo['participant'], expInfo['date'])

# An ExperimentHandler isn't essential but helps with data saving
thisExp = data.ExperimentHandler(name=expName, version='',
    extraInfo=expInfo, runtimeInfo=None,
    originPath='C:\\Users\\SATO\\OneDrive\\Coding\\Dr.Sato\\exp_202311\\3_ResumeRepMom_lastrun.py',
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
# Run 'Begin Experiment' code from init_stim
cmd_n_55 = "facs InnerBrow "   + str(iBrow_n * 0.55) + \
  " Outerbrow "           + str(0) + \
  " UpperLid "            + str(uLid_n * 0.55) + \
  " LowerLid "            + str(0) + \
  " Cheek "               + str(0) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_n * 0.55) + \
  " MouthCornerStickout " + str(0) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"

cmd_n_60 = "facs InnerBrow "   + str(iBrow_n * 0.6) + \
  " Outerbrow "           + str(0) + \
  " UpperLid "            + str(uLid_n * 0.6) + \
  " LowerLid "            + str(0) + \
  " Cheek "               + str(0) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_n * 0.6) + \
  " MouthCornerStickout " + str(0) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"

cmd_n_65 = "facs InnerBrow "   + str(iBrow_n * 0.65) + \
  " Outerbrow "           + str(0) + \
  " UpperLid "            + str(uLid_n * 0.65) + \
  " LowerLid "            + str(0) + \
  " Cheek "               + str(0) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_n * 0.65) + \
  " MouthCornerStickout " + str(0) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"

cmd_n_85 = "facs InnerBrow "   + str(iBrow_n * 0.85) + \
  " Outerbrow "           + str(0) + \
  " UpperLid "            + str(uLid_n * 0.85) + \
  " LowerLid "            + str(0) + \
  " Cheek "               + str(0) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_n * 0.85) + \
  " MouthCornerStickout " + str(0) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"

cmd_n_90 = "facs InnerBrow "   + str(iBrow_n * 0.9) + \
  " Outerbrow "           + str(0) + \
  " UpperLid "            + str(uLid_n * 0.9) + \
  " LowerLid "            + str(0) + \
  " Cheek "               + str(0) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_n * 0.9) + \
  " MouthCornerStickout " + str(0) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"
  
cmd_n_95 = "facs InnerBrow "   + str(iBrow_n * 0.95) + \
  " Outerbrow "           + str(0) + \
  " UpperLid "            + str(uLid_n * 0.95) + \
  " LowerLid "            + str(0) + \
  " Cheek "               + str(0) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_n * 0.95) + \
  " MouthCornerStickout " + str(0) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"
  
cmd_p_55 = "facs InnerBrow "   + str(iBrow * 0.55) + \
  " Outerbrow "           + str(oBrow_p * 0.55) + \
  " UpperLid "            + str(uLid_p * 0.55) + \
  " LowerLid "            + str(lLid_p * 0.55) + \
  " Cheek "               + str(cheek_p * 0.55) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_p * 0.55) + \
  " MouthCornerStickout " + str(mCS_p * 0.55) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"
  
cmd_p_60 = "facs InnerBrow "   + str(iBrow * 0.60) + \
  " Outerbrow "           + str(oBrow_p * 0.60) + \
  " UpperLid "            + str(uLid_p * 0.60) + \
  " LowerLid "            + str(lLid_p * 0.60) + \
  " Cheek "               + str(cheek_p * 0.60) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_p * 0.60) + \
  " MouthCornerStickout " + str(mCS_p * 0.60) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"
  
cmd_p_65 = "facs InnerBrow "   + str(iBrow * 0.65) + \
  " Outerbrow "           + str(oBrow_p * 0.65) + \
  " UpperLid "            + str(uLid_p * 0.65) + \
  " LowerLid "            + str(lLid_p * 0.65) + \
  " Cheek "               + str(cheek_p * 0.65) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_p * 0.65) + \
  " MouthCornerStickout " + str(mCS_p * 0.65) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"
  
cmd_p_85 = "facs InnerBrow "   + str(iBrow * 0.85) + \
  " Outerbrow "           + str(oBrow_p * 0.85) + \
  " UpperLid "            + str(uLid_p * 0.85) + \
  " LowerLid "            + str(lLid_p * 0.85) + \
  " Cheek "               + str(cheek_p * 0.85) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_p * 0.85) + \
  " MouthCornerStickout " + str(mCS_p * 0.85) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"
  
cmd_p_90 = "facs InnerBrow "   + str(iBrow * 0.90) + \
  " Outerbrow "           + str(oBrow_p * 0.90) + \
  " UpperLid "            + str(uLid_p * 0.90) + \
  " LowerLid "            + str(lLid_p * 0.90) + \
  " Cheek "               + str(cheek_p * 0.90) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_p * 0.90) + \
  " MouthCornerStickout " + str(mCS_p * 0.90) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"
  
cmd_p_95 = "facs InnerBrow "   + str(iBrow * 0.95) + \
  " Outerbrow "           + str(oBrow_p * 0.95) + \
  " UpperLid "            + str(uLid_p * 0.95) + \
  " LowerLid "            + str(lLid_p * 0.95) + \
  " Cheek "               + str(cheek_p * 0.95) + \
  " NoseWrinkler "        + str(0) + \
  " UpperLip "            + str(0) + \
  " LipCorner "           + str(lCorner_p * 0.95) + \
  " MouthCornerStickout " + str(mCS_p * 0.95) + \
  " MouthOpen "           + str(0) + \
  " JawDrop "             + str(0) + "\n"

# --- Initialize components for Routine "Outline" ---
outline_1 = visual.TextStim(win=win, name='outline_1',
    text='- 実験の説明 -\n',
    font='Open Sans',
    pos=(0, 0.4), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
outline_2 = visual.TextStim(win=win, name='outline_2',
    text='各施行の開始時にNikolaが「行きます」と言います\nその後、Nikolaが動的あるいは静的に表情を表出します\n続けて、Nikolaが「いち」「に」「さん」という掛け声と共に\n３つの表情を表出するので、その中で表情の最後の状態\nがどれだったか、キーボードの左側の１~３で選択してください\n\nあまり考えこまずに直感的に入力してください。',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);
outline_3 = visual.TextStim(win=win, name='outline_3',
    text='「Enterキー」を押して実験を開始します',
    font='Open Sans',
    pos=(0, -0.35), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-2.0);
key_resp = keyboard.Keyboard()

# --- Initialize components for Routine "Exer_Eyecatch" ---
exer_msg_1 = visual.TextStim(win=win, name='exer_msg_1',
    text='評定（キーボード入力）の練習を3回します\nNikolaが表現した後に、「いち」「に」「さん」の掛け声と\n共に3つの表情を表出するので、その中で\n最も近い表情をキーボードの１～３で選択してください',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
exer_msg_2 = visual.TextStim(win=win, name='exer_msg_2',
    text='「Enterキー」を押してを次に進みます',
    font='Open Sans',
    pos=(0, -0.4), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);
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

# --- Initialize components for Routine "Counter" ---
# Run 'Begin Experiment' code from code
trialCounter = 1
text_2 = visual.TextStim(win=win, name='text_2',
    text=None,
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);

# --- Initialize components for Routine "Test" ---

# --- Initialize components for Routine "count" ---

# --- Initialize components for Routine "Emo" ---

# --- Initialize components for Routine "Emo_end" ---

# --- Initialize components for Routine "Check_beep" ---

# --- Initialize components for Routine "Check" ---
check_f_msg = visual.TextStim(win=win, name='check_f_msg',
    text='Nikolaの表出した表情の中で最も近い番号を\nキーボードの１～３で選択してください。',
    font='Open Sans',
    pos=(0, 0.15), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
face = keyboard.Keyboard()

# --- Initialize components for Routine "Exer_CounterReset" ---

# --- Initialize components for Routine "Idleon" ---

# --- Initialize components for Routine "Test_Standby" ---
standby_msg = visual.TextStim(win=win, name='standby_msg',
    text='練習は終わりです\n\n本番の実験を開始します\n本番は合計48パターンの表現を評定いただきます\n休憩はなく連続で実施します\nあまり考えこまずに直感的に入力してください\n\n準備できたら「Enterキー」を押してください',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
key_resp_4 = keyboard.Keyboard()

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
# Run 'Begin Experiment' code from code
trialCounter = 1
text_2 = visual.TextStim(win=win, name='text_2',
    text=None,
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);

# --- Initialize components for Routine "Test" ---

# --- Initialize components for Routine "count" ---

# --- Initialize components for Routine "Emo" ---

# --- Initialize components for Routine "Emo_end" ---

# --- Initialize components for Routine "Check_beep" ---

# --- Initialize components for Routine "Check" ---
check_f_msg = visual.TextStim(win=win, name='check_f_msg',
    text='Nikolaの表出した表情の中で最も近い番号を\nキーボードの１～３で選択してください。',
    font='Open Sans',
    pos=(0, 0.15), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
face = keyboard.Keyboard()

# --- Initialize components for Routine "Idleon" ---

# --- Initialize components for Routine "Fin" ---
fin_msg = visual.TextStim(win=win, name='fin_msg',
    text='実験を終了します',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
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
Exer_EyecatchComponents = [exer_msg_1, exer_msg_2, key_resp_3]
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
    
    # *exer_msg_1* updates
    if exer_msg_1.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        exer_msg_1.frameNStart = frameN  # exact frame index
        exer_msg_1.tStart = t  # local t and not account for scr refresh
        exer_msg_1.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(exer_msg_1, 'tStartRefresh')  # time at next scr refresh
        exer_msg_1.setAutoDraw(True)
    
    # *exer_msg_2* updates
    if exer_msg_2.status == NOT_STARTED and tThisFlip >= 3-frameTolerance:
        # keep track of start time/frame for later
        exer_msg_2.frameNStart = frameN  # exact frame index
        exer_msg_2.tStart = t  # local t and not account for scr refresh
        exer_msg_2.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(exer_msg_2, 'tStartRefresh')  # time at next scr refresh
        exer_msg_2.setAutoDraw(True)
    
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
    trialList=data.importConditions('xlsx/ResumeRepMom_exer.xlsx'),
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
    
    # --- Prepare to start Routine "Counter" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from code
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
            if tThisFlipGlobal > text_2.tStartRefresh + 1.0-frameTolerance:
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
    client1.send(head_cmd_rm.encode("utf-8"))
    time.sleep(2)
    # Run 'Begin Routine' code from stim_test
    num_01 = method
    num_02 = expression
    num_03 = level_
    
    if num_01 == 1:# DYNAMIC
        client2.send(facs_cmd1.encode("utf-8"))
        client1.send(head_cmd1_2.encode("utf-8"))
        client1.send(eye_cmd1_2.encode("utf-8"))
        time.sleep(2)
        
        if num_02 == 1:# NEGATIVE
            if num_03 == 1:# 60%
                client2.send(cmd_n_60.encode("utf-8"))
                time.sleep(1)
                
            elif num_03 == 2:#90%
                client2.send(cmd_n_90.encode("utf-8"))
                time.sleep(1)
                
        elif num_02 == 2:# POSITIVE
            if num_03 == 1:# 60%
                client2.send(cmd_p_60.encode("utf-8"))
                time.sleep(1)
                
            elif num_03 == 2:# 90%
                client2.send(cmd_p_90.encode("utf-8"))
                time.sleep(1)
                
    elif num_01 == 2:# STATIC
        
        if num_02 == 1:# NEGATIVE
            if num_03 == 1:# 60%
                client2.send(cmd_n_60.encode("utf-8"))
                time.sleep(1.5)
                
            elif num_03 == 2:# 90%
                client2.send(cmd_n_90.encode("utf-8"))
                time.sleep(1.5)
                
        elif num_02 == 2:# POSITIVE
            if num_03 == 1:# 60%
                client2.send(cmd_p_60.encode("utf-8"))
                time.sleep(1.5)
                
            elif num_03 == 2:# 90%
                client2.send(cmd_p_90.encode("utf-8"))
                time.sleep(1.5)
    
        client1.send(head_cmd1_4.encode("utf-8"))
        client1.send(eye_cmd1_4.encode("utf-8"))
        time.sleep(2)
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
    
    # --- Prepare to start Routine "count" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from code_2
    counter_ = 1
    # keep track of which components have finished
    countComponents = []
    for thisComponent in countComponents:
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
    
    # --- Run Routine "count" ---
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
        for thisComponent in countComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "count" ---
    for thisComponent in countComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "count" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # set up handler to look after randomisation of conditions etc
    exer_emo = data.TrialHandler(nReps=1.0, method='random', 
        extraInfo=expInfo, originPath=-1,
        trialList=data.importConditions('xlsx/Emo.xlsx'),
        seed=None, name='exer_emo')
    thisExp.addLoop(exer_emo)  # add the loop to the experiment
    thisExer_emo = exer_emo.trialList[0]  # so we can initialise stimuli with some values
    # abbreviate parameter names if possible (e.g. rgb = thisExer_emo.rgb)
    if thisExer_emo != None:
        for paramName in thisExer_emo:
            exec('{} = thisExer_emo[paramName]'.format(paramName))
    
    for thisExer_emo in exer_emo:
        currentLoop = exer_emo
        # abbreviate parameter names if possible (e.g. rgb = thisExer_emo.rgb)
        if thisExer_emo != None:
            for paramName in thisExer_emo:
                exec('{} = thisExer_emo[paramName]'.format(paramName))
        
        # --- Prepare to start Routine "Emo" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # Run 'Begin Routine' code from p_ready_emo
        client2.send(facs_cmd1.encode("utf-8"))
        client1.send(head_cmd1_1.encode("utf-8"))
        time.sleep(1)
        # Run 'Begin Routine' code from t_kazu_emo
        if counter_ == 1:
            json_string = json.dumps(json_data_ichi)
        elif counter_ == 2:
            json_string = json.dumps(json_data_ni)
        elif counter_ == 3:
            json_string = json.dumps(json_data_san)
            
        message = roslibpy.Message({'data': json_string})
        time.sleep(1)
        
        tts_topic.publish(message)
        time.sleep(1.5)
        # Run 'Begin Routine' code from p_down_emo
        client1.send(head_cmd_rm.encode("utf-8"))
        client2.send(facs_cmd0.encode("utf-8"))
        time.sleep(1.5)
        # Run 'Begin Routine' code from stim_emo
        num_02 = expression
        num_03 = level_
        num_04 = emo
        
        # NEGATIVE
        if num_02 == 1:
            if num_03 == 1: # 60%
                if num_04 == 1:
                    client2.send(cmd_n_55.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 2:
                    client2.send(cmd_n_60.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 3:
                    client2.send(cmd_n_65.encode("utf-8"))
                    time.sleep(1.5)
                
            elif num_03 == 2: # 90%
                if num_04 == 1:
                    client2.send(cmd_n_85.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 2:
                    client2.send(cmd_n_90.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 3:
                    client2.send(cmd_n_95.encode("utf-8"))
                    time.sleep(1.5)
                
        # POSITIVE
        elif num_02 == 2:
            if num_03 == 1: # 60%
                if num_04 == 1:
                    client2.send(cmd_p_55.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 2:
                    client2.send(cmd_p_60.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 3:
                    client2.send(cmd_p_65.encode("utf-8"))
                    time.sleep(1.5)
                
            elif num_03 == 2: # 90%
                if num_04 == 1:
                    client2.send(cmd_p_85.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 2:
                    client2.send(cmd_p_90.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 3:
                    client2.send(cmd_p_95.encode("utf-8"))
                    time.sleep(1.5)
        
        client1.send(head_cmd1_4.encode("utf-8"))
        client1.send(eye_cmd1_4.encode("utf-8"))
        time.sleep(2)
        
        counter_ = counter_ + 1
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
        thisExp.nextEntry()
        
    # completed 1.0 repeats of 'exer_emo'
    
    
    # --- Prepare to start Routine "Emo_end" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from p_end_emo
    client2.send(facs_cmd0.encode("utf-8"))
    time.sleep(1)
    
    client1.send(head_cmd0_2.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    time.sleep(2)
    # keep track of which components have finished
    Emo_endComponents = []
    for thisComponent in Emo_endComponents:
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
    
    # --- Run Routine "Emo_end" ---
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
        for thisComponent in Emo_endComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Emo_end" ---
    for thisComponent in Emo_endComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Emo_end" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Check_beep" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from check_beep
    ser.write(b"5")
    time.sleep(0.5)
    # keep track of which components have finished
    Check_beepComponents = []
    for thisComponent in Check_beepComponents:
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
    
    # --- Run Routine "Check_beep" ---
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
        for thisComponent in Check_beepComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Check_beep" ---
    for thisComponent in Check_beepComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Check_beep" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Check" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    face.keys = []
    face.rt = []
    _face_allKeys = []
    # keep track of which components have finished
    CheckComponents = [check_f_msg, face]
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
        
        # *check_f_msg* updates
        if check_f_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            check_f_msg.frameNStart = frameN  # exact frame index
            check_f_msg.tStart = t  # local t and not account for scr refresh
            check_f_msg.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(check_f_msg, 'tStartRefresh')  # time at next scr refresh
            check_f_msg.setAutoDraw(True)
        
        # *face* updates
        if face.status == NOT_STARTED and t >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            face.frameNStart = frameN  # exact frame index
            face.tStart = t  # local t and not account for scr refresh
            face.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(face, 'tStartRefresh')  # time at next scr refresh
            face.status = STARTED
            # keyboard checking is just starting
            face.clock.reset()  # now t=0
            face.clearEvents(eventType='keyboard')
        if face.status == STARTED:
            theseKeys = face.getKeys(keyList=['1','2','3'], waitRelease=False)
            _face_allKeys.extend(theseKeys)
            if len(_face_allKeys):
                face.keys = _face_allKeys[-1].name  # just the last key pressed
                face.rt = _face_allKeys[-1].rt
                # was this correct?
                if (face.keys == str('')) or (face.keys == ''):
                    face.corr = 1
                else:
                    face.corr = 0
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
    if face.keys in ['', [], None]:  # No response was made
        face.keys = None
        # was no response the correct answer?!
        if str('').lower() == 'none':
           face.corr = 1;  # correct non-response
        else:
           face.corr = 0;  # failed to respond (incorrectly)
    # store data for exer_trials (TrialHandler)
    exer_trials.addData('face.keys',face.keys)
    exer_trials.addData('face.corr', face.corr)
    if face.keys != None:  # we had a response
        exer_trials.addData('face.rt', face.rt)
    # the Routine "Check" was not non-slip safe, so reset the non-slip timer
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

# --- Prepare to start Routine "Idleon" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# Run 'Begin Routine' code from start_idle
dib_topic.publish(message_idleon)
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

# --- Prepare to start Routine "Test_Standby" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
key_resp_4.keys = []
key_resp_4.rt = []
_key_resp_4_allKeys = []
# keep track of which components have finished
Test_StandbyComponents = [standby_msg, key_resp_4]
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
    
    # *key_resp_4* updates
    if key_resp_4.status == NOT_STARTED and t >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        key_resp_4.frameNStart = frameN  # exact frame index
        key_resp_4.tStart = t  # local t and not account for scr refresh
        key_resp_4.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(key_resp_4, 'tStartRefresh')  # time at next scr refresh
        key_resp_4.status = STARTED
        # keyboard checking is just starting
        key_resp_4.clock.reset()  # now t=0
        key_resp_4.clearEvents(eventType='keyboard')
    if key_resp_4.status == STARTED:
        theseKeys = key_resp_4.getKeys(keyList=['return'], waitRelease=False)
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
test_trials = data.TrialHandler(nReps=3.0, method='random', 
    extraInfo=expInfo, originPath=-1,
    trialList=data.importConditions('xlsx/ResumeRepMom .xlsx'),
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
    # Run 'Begin Routine' code from code
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
            if tThisFlipGlobal > text_2.tStartRefresh + 1.0-frameTolerance:
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
    client1.send(head_cmd_rm.encode("utf-8"))
    time.sleep(2)
    # Run 'Begin Routine' code from stim_test
    num_01 = method
    num_02 = expression
    num_03 = level_
    
    if num_01 == 1:# DYNAMIC
        client2.send(facs_cmd1.encode("utf-8"))
        client1.send(head_cmd1_2.encode("utf-8"))
        client1.send(eye_cmd1_2.encode("utf-8"))
        time.sleep(2)
        
        if num_02 == 1:# NEGATIVE
            if num_03 == 1:# 60%
                client2.send(cmd_n_60.encode("utf-8"))
                time.sleep(1)
                
            elif num_03 == 2:#90%
                client2.send(cmd_n_90.encode("utf-8"))
                time.sleep(1)
                
        elif num_02 == 2:# POSITIVE
            if num_03 == 1:# 60%
                client2.send(cmd_p_60.encode("utf-8"))
                time.sleep(1)
                
            elif num_03 == 2:# 90%
                client2.send(cmd_p_90.encode("utf-8"))
                time.sleep(1)
                
    elif num_01 == 2:# STATIC
        
        if num_02 == 1:# NEGATIVE
            if num_03 == 1:# 60%
                client2.send(cmd_n_60.encode("utf-8"))
                time.sleep(1.5)
                
            elif num_03 == 2:# 90%
                client2.send(cmd_n_90.encode("utf-8"))
                time.sleep(1.5)
                
        elif num_02 == 2:# POSITIVE
            if num_03 == 1:# 60%
                client2.send(cmd_p_60.encode("utf-8"))
                time.sleep(1.5)
                
            elif num_03 == 2:# 90%
                client2.send(cmd_p_90.encode("utf-8"))
                time.sleep(1.5)
    
        client1.send(head_cmd1_4.encode("utf-8"))
        client1.send(eye_cmd1_4.encode("utf-8"))
        time.sleep(2)
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
    
    # --- Prepare to start Routine "count" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from code_2
    counter_ = 1
    # keep track of which components have finished
    countComponents = []
    for thisComponent in countComponents:
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
    
    # --- Run Routine "count" ---
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
        for thisComponent in countComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "count" ---
    for thisComponent in countComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "count" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # set up handler to look after randomisation of conditions etc
    test_emo = data.TrialHandler(nReps=1.0, method='random', 
        extraInfo=expInfo, originPath=-1,
        trialList=data.importConditions('xlsx/Emo.xlsx'),
        seed=None, name='test_emo')
    thisExp.addLoop(test_emo)  # add the loop to the experiment
    thisTest_emo = test_emo.trialList[0]  # so we can initialise stimuli with some values
    # abbreviate parameter names if possible (e.g. rgb = thisTest_emo.rgb)
    if thisTest_emo != None:
        for paramName in thisTest_emo:
            exec('{} = thisTest_emo[paramName]'.format(paramName))
    
    for thisTest_emo in test_emo:
        currentLoop = test_emo
        # abbreviate parameter names if possible (e.g. rgb = thisTest_emo.rgb)
        if thisTest_emo != None:
            for paramName in thisTest_emo:
                exec('{} = thisTest_emo[paramName]'.format(paramName))
        
        # --- Prepare to start Routine "Emo" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # Run 'Begin Routine' code from p_ready_emo
        client2.send(facs_cmd1.encode("utf-8"))
        client1.send(head_cmd1_1.encode("utf-8"))
        time.sleep(1)
        # Run 'Begin Routine' code from t_kazu_emo
        if counter_ == 1:
            json_string = json.dumps(json_data_ichi)
        elif counter_ == 2:
            json_string = json.dumps(json_data_ni)
        elif counter_ == 3:
            json_string = json.dumps(json_data_san)
            
        message = roslibpy.Message({'data': json_string})
        time.sleep(1)
        
        tts_topic.publish(message)
        time.sleep(1.5)
        # Run 'Begin Routine' code from p_down_emo
        client1.send(head_cmd_rm.encode("utf-8"))
        client2.send(facs_cmd0.encode("utf-8"))
        time.sleep(1.5)
        # Run 'Begin Routine' code from stim_emo
        num_02 = expression
        num_03 = level_
        num_04 = emo
        
        # NEGATIVE
        if num_02 == 1:
            if num_03 == 1: # 60%
                if num_04 == 1:
                    client2.send(cmd_n_55.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 2:
                    client2.send(cmd_n_60.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 3:
                    client2.send(cmd_n_65.encode("utf-8"))
                    time.sleep(1.5)
                
            elif num_03 == 2: # 90%
                if num_04 == 1:
                    client2.send(cmd_n_85.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 2:
                    client2.send(cmd_n_90.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 3:
                    client2.send(cmd_n_95.encode("utf-8"))
                    time.sleep(1.5)
                
        # POSITIVE
        elif num_02 == 2:
            if num_03 == 1: # 60%
                if num_04 == 1:
                    client2.send(cmd_p_55.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 2:
                    client2.send(cmd_p_60.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 3:
                    client2.send(cmd_p_65.encode("utf-8"))
                    time.sleep(1.5)
                
            elif num_03 == 2: # 90%
                if num_04 == 1:
                    client2.send(cmd_p_85.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 2:
                    client2.send(cmd_p_90.encode("utf-8"))
                    time.sleep(1.5)
                    
                elif num_04 == 3:
                    client2.send(cmd_p_95.encode("utf-8"))
                    time.sleep(1.5)
        
        client1.send(head_cmd1_4.encode("utf-8"))
        client1.send(eye_cmd1_4.encode("utf-8"))
        time.sleep(2)
        
        counter_ = counter_ + 1
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
        thisExp.nextEntry()
        
    # completed 1.0 repeats of 'test_emo'
    
    
    # --- Prepare to start Routine "Emo_end" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from p_end_emo
    client2.send(facs_cmd0.encode("utf-8"))
    time.sleep(1)
    
    client1.send(head_cmd0_2.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    time.sleep(2)
    # keep track of which components have finished
    Emo_endComponents = []
    for thisComponent in Emo_endComponents:
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
    
    # --- Run Routine "Emo_end" ---
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
        for thisComponent in Emo_endComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Emo_end" ---
    for thisComponent in Emo_endComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Emo_end" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Check_beep" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from check_beep
    ser.write(b"5")
    time.sleep(0.5)
    # keep track of which components have finished
    Check_beepComponents = []
    for thisComponent in Check_beepComponents:
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
    
    # --- Run Routine "Check_beep" ---
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
        for thisComponent in Check_beepComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Check_beep" ---
    for thisComponent in Check_beepComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # the Routine "Check_beep" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Check" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    face.keys = []
    face.rt = []
    _face_allKeys = []
    # keep track of which components have finished
    CheckComponents = [check_f_msg, face]
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
        
        # *check_f_msg* updates
        if check_f_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            check_f_msg.frameNStart = frameN  # exact frame index
            check_f_msg.tStart = t  # local t and not account for scr refresh
            check_f_msg.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(check_f_msg, 'tStartRefresh')  # time at next scr refresh
            check_f_msg.setAutoDraw(True)
        
        # *face* updates
        if face.status == NOT_STARTED and t >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            face.frameNStart = frameN  # exact frame index
            face.tStart = t  # local t and not account for scr refresh
            face.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(face, 'tStartRefresh')  # time at next scr refresh
            face.status = STARTED
            # keyboard checking is just starting
            face.clock.reset()  # now t=0
            face.clearEvents(eventType='keyboard')
        if face.status == STARTED:
            theseKeys = face.getKeys(keyList=['1','2','3'], waitRelease=False)
            _face_allKeys.extend(theseKeys)
            if len(_face_allKeys):
                face.keys = _face_allKeys[-1].name  # just the last key pressed
                face.rt = _face_allKeys[-1].rt
                # was this correct?
                if (face.keys == str('')) or (face.keys == ''):
                    face.corr = 1
                else:
                    face.corr = 0
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
    if face.keys in ['', [], None]:  # No response was made
        face.keys = None
        # was no response the correct answer?!
        if str('').lower() == 'none':
           face.corr = 1;  # correct non-response
        else:
           face.corr = 0;  # failed to respond (incorrectly)
    # store data for test_trials (TrialHandler)
    test_trials.addData('face.keys',face.keys)
    test_trials.addData('face.corr', face.corr)
    if face.keys != None:  # we had a response
        test_trials.addData('face.rt', face.rt)
    # the Routine "Check" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    thisExp.nextEntry()
    
# completed 3.0 repeats of 'test_trials'


# --- Prepare to start Routine "Idleon" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# Run 'Begin Routine' code from start_idle
dib_topic.publish(message_idleon)
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
        fin_msg.setAutoDraw(True)
    if fin_msg.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > fin_msg.tStartRefresh + 10-frameTolerance:
            # keep track of stop time/frame for later
            fin_msg.tStop = t  # not accounting for scr refresh
            fin_msg.frameNStop = frameN  # exact frame index
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
