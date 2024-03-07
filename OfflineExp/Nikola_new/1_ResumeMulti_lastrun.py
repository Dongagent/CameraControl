#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This experiment was created using PsychoPy3 Experiment Builder (v2022.2.5),
    on 12月 05, 2023, at 14:06
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

head_cmd_gaze = "HeadController={\"id\":\"HeadController\", \"motionTowardObject\":\"\", \"targetMotionMode\":2, \
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
stim_txt = ["ぼくはふつう", "ぼくはかなしい", "ぼくはつらい", "ぼくはうれしい", "ぼくはたのしい"]
stim_exp = ["facs InnerBrow 0 Outerbrow 0 UpperLid 0 LowerLid 0 Cheek 0 NoseWrinkler 0 UpperLip 0 LipCorner 0 MouthCornerStickout 0 MouthOpen 0 JawDrop 0\n", \
            "facs InnerBrow 1.0 Outerbrow 0.0 UpperLid -0.7 LowerLid 0 Cheek 0 NoseWrinkler 0.0 UpperLip 0 LipCorner -0.9 MouthCornerStickout 0 MouthOpen 0 JawDrop 0\n", \
            "facs InnerBrow 1.0 Outerbrow 0.0 UpperLid -0.8 LowerLid 0 Cheek 0 NoseWrinkler 0.0 UpperLip 0 LipCorner -0.9 MouthCornerStickout 0 MouthOpen 0 JawDrop 0\n", \
            "facs InnerBrow 1.0 Outerbrow 0.5 UpperLid 0.2 LowerLid 1 Cheek 1.0 NoseWrinkler 0 UpperLip 0 LipCorner 0.5 MouthCornerStickout 0.5 MouthOpen 0 JawDrop 0\n", \
            "facs InnerBrow 1.0 Outerbrow 0.5 UpperLid 0.3 LowerLid 1 Cheek 1.0 NoseWrinkler 0 UpperLip 0 LipCorner 0.5 MouthCornerStickout 0.5 MouthOpen 0 JawDrop 0\n"]


# Ensure that relative paths start from the same directory as this script
_thisDir = os.path.dirname(os.path.abspath(__file__))
os.chdir(_thisDir)
# Store info about the experiment session
psychopyVersion = '2022.2.5'
expName = 'Resume Gaze'  # from the Builder filename that created this script
expInfo = {
    'participant': f"{randint(0, 999999):06.0f}",
    'subj': '001',
    'cndFileName': ['xlsx/ResumeMulti_1.xlsx', 'xlsx/ResumeMulti_2.xlsx',  'xlsx/ResumeMulti_3.xlsx', 'xlsx/ResumeMulti_4.xlsx', 'xlsx/ResumeMulti_5.xlsx'],
}
# --- Show participant info dialog --
dlg = gui.DlgFromDict(dictionary=expInfo, sortKeys=False, title=expName)
if dlg.OK == False:
    core.quit()  # user pressed cancel
expInfo['date'] = data.getDateStr()  # add a simple timestamp
expInfo['expName'] = expName
expInfo['psychopyVersion'] = psychopyVersion

# Data file name stem = absolute path + name; later add .psyexp, .csv, .log, etc
filename = _thisDir + os.sep + u'data/%s_%s_%s_%s' % (expInfo['subj'], '1-Multi', expInfo['participant'], expInfo['date'])

# An ExperimentHandler isn't essential but helps with data saving
thisExp = data.ExperimentHandler(name=expName, version='',
    extraInfo=expInfo, runtimeInfo=None,
    originPath='C:\\Users\\SATO\\OneDrive\\Coding\\Dr.Sato\\exp_202311\\1_ResumeMulti_lastrun.py',
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

# --- Initialize components for Routine "Outline" ---
outline_1 = visual.TextStim(win=win, name='outline_1',
    text='- 実験の説明 -',
    font='Open Sans',
    pos=(0, 0.4), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
outline_2 = visual.TextStim(win=win, name='outline_2',
    text='各施行の開始時にNikolaが「行きます」と言います\nその後、表情・声の調子・発話内容を変えて感情の表現をします\nビープ音がなった後、手元のキーボード左側の１~９の数字キーを\n使ってNikolaがどのような感情表現だったか、次の2種類の評価を順番に\n入力してください。\n\n①不快~快\n：すごく不快が１、すごく快が９、普通が５を表します\n\n②人らしさ\n：全く人らしくないが１、すごく人らしいが９です。\n\nあまり考えこまずに直感的に入力してください。',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);
outline_3 = visual.TextStim(win=win, name='outline_3',
    text='「Enterキー」を押して次に進みます',
    font='Open Sans',
    pos=(0, -0.4), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-2.0);
key_resp = keyboard.Keyboard()

# --- Initialize components for Routine "Demo_Eyecatch" ---
demo_msg = visual.TextStim(win=win, name='demo_msg',
    text='最初にデモとしてNikolaが\n連続で23パターン表現します\nどのような表現をするか確認してください',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);

# --- Initialize components for Routine "Idleoff" ---

# --- Initialize components for Routine "Demo_init" ---

# --- Initialize components for Routine "Demo_stim" ---

# --- Initialize components for Routine "Exer_Eyecatch" ---
exer_msg_1 = visual.TextStim(win=win, name='exer_msg_1',
    text='次に評定（キーボード入力）の練習をします\nNikolaが表現後に、ブザーが鳴るのでその後\n感情表現に対して不快～快の状態とヒトらしさを\nキーボードの1~9で評定してください',
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

# --- Initialize components for Routine "Exer_Counter" ---
# Run 'Begin Experiment' code from code
trialCounter = 1
text_3 = visual.TextStim(win=win, name='text_3',
    text=None,
    font='Open Sans',
    pos=(0, 0), height=0.1, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);

# --- Initialize components for Routine "Test" ---

# --- Initialize components for Routine "Check_beep" ---

# --- Initialize components for Routine "Check_v" ---
check_v_msg = visual.TextStim(win=win, name='check_v_msg',
    text='感情価の値を1~9のボタンで評価してください',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
valence = keyboard.Keyboard()

# --- Initialize components for Routine "Check_h" ---
check_h_msg = visual.TextStim(win=win, name='check_h_msg',
    text='ヒトらしさの値を1~9のボタンで評価してください。',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
humanness = keyboard.Keyboard()

# --- Initialize components for Routine "Exer_CounterReset" ---

# --- Initialize components for Routine "Idleon" ---

# --- Initialize components for Routine "Test_Standby" ---
standby_msg = visual.TextStim(win=win, name='standby_msg',
    text='練習は終わりです\n\n本番の実験を開始します\n本番は合計56パターンの表現を評定いただきます\n連続で28回実施したあと、休憩を挟み\n残り28回を連続で実施します\nあまり考えこまずに直感的に入力してください\n\n準備できたら「Enterキー」を押してください',
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
# Run 'Begin Experiment' code from break_counter
trialCounter = 1
nRestTrial = 29 # 28回目で休憩をはさむ
test_start = visual.TextStim(win=win, name='test_start',
    text=None,
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);

# --- Initialize components for Routine "Idleon" ---

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
    pos=(0, -0.35), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=-1.0);
key_resp_3 = keyboard.Keyboard()

# --- Initialize components for Routine "Idleoff" ---

# --- Initialize components for Routine "Test" ---

# --- Initialize components for Routine "Check_beep" ---

# --- Initialize components for Routine "Check_v" ---
check_v_msg = visual.TextStim(win=win, name='check_v_msg',
    text='感情価の値を1~9のボタンで評価してください',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
valence = keyboard.Keyboard()

# --- Initialize components for Routine "Check_h" ---
check_h_msg = visual.TextStim(win=win, name='check_h_msg',
    text='ヒトらしさの値を1~9のボタンで評価してください。',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
humanness = keyboard.Keyboard()

# --- Initialize components for Routine "Idleon" ---

# --- Initialize components for Routine "Fin" ---
fin_msg = visual.TextStim(win=win, name='fin_msg',
    text='お疲れさまでした\n実験を終了します',
    font='Open Sans',
    pos=(0, 0), height=0.04, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);
key_resp_5 = keyboard.Keyboard()

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

# set up handler to look after randomisation of conditions etc
switch_demo = data.TrialHandler(nReps=1.0, method='random', 
    extraInfo=expInfo, originPath=-1,
    trialList=[None],
    seed=None, name='switch_demo')
thisExp.addLoop(switch_demo)  # add the loop to the experiment
thisSwitch_demo = switch_demo.trialList[0]  # so we can initialise stimuli with some values
# abbreviate parameter names if possible (e.g. rgb = thisSwitch_demo.rgb)
if thisSwitch_demo != None:
    for paramName in thisSwitch_demo:
        exec('{} = thisSwitch_demo[paramName]'.format(paramName))

for thisSwitch_demo in switch_demo:
    currentLoop = switch_demo
    # abbreviate parameter names if possible (e.g. rgb = thisSwitch_demo.rgb)
    if thisSwitch_demo != None:
        for paramName in thisSwitch_demo:
            exec('{} = thisSwitch_demo[paramName]'.format(paramName))
    
    # --- Prepare to start Routine "Demo_Eyecatch" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # keep track of which components have finished
    Demo_EyecatchComponents = [demo_msg]
    for thisComponent in Demo_EyecatchComponents:
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
    
    # --- Run Routine "Demo_Eyecatch" ---
    while continueRoutine and routineTimer.getTime() < 1.0:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # *demo_msg* updates
        if demo_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            demo_msg.frameNStart = frameN  # exact frame index
            demo_msg.tStart = t  # local t and not account for scr refresh
            demo_msg.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(demo_msg, 'tStartRefresh')  # time at next scr refresh
            demo_msg.setAutoDraw(True)
        if demo_msg.status == STARTED:
            # is it time to stop? (based on global clock, using actual start)
            if tThisFlipGlobal > demo_msg.tStartRefresh + 1.0-frameTolerance:
                # keep track of stop time/frame for later
                demo_msg.tStop = t  # not accounting for scr refresh
                demo_msg.frameNStop = frameN  # exact frame index
                demo_msg.setAutoDraw(False)
        
        # check for quit (typically the Esc key)
        if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
            core.quit()
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineForceEnded = True
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in Demo_EyecatchComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Demo_Eyecatch" ---
    for thisComponent in Demo_EyecatchComponents:
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
    
    # --- Prepare to start Routine "Demo_init" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    # Run 'Begin Routine' code from p_start_demo
    client1.send(head_cmd1_2.encode("utf-8"))
    time.sleep(1.5)
    # Run 'Begin Routine' code from t_go_demo
    # talk ikimasu
    tts_topic.publish(message_ikimasu)
    time.sleep(1.5)
    
    # prepar next demo
    json_data_5 = {
        "Command": "ttsPlaywithParam",
        "TtsID": 1023,
        "Text": "ぼくはたのしい",
        "Volume": 1.3,
        "Rate": 1.0,
        "Pitch": 1.0,
        "Emphasis": 1.0,
        "Emotion": {"Joy": 0.7, "Angry": 0.0, "Sad": 0.0}
    }
    
    time.sleep(1)
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
    demo_trials = data.TrialHandler(nReps=1.0, method='sequential', 
        extraInfo=expInfo, originPath=-1,
        trialList=data.importConditions('xlsx/Random.xlsx'),
        seed=None, name='demo_trials')
    thisExp.addLoop(demo_trials)  # add the loop to the experiment
    thisDemo_trial = demo_trials.trialList[0]  # so we can initialise stimuli with some values
    # abbreviate parameter names if possible (e.g. rgb = thisDemo_trial.rgb)
    if thisDemo_trial != None:
        for paramName in thisDemo_trial:
            exec('{} = thisDemo_trial[paramName]'.format(paramName))
    
    for thisDemo_trial in demo_trials:
        currentLoop = demo_trials
        # abbreviate parameter names if possible (e.g. rgb = thisDemo_trial.rgb)
        if thisDemo_trial != None:
            for paramName in thisDemo_trial:
                exec('{} = thisDemo_trial[paramName]'.format(paramName))
        
        # --- Prepare to start Routine "Demo_stim" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
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
        time.sleep(1)
        
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
        
        # wait thread
        thread1.join()
        thread2.join()
        thread3.join()
        # Run 'Begin Routine' code from _stim_demo
        client1.send(head_cmd1_2.encode("utf-8"))
        client1.send(eye_cmd1_2.encode("utf-8"))
        client2.send(facs_cmd1.encode("utf-8"))
        time.sleep(1)
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
        
    # completed 1.0 repeats of 'demo_trials'
    
    thisExp.nextEntry()
    
# completed 1.0 repeats of 'switch_demo'


# set up handler to look after randomisation of conditions etc
switch_exer = data.TrialHandler(nReps=1.0, method='random', 
    extraInfo=expInfo, originPath=-1,
    trialList=[None],
    seed=None, name='switch_exer')
thisExp.addLoop(switch_exer)  # add the loop to the experiment
thisSwitch_exer = switch_exer.trialList[0]  # so we can initialise stimuli with some values
# abbreviate parameter names if possible (e.g. rgb = thisSwitch_exer.rgb)
if thisSwitch_exer != None:
    for paramName in thisSwitch_exer:
        exec('{} = thisSwitch_exer[paramName]'.format(paramName))

for thisSwitch_exer in switch_exer:
    currentLoop = switch_exer
    # abbreviate parameter names if possible (e.g. rgb = thisSwitch_exer.rgb)
    if thisSwitch_exer != None:
        for paramName in thisSwitch_exer:
            exec('{} = thisSwitch_exer[paramName]'.format(paramName))
    
    # --- Prepare to start Routine "Exer_Eyecatch" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    key_resp_2.keys = []
    key_resp_2.rt = []
    _key_resp_2_allKeys = []
    # keep track of which components have finished
    Exer_EyecatchComponents = [exer_msg_1, exer_msg_2, key_resp_2]
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
    # check responses
    if key_resp_2.keys in ['', [], None]:  # No response was made
        key_resp_2.keys = None
    switch_exer.addData('key_resp_2.keys',key_resp_2.keys)
    if key_resp_2.keys != None:  # we had a response
        switch_exer.addData('key_resp_2.rt', key_resp_2.rt)
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
        if count_sc.status == NOT_STARTED and tThisFlip >= 0-frameTolerance:
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
    exer_trials = data.TrialHandler(nReps=1.0, method='sequential', 
        extraInfo=expInfo, originPath=-1,
        trialList=data.importConditions('xlsx/exercise.xlsx'),
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
        
        # --- Prepare to start Routine "Exer_Counter" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # Run 'Begin Routine' code from code
        text_3.setText(trialCounter)
        trialCounter = trialCounter + 1
        # keep track of which components have finished
        Exer_CounterComponents = [text_3]
        for thisComponent in Exer_CounterComponents:
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
        
        # --- Run Routine "Exer_Counter" ---
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
            for thisComponent in Exer_CounterComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
        
        # --- Ending Routine "Exer_Counter" ---
        for thisComponent in Exer_CounterComponents:
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
        client1.send(head_cmd1_2.encode("utf-8"))
        time.sleep(1)
        # Run 'Begin Routine' code from t_go_test
        # Talk message
        tts_topic.publish(message_ikimasu)
        time.sleep(1.5)
        # Run 'Begin Routine' code from p_start_test
        client2.send(facs_cmd1.encode("utf-8"))
        client1.send(eye_cmd1_2.encode("utf-8"))
        time.sleep(1)
        # Run 'Begin Routine' code from stim_test
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
        time.sleep(1)
        
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
        # Run 'Begin Routine' code from _stim_test
        client1.send(head_cmd1_1.encode("utf-8"))
        client1.send(eye_cmd1_2.encode("utf-8"))
        #client2.send(facs_cmd1.encode("utf-8"))
        
        time.sleep(1)
        # Run 'Begin Routine' code from _p_init_test
        client1.send(head_cmd0_2.encode("utf-8"))
        client1.send(eye_cmd0_2.encode("utf-8"))
        client2.send(facs_cmd0.encode("utf-8"))
        time.sleep(1.5)
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
        
        # --- Prepare to start Routine "Check_v" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        valence.keys = []
        valence.rt = []
        _valence_allKeys = []
        # keep track of which components have finished
        Check_vComponents = [check_v_msg, valence]
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
            
            # *check_v_msg* updates
            if check_v_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                check_v_msg.frameNStart = frameN  # exact frame index
                check_v_msg.tStart = t  # local t and not account for scr refresh
                check_v_msg.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(check_v_msg, 'tStartRefresh')  # time at next scr refresh
                check_v_msg.setAutoDraw(True)
            
            # *valence* updates
            if valence.status == NOT_STARTED and t >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                valence.frameNStart = frameN  # exact frame index
                valence.tStart = t  # local t and not account for scr refresh
                valence.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(valence, 'tStartRefresh')  # time at next scr refresh
                # add timestamp to datafile
                thisExp.addData('valence.started', t)
                valence.status = STARTED
                # keyboard checking is just starting
                valence.clock.reset()  # now t=0
                valence.clearEvents(eventType='keyboard')
            if valence.status == STARTED:
                theseKeys = valence.getKeys(keyList=['1','2','3','4','5','6','7','8','9', 'escape', 'return'], waitRelease=False)
                _valence_allKeys.extend(theseKeys)
                if len(_valence_allKeys):
                    valence.keys = _valence_allKeys[-1].name  # just the last key pressed
                    valence.rt = _valence_allKeys[-1].rt
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
        if valence.keys in ['', [], None]:  # No response was made
            valence.keys = None
        exer_trials.addData('valence.keys',valence.keys)
        if valence.keys != None:  # we had a response
            exer_trials.addData('valence.rt', valence.rt)
        # the Routine "Check_v" was not non-slip safe, so reset the non-slip timer
        routineTimer.reset()
        
        # --- Prepare to start Routine "Check_h" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        humanness.keys = []
        humanness.rt = []
        _humanness_allKeys = []
        # keep track of which components have finished
        Check_hComponents = [check_h_msg, humanness]
        for thisComponent in Check_hComponents:
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
        
        # --- Run Routine "Check_h" ---
        while continueRoutine:
            # get current time
            t = routineTimer.getTime()
            tThisFlip = win.getFutureFlipTime(clock=routineTimer)
            tThisFlipGlobal = win.getFutureFlipTime(clock=None)
            frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
            # update/draw components on each frame
            
            # *check_h_msg* updates
            if check_h_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                check_h_msg.frameNStart = frameN  # exact frame index
                check_h_msg.tStart = t  # local t and not account for scr refresh
                check_h_msg.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(check_h_msg, 'tStartRefresh')  # time at next scr refresh
                check_h_msg.setAutoDraw(True)
            
            # *humanness* updates
            if humanness.status == NOT_STARTED and t >= 0.0-frameTolerance:
                # keep track of start time/frame for later
                humanness.frameNStart = frameN  # exact frame index
                humanness.tStart = t  # local t and not account for scr refresh
                humanness.tStartRefresh = tThisFlipGlobal  # on global time
                win.timeOnFlip(humanness, 'tStartRefresh')  # time at next scr refresh
                # add timestamp to datafile
                thisExp.addData('humanness.started', t)
                humanness.status = STARTED
                # keyboard checking is just starting
                humanness.clock.reset()  # now t=0
                humanness.clearEvents(eventType='keyboard')
            if humanness.status == STARTED:
                theseKeys = humanness.getKeys(keyList=['1','2','3','4','5','6','7','8','9', 'escape', 'return'], waitRelease=False)
                _humanness_allKeys.extend(theseKeys)
                if len(_humanness_allKeys):
                    humanness.keys = _humanness_allKeys[-1].name  # just the last key pressed
                    humanness.rt = _humanness_allKeys[-1].rt
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
            for thisComponent in Check_hComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
        
        # --- Ending Routine "Check_h" ---
        for thisComponent in Check_hComponents:
            if hasattr(thisComponent, "setAutoDraw"):
                thisComponent.setAutoDraw(False)
        # check responses
        if humanness.keys in ['', [], None]:  # No response was made
            humanness.keys = None
        exer_trials.addData('humanness.keys',humanness.keys)
        if humanness.keys != None:  # we had a response
            exer_trials.addData('humanness.rt', humanness.rt)
        # the Routine "Check_h" was not non-slip safe, so reset the non-slip timer
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
    # Run 'Begin Routine' code from start_idle_2
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
    thisExp.nextEntry()
    
# completed 1.0 repeats of 'switch_exer'


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
    if count_sc.status == NOT_STARTED and tThisFlip >= 0-frameTolerance:
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
test_trials = data.TrialHandler(nReps=1.0, method='sequential', 
    extraInfo=expInfo, originPath=-1,
    trialList=data.importConditions(expInfo['cndFileName']),
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
    
    test_start.setText(trialCounter)
    trialCounter = trialCounter + 1
    
    # keep track of which components have finished
    CounterComponents = [test_start]
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
        
        # *test_start* updates
        if test_start.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            test_start.frameNStart = frameN  # exact frame index
            test_start.tStart = t  # local t and not account for scr refresh
            test_start.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(test_start, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'test_start.started')
            test_start.setAutoDraw(True)
        if test_start.status == STARTED:
            # is it time to stop? (based on global clock, using actual start)
            if tThisFlipGlobal > test_start.tStartRefresh + 1-frameTolerance:
                # keep track of stop time/frame for later
                test_start.tStop = t  # not accounting for scr refresh
                test_start.frameNStop = frameN  # exact frame index
                # add timestamp to datafile
                thisExp.timestampOnFlip(win, 'test_start.stopped')
                test_start.setAutoDraw(False)
        
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
    recess_triger = data.TrialHandler(nReps=isRest, method='random', 
        extraInfo=expInfo, originPath=-1,
        trialList=[None],
        seed=None, name='recess_triger')
    thisExp.addLoop(recess_triger)  # add the loop to the experiment
    thisRecess_triger = recess_triger.trialList[0]  # so we can initialise stimuli with some values
    # abbreviate parameter names if possible (e.g. rgb = thisRecess_triger.rgb)
    if thisRecess_triger != None:
        for paramName in thisRecess_triger:
            exec('{} = thisRecess_triger[paramName]'.format(paramName))
    
    for thisRecess_triger in recess_triger:
        currentLoop = recess_triger
        # abbreviate parameter names if possible (e.g. rgb = thisRecess_triger.rgb)
        if thisRecess_triger != None:
            for paramName in thisRecess_triger:
                exec('{} = thisRecess_triger[paramName]'.format(paramName))
        
        # --- Prepare to start Routine "Idleon" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        # Run 'Begin Routine' code from start_idle_2
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
        
        # --- Prepare to start Routine "Routine" ---
        continueRoutine = True
        routineForceEnded = False
        # update component parameters for each repeat
        key_resp_3.keys = []
        key_resp_3.rt = []
        _key_resp_3_allKeys = []
        # keep track of which components have finished
        RoutineComponents = [countd_msg, recess, key_resp_3]
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
                # add timestamp to datafile
                thisExp.timestampOnFlip(win, 'countd_msg.started')
                countd_msg.setAutoDraw(True)
            if countd_msg.status == STARTED:
                # is it time to stop? (based on global clock, using actual start)
                if tThisFlipGlobal > countd_msg.tStartRefresh + 300-frameTolerance:
                    # keep track of stop time/frame for later
                    countd_msg.tStop = t  # not accounting for scr refresh
                    countd_msg.frameNStop = frameN  # exact frame index
                    # add timestamp to datafile
                    thisExp.timestampOnFlip(win, 'countd_msg.stopped')
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
                recess.setAutoDraw(True)
            
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
        
    # completed isRest repeats of 'recess_triger'
    
    
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
    client1.send(head_cmd1_2.encode("utf-8"))
    time.sleep(1)
    # Run 'Begin Routine' code from t_go_test
    # Talk message
    tts_topic.publish(message_ikimasu)
    time.sleep(1.5)
    # Run 'Begin Routine' code from p_start_test
    client2.send(facs_cmd1.encode("utf-8"))
    client1.send(eye_cmd1_2.encode("utf-8"))
    time.sleep(1)
    # Run 'Begin Routine' code from stim_test
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
    time.sleep(1)
    
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
    # Run 'Begin Routine' code from _stim_test
    client1.send(head_cmd1_1.encode("utf-8"))
    client1.send(eye_cmd1_2.encode("utf-8"))
    #client2.send(facs_cmd1.encode("utf-8"))
    
    time.sleep(1)
    # Run 'Begin Routine' code from _p_init_test
    client1.send(head_cmd0_2.encode("utf-8"))
    client1.send(eye_cmd0_2.encode("utf-8"))
    client2.send(facs_cmd0.encode("utf-8"))
    time.sleep(1.5)
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
    
    # --- Prepare to start Routine "Check_v" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    valence.keys = []
    valence.rt = []
    _valence_allKeys = []
    # keep track of which components have finished
    Check_vComponents = [check_v_msg, valence]
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
        
        # *check_v_msg* updates
        if check_v_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            check_v_msg.frameNStart = frameN  # exact frame index
            check_v_msg.tStart = t  # local t and not account for scr refresh
            check_v_msg.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(check_v_msg, 'tStartRefresh')  # time at next scr refresh
            check_v_msg.setAutoDraw(True)
        
        # *valence* updates
        if valence.status == NOT_STARTED and t >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            valence.frameNStart = frameN  # exact frame index
            valence.tStart = t  # local t and not account for scr refresh
            valence.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(valence, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.addData('valence.started', t)
            valence.status = STARTED
            # keyboard checking is just starting
            valence.clock.reset()  # now t=0
            valence.clearEvents(eventType='keyboard')
        if valence.status == STARTED:
            theseKeys = valence.getKeys(keyList=['1','2','3','4','5','6','7','8','9', 'escape', 'return'], waitRelease=False)
            _valence_allKeys.extend(theseKeys)
            if len(_valence_allKeys):
                valence.keys = _valence_allKeys[-1].name  # just the last key pressed
                valence.rt = _valence_allKeys[-1].rt
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
    if valence.keys in ['', [], None]:  # No response was made
        valence.keys = None
    test_trials.addData('valence.keys',valence.keys)
    if valence.keys != None:  # we had a response
        test_trials.addData('valence.rt', valence.rt)
    # the Routine "Check_v" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    
    # --- Prepare to start Routine "Check_h" ---
    continueRoutine = True
    routineForceEnded = False
    # update component parameters for each repeat
    humanness.keys = []
    humanness.rt = []
    _humanness_allKeys = []
    # keep track of which components have finished
    Check_hComponents = [check_h_msg, humanness]
    for thisComponent in Check_hComponents:
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
    
    # --- Run Routine "Check_h" ---
    while continueRoutine:
        # get current time
        t = routineTimer.getTime()
        tThisFlip = win.getFutureFlipTime(clock=routineTimer)
        tThisFlipGlobal = win.getFutureFlipTime(clock=None)
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # *check_h_msg* updates
        if check_h_msg.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            check_h_msg.frameNStart = frameN  # exact frame index
            check_h_msg.tStart = t  # local t and not account for scr refresh
            check_h_msg.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(check_h_msg, 'tStartRefresh')  # time at next scr refresh
            check_h_msg.setAutoDraw(True)
        
        # *humanness* updates
        if humanness.status == NOT_STARTED and t >= 0.0-frameTolerance:
            # keep track of start time/frame for later
            humanness.frameNStart = frameN  # exact frame index
            humanness.tStart = t  # local t and not account for scr refresh
            humanness.tStartRefresh = tThisFlipGlobal  # on global time
            win.timeOnFlip(humanness, 'tStartRefresh')  # time at next scr refresh
            # add timestamp to datafile
            thisExp.addData('humanness.started', t)
            humanness.status = STARTED
            # keyboard checking is just starting
            humanness.clock.reset()  # now t=0
            humanness.clearEvents(eventType='keyboard')
        if humanness.status == STARTED:
            theseKeys = humanness.getKeys(keyList=['1','2','3','4','5','6','7','8','9', 'escape', 'return'], waitRelease=False)
            _humanness_allKeys.extend(theseKeys)
            if len(_humanness_allKeys):
                humanness.keys = _humanness_allKeys[-1].name  # just the last key pressed
                humanness.rt = _humanness_allKeys[-1].rt
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
        for thisComponent in Check_hComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
    
    # --- Ending Routine "Check_h" ---
    for thisComponent in Check_hComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    # check responses
    if humanness.keys in ['', [], None]:  # No response was made
        humanness.keys = None
    test_trials.addData('humanness.keys',humanness.keys)
    if humanness.keys != None:  # we had a response
        test_trials.addData('humanness.rt', humanness.rt)
    # the Routine "Check_h" was not non-slip safe, so reset the non-slip timer
    routineTimer.reset()
    thisExp.nextEntry()
    
# completed 1.0 repeats of 'test_trials'


# --- Prepare to start Routine "Idleon" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# Run 'Begin Routine' code from start_idle_2
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
key_resp_5.keys = []
key_resp_5.rt = []
_key_resp_5_allKeys = []
# keep track of which components have finished
FinComponents = [fin_msg, key_resp_5]
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
    
    # *key_resp_5* updates
    if key_resp_5.status == NOT_STARTED and t >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        key_resp_5.frameNStart = frameN  # exact frame index
        key_resp_5.tStart = t  # local t and not account for scr refresh
        key_resp_5.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(key_resp_5, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.addData('key_resp_5.started', t)
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
