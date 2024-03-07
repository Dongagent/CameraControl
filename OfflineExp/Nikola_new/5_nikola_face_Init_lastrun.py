#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This experiment was created using PsychoPy3 Experiment Builder (v2022.2.5),
    on 12月 05, 2023, at 14:22
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


# Ensure that relative paths start from the same directory as this script
_thisDir = os.path.dirname(os.path.abspath(__file__))
os.chdir(_thisDir)
# Store info about the experiment session
psychopyVersion = '2022.2.5'
expName = 'nikola_face_Init'  # from the Builder filename that created this script
expInfo = {
    'participant': f"{randint(0, 999999):06.0f}",
    'session': '001',
}
expInfo['date'] = data.getDateStr()  # add a simple timestamp
expInfo['expName'] = expName
expInfo['psychopyVersion'] = psychopyVersion

# Data file name stem = absolute path + name; later add .psyexp, .csv, .log, etc
filename = _thisDir + os.sep + u'data/%s_%s_%s' % (expInfo['participant'], expName, expInfo['date'])

# An ExperimentHandler isn't essential but helps with data saving
thisExp = data.ExperimentHandler(name=expName, version='',
    extraInfo=expInfo, runtimeInfo=None,
    originPath='C:\\Users\\SATO\\OneDrive\\Coding\\Dr.Sato\\exp_202311\\5_nikola_face_Init_lastrun.py',
    savePickle=True, saveWideText=False,
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

# --- Initialize components for Routine "Idle_off" ---

# --- Initialize components for Routine "Init_pose" ---

# --- Initialize components for Routine "routine_1" ---
text = visual.TextStim(win=win, name='text',
    text='Nikolaの顔の向きを初期化しました。\nAASSからで「disconnect」してください。',
    font='Open Sans',
    pos=(0, 0), height=0.05, wrapWidth=None, ori=0.0, 
    color='white', colorSpace='rgb', opacity=None, 
    languageStyle='LTR',
    depth=0.0);

# Create some handy timers
globalClock = core.Clock()  # to track the time since experiment started
routineTimer = core.Clock()  # to track time remaining of each (possibly non-slip) routine 

# --- Prepare to start Routine "Init" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# keep track of which components have finished
InitComponents = []
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
# the Routine "Init" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "Idle_off" ---
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
Idle_offComponents = []
for thisComponent in Idle_offComponents:
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

# --- Run Routine "Idle_off" ---
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
    for thisComponent in Idle_offComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Idle_off" ---
for thisComponent in Idle_offComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# the Routine "Idle_off" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "Init_pose" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# Run 'Begin Routine' code from start_pose_demo
time.sleep(1)

client1.send(head_cmd1_1.encode("utf-8"))
client1.send(eye_cmd1_2.encode("utf-8"))
client2.send(facs_cmd1.encode("utf-8"))

time.sleep(3)
# keep track of which components have finished
Init_poseComponents = []
for thisComponent in Init_poseComponents:
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

# --- Run Routine "Init_pose" ---
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
    for thisComponent in Init_poseComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "Init_pose" ---
for thisComponent in Init_poseComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# the Routine "Init_pose" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- Prepare to start Routine "routine_1" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
# keep track of which components have finished
routine_1Components = [text]
for thisComponent in routine_1Components:
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

# --- Run Routine "routine_1" ---
while continueRoutine and routineTimer.getTime() < 100.0:
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
        if tThisFlipGlobal > text.tStartRefresh + 100-frameTolerance:
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
    for thisComponent in routine_1Components:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "routine_1" ---
for thisComponent in routine_1Components:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# using non-slip timing so subtract the expected duration of this Routine (unless ended on request)
if routineForceEnded:
    routineTimer.reset()
else:
    routineTimer.addTime(-100.000000)
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
thisExp.saveAsPickle(filename)
# make sure everything is closed down
if eyetracker:
    eyetracker.setConnectionState(False)
thisExp.abort()  # or data files will save again on exit
win.close()
core.quit()
