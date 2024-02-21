#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Author: Dongsheng Yang
# @Email:  yang.dongsheng.46w@st.kyoto-u.ac.jp
# @Copyright = Copyright 2021, The Riken Robotics Project
# @Date:   2021-05-20 18:19:38
# @Last Modified by:   dongshengyang
# @Last Modified time: 2021-05-20 18:21:38
'''
__author__ = "Dongsheng Yang"
__copyright__ = "Copyright 2021, The Riken Robotics Project"
__version__ = "1.0.0"
__maintainer__ = "Dongsheng Yang"
__email__ = "yang.dongsheng.46w@st.kyoto-u.ac.jp"
__status__ = "Developing"
'''

import random
from typing import Counter
import defaultPose

# import socket
from threading import Thread
import cv2, time, copy, sys, math
import os, subprocess
import platform
import numpy as np
import pandas as pd
from bayes_opt import BayesianOptimization
from bayes_opt.logger import JSONLogger
from bayes_opt.event import Events
# ----- for ros------
import rospy, json, base64
from std_msgs.msg import String
import struct
# ----- for ros END------

# ----- for new model -----
# from SiameseRankNet import SiameseRankNet_analysis
# ----- for new model END -----
SPACE = ' '
LINUXVIDEOPATH = '/dev/video2' # ffplay # v4l2-ctl --list-devices
loopFlag = 0 # 0 - py-feat, 1 - human
DEBUG = 0
FEAT_VERSION = 0 # 0 - py-feat 0.3.7 , 1 - py-feat 0.6.1
# 0 - Run; 
# 1 - Debuging with robot; 
# 2 - Debug WITHOUT robot; for image debuging
# 3 - Debug without pic, with  robot; 
# 4 - Debug without pic, without robot;

headYaw_fix_flag = False
headYaw_fix = 105

global smoothSleepTime
smoothSleepTime = 0.2

# pyfeat
from feat import Detector
detector = ''

class WebcamStreamWidget(object):
    def __init__(self, stream_id=0, width=1280, height=720):
        # initialize the video camera stream and read the first frame
        print("[INFO]WebcamStreamWidget initializing...")

        self.stream_id = stream_id # default is 0 for main camera 
        
        # opening video capture stream vcap
        self.vcap = cv2.VideoCapture(stream_id)
        # set resolution to 1920x1080
        self.vcap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.vcap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.vcap.isOpened:
            raise AttributeError("[ERROR]: Error accessing webcam stream.")
        
        # reading a single frame from vcap stream for initializing 
        self.status , self.frame = self.vcap.read()
        if not self.status:
            print('[Exiting] No more frames to read')
            raise Exception 
        # self.stopped is initialized to False 
        self.stopped = True
        # thread instantiation  
        self.vthread = Thread(target=self.update, args=())
        self.vthread.daemon = True # daemon threads run in background 
        print("[INFO]WebcamStreamWidget initialized.")

    # start vthread 
    def start(self):
        self.stopped = False
        self.vthread.start()

    # the target method passed to vthread for reading the next available frame  
    def update(self):
        while True :
            if self.stopped is True :
                break
            self.status, self.frame = self.vcap.read()
            time.sleep(.01) # delay for simulating video processing
            if self.status is False :
                print('[Exiting] No more frames to read')
                self.stopped = True
                break 
        self.vcap.release()

    def read(self):
        return self.frame

    # stop reading frames
    def stop(self):
        self.stopped = True
        
    def save_frame(self, path):
        if not self.stopped:
            cv2.imwrite(path, self.read())
        else:
            raise AttributeError('frame not found')

    # Video recording methods
    def start_video_recording(self, path, codec='XVID', fps=60.0):
        self.video_path = path
        self.video_codec = codec
        self.video_fps = fps
        self.video_stopped = False

        self.video_thread = Thread(target=self._record_video)
        self.video_thread.daemon = True
        self.video_thread.start()

    def _record_video(self):
        fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
        out = cv2.VideoWriter(self.video_path, fourcc, self.video_fps, (self.frame.shape[1], self.frame.shape[0]))

        while not self.video_stopped:
            if self.frame is not None:
                out.write(self.frame)
            time.sleep(1 / self.video_fps)

        out.release()
        print(f'Video saved to {self.video_path}')

    def stop_video_recording(self):
        self.video_stopped = True
        if self.video_thread.is_alive():
            self.video_thread.join()


    

    



class robot:
    def __init__(self, duration=3, webcam=True, fps=60):
        # initialization of robot
        print("[INFO]robot initializing...")
        # Return to normal state first
        # Example: robotParams = {'1': 64, '2': 64, '3': 128, ...}
        self.connection = True
        self.robotParams = {}
        self.executionCode = ''
        self.stableState = [
            64, 64, 128, 128, 128, 
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 32, 128, 128, headYaw_fix
        ]

        # initialization of States, like [0, 0, 0, ... , 0]
        self.lastState = self.stableState # initialization
        self.nextState = self.stableState # initialization

        # initialization of lastParams
        self.lastParams = self.robotParams
        self.defaultPose = defaultPose.defaultPose
        self.AUPose = defaultPose.actionUnitParams

        # Camera Parameters
        if LINUXVIDEOPATH == '/dev/video2':
            self.DEVICE_ID = 2
        else:
            self.DEVICE_ID = 0
        self.WIDTH = 1280
        self.HEIGHT = 720
        self.FPS = fps
        self.FRAMERATE = fps
        self.counter = 0 # used in the fileName
        self.fileName =  ""
        self.readablefileName = ''
        self.VIDEOSIZE = "1280x720"
        self.DURATION = duration
        self.client = ""
        self.photoform = '%01d.png' 

        # human experiment
        self.bestImg = ""

        # Final initialization
        self.initialize_robotParams()
        self.return_to_stable_state()
        
        # webcam stream thread, initialize
        if webcam:
            self.webcam_stream_widget = WebcamStreamWidget(self.DEVICE_ID, self.WIDTH, self.HEIGHT)
            self.webcam_stream_widget.start()
            print("[INFO]robot and webcam initialized.")
        else:
            print("[INFO]robot initialized.")

    def take_picture(self, isUsingCounter=True, appendix='', folder=''):
        # we don't need this
        # Allright, we need this 
        self.counter += 1
        if isUsingCounter:
            self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S_No", time.localtime()) + str(self.counter)
            if appendix:
                self.fileName += "_" + appendix + self.photoform
            else:
                self.fileName += self.photoform
        else:
            self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) 
            if appendix:
                self.fileName += "_" + appendix + self.photoform
            else:
                self.fileName += self.photoform
        if DEBUG == 3 or DEBUG == 4:
            print("Filename is {}".format(self.fileName))
            return
        if folder:
            folderPath = "image_analysis/{}/".format(folder)
            if not os.path.exists(folderPath):
                try:
                    os.mkdir(folderPath)
                except Exception as e:
                    print(e)
        if not folder:
            self.fileName = "image_analysis/temp/{}".format(self.fileName)
        else:
            self.fileName = folderPath + self.fileName
        
        self.readablefileName = self.fileName[:-8] + '2.png'
        

        if os.path.exists(self.fileName):
            raise Exception("Same File!")
        if "Linux" in platform.platform():
            # Remember to check the path everytime.
            videoPath = LINUXVIDEOPATH
            fParam = "v4l2"
            videoTypeParm = "-input_format"
        elif "Windows" in platform.platform():
            videoPath = "video='C922 Pro Stream Webcam'"
            fParam = "dshow" 
            videoTypeParm = "-vcodec"

        # only the command is different from take_video
        command = "ffmpeg -f {} -i {} -vframes 2 {}".format(
            fParam, 
            videoPath, 
            self.fileName)
        # ffmpeg -f v4l2 -i /dev/video2 -vframes 1 /home/dongagent/github/CameraControl/algorithm/test.png
        print(command)
        if "Linux" in platform.platform():
            # Linux
            return subprocess.Popen([command], stdout=subprocess.PIPE, shell=True)
        elif "Windows" in platform.platform():
            # Windows
            return subprocess.Popen(["pwsh", "-Command", command], stdout=subprocess.PIPE)

    def take_picture_cv(self, isUsingCounter=True, appendix='', folder=''):
        self.counter += 1
        if isUsingCounter:
            self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S_No", time.localtime()) + str(self.counter)
            if appendix:
                self.fileName += "_" + appendix + '.png'
            else:
                self.fileName += '.png'
        else:
            self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) 
            if appendix:
                self.fileName += "_" + appendix + '.png'
            else:
                self.fileName += '.png'
        if DEBUG == 3 or DEBUG == 4:    
            print("[take_picture_cv] Filename is {}".format(self.fileName))
            return
        if folder:
            folderPath = "image_analysis/{}/".format(folder)
            if not os.path.exists(folderPath):
                try:
                    os.mkdir(folderPath)
                except Exception as e:
                    print(e)
        if not folder:
            self.fileName = "image_analysis/temp/{}".format(self.fileName)
        else:
            self.fileName = folderPath + self.fileName
        
        self.readablefileName = self.fileName
        
        # save file in another thread
        if self.readablefileName:
            self.webcam_stream_widget.save_frame(self.readablefileName)
            print('[INFO]frame captured.')
        else:
            raise ValueError('[ValueError] self.readablefileName is {}.'.format(self.readablefileName))

    # use webcam_stream_widget to take video
    def start_taking_video(self, isUsingCounter=True, appendix='', folder=''):
        self.counter += 1
        if isUsingCounter:
            self.fileName = str(self.counter) + '_' + time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
            if appendix:
                self.fileName += "_" + appendix + ".mkv"
            else:
                self.fileName += ".mkv"
        else:
            self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) 
            if appendix:
                self.fileName += "_" + appendix + ".mkv"
            else:
                self.fileName += ".mkv"
        if DEBUG == 3 or DEBUG == 4:
            print("Filename is {}".format(self.fileName))
            return

        if not os.path.exists('video_analysis/temp'):
            os.makedirs('video_analysis/temp')

        if folder:
            folderPath = "video_analysis/{}/".format(folder)
            if not os.path.exists(folderPath):
                try:
                    os.makedirs(folderPath)
                    print('[INFO]make video folder')
                except Exception as e:
                    print(e)
        if not folder:
            self.fileName = "video_analysis/temp/{}".format(self.fileName)
        else:
            self.fileName = folderPath + self.fileName
        
        self.readablefileName = self.fileName
        
        # save file in another thread
        if self.readablefileName:
            self.webcam_stream_widget.start_video_recording(self.readablefileName)
            print('[INFO]video recording started.')
            return 0
        else:
            raise ValueError('[ValueError] self.readablefileName is {}.'.format(self.readablefileName))
        
        return 404

    # stop video recording
    def stop_taking_video(self):
        self.webcam_stream_widget.stop_video_recording()
        print('[INFO]video recording stopped.')

    def initialize_robotParams(self):
        # initialize robotParams like {"x1":0, "x2":0, ... , "x35": 0}
        print("initialize_robotParams")
        for i in range(1, 36):
            codeNum = "x{}".format(i)
            self.robotParams[codeNum] = 0
    def randomize_robotParams(self):
        # ALERT!!
        # DO NOT USE IT UNLESS YOU KNOW WHAT YOU ARE DOING
        pass
        """
        for j,k in self.robotParams.items():
            self.robotParams[j] = random.randint(0, 10)
        self.__check_robotParams()
        """
    def return_to_stable_state(self):
        # set all params in robotParams to 0
        stableState = [
            64, 64, 128, 128, 128, 
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 32, 128, 128, 95
        ]
        for i in range(1, 36):
            self.robotParams["x{}".format(i)] = stableState[i - 1]
        print("\n")
        self.__check_robotParams()
        # Drive the robot to the 
        self.connect_ros(isSmoothly=True)
        time.sleep(0.5)
        print("[INFO]return_to_stable_state, self.robotParams are all set")
    def transfer_robotParams_to_states(self, params):
        states = [0 for x in range(35)]
        for i in range(1, 36):
            states[i - 1] = params["x{}".format(i)]
        return states
    def switch_to_defaultPose(self, pose):
        # pose number is [1,2,3,4,5,6, ,8,9,10, ,12,13,14,15,16] 
        # 1 標準 2 笑顔 3 怒り 4 悲しみ 5 驚き 6 微笑 
        # 7 None 8 くさい 9 ウィンク左 10 ウィンク右 11 None 
        # 12 「あ」 13「い」 14「う」 15「え」 16「お」
        assert pose in [1,2,3,4,5,6,8,9,10,12,13,14,15,16], "ERROR! The selected pose is not existed."
        poseNum = pose - 1
        self.change_robotParams(self.defaultPose[poseNum])
    def switch_to_customizedPose(self, customizedPose):
        # Notice: customizedPose should clarify 35 axes
        # E.g. [1, 2, 3, 0, ... , 255]
        assert len(self.nextState) == 35, "ERROR! The customizedPose don't have 35 axes." + self.nextState
        assert len(customizedPose) == 35, "ERROR! The customizedPose don't have 35 axes."
        # States
        self.lastState = self.nextState
        self.nextState = customizedPose
        if DEBUG >= 1:
            print("[INFO]self.lastState", self.lastState)
            print("[INFO]self.nextState", self.nextState)

        # params
        self.change_robotParams(customizedPose)
        if DEBUG >= 1:
            print(self.robotParams)
    def change_robotParams(self, params):
        # change robot params
        assert isinstance(params, list), isinstance(params, list)
        # Set the value of lastParams to be the current robotParams
        self.lastParams = copy.deepcopy(self.robotParams)
        # Construct current robotParams
        for i in range(1, 36):
            self.robotParams["x{}".format(i)] = params[i - 1]
        self.__check_robotParams()





    def sigmoid_smooth_execution_mode(self, steps = 20, total_time = 2, isSigmoidForTime = False, sigmoid_factor=1):
        if self.robotParams:
            stepNum = steps
            for i in range(0, stepNum):
                frab = (i + 1) / float(stepNum)
                currentParams = {}
                
                
                for k in self.lastParams.keys():
                    start = self.lastParams[k]
                    end = self.robotParams[k]
                    currentParams[k] = start + (end - start) * sigmoid(sigmoid_factor * (i / stepNum - 0.5))
                # print('DEBUG:', currentParams)
                
                self.nextState = self.transfer_robotParams_to_states(currentParams)
                # self.__sendExecutionCode() # Use socket
                self.ros_talker()# Use ROS

                # MODIFY HERE if you want to setup the duration of emotion
                global smoothSleepTime
                if isSigmoidForTime:
                    total_time = smoothSleepTime * steps
                    time_interval = total_time * sigmoid(7 * (i / stepNum - 0.5))
                    # print(time_interval)
                    time.sleep(time_interval)
                else:
                    time.sleep(smoothSleepTime)

    def smooth_execution_mode(self, steps = 20):
        # steps: the middle steps between two robot expressions, default value is 5
        if self.robotParams:
            stepNum = steps

            for i in range(0, stepNum):
                frab = (i + 1) / float(stepNum)
                currentParams = {}
                for k in self.lastParams.keys():
                    interval = abs(self.lastParams[k] - self.robotParams[k]) * frab  
                    currentParams[k] = int(self.lastParams[k] - interval) if self.lastParams[k] > self.robotParams[k] else int(self.lastParams[k] + interval)
                    # if k == "1":
                    #     print(self.lastParams[k], self.robotParams[k], interval, currentParams[k])


                self.nextState = self.transfer_robotParams_to_states(currentParams)
                # self.__sendExecutionCode() # Use socket
                self.ros_talker()# Use ROS

                # MODIFY HERE if you want to setup the duration of emotion
                global smoothSleepTime
                time.sleep(smoothSleepTime)

    # @deprecated
    def normal_execution_mode(self):
        # self.__sendExecutionCode() # Use socket
        self.ros_talker()# Use ROS

    def ros_talker(self):
        rospy.init_node('rcpublisher', anonymous=True)
        pub = rospy.Publisher('rc/command', String, queue_size=10)
        sub = rospy.Subscriber('rc/return', String, self.sub_callback)
        r = rospy.Rate(10) # speed

        target = self.nextState

        # if you wan to use MoveAllAxes
        dictdata = {
            "Command": "MoveAllAxes",
            "Vals": list2string(target)
        }
        
        if not rospy.is_shutdown():
            strdata = json.dumps(dictdata)
            # print("strdata", strdata)

            pub.publish(strdata)
            # if rospy.Message:
            #     print(rospy.Message)
            # r.sleep()
        
    # Now use ROS instead
    # @deprecated
    # def __sendExecutionCode(self):
    #     # This function cannot be called outside
    #     assert "move" in self.executionCode
    #     msg = self.executionCode # message

    #     self.client.send(msg.encode('utf-8')) # send a message to server, python3 only receive byte data
    #     data = self.client.recv(1024) # receive a message, the maximum length is 1024 bytes
    #     # print('recv:', data.decode()) # print the data I received
    #     if "OK" not in data.decode():
    #         raise Exception("ERROR! Did NOT receive 'OK'")

    def sub_callback(self, data):
        recv_dict = json.loads(data.data)
        if recv_dict['Message'] == "PotentioValsBase64":
            potval_bin = base64.b64decode(recv_dict['ValsBase64'])
            potval = struct.unpack('%sB' % len(potval_bin), potval_bin)
            potentio = list(potval)
            print(potentio)

        if recv_dict['Message'] == "PotentioVals":
            potvalstr = recv_dict['Vals'].split(',')
            potentio = map((lambda x: int(x)), potvalstr)
            print(potentio)

        if recv_dict['Message'] == "PotentioAxes":
            potaxisstr = recv_dict['Axes'].split(',')
            axiswithpotentio = map((lambda x: int(x)), potaxisstr)
            print(axiswithpotentio)

    def connect_ros(self, isSmoothly=True, isRecording=False, appendix="", steps=20, timeIntervalBeforeExp=1, isUsingSigmoid=False, sigmoid_factor=1):

        if DEBUG == 2 or DEBUG == 4:
            print('you are DEBUGING')
            return 0
        '''
        # socket method
        # @deprecated
        
        # self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                # create socket object
        
        # Please use ipconfig on the server to check the ip first. It may change every time we open the server.
        host = '172.27.174.125'                                                      # set server address
        port = 12000                                                                 # set port

        # --------- DEBUG -------------
        if DEBUG == 2:
            print("function connect_ros:", self.robotParams)
            # Test fileName
            if isRecording:
                self.take_video(isUsingCounter=False, appendix=appendix)
            return
        
        # --------- DEBUG END -------------
        
        
        try:
            self.client.connect((host, port))
            self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except Exception as e:
            print("Connection Failed, ERROR code is: ", e)
            self.connection = False
        '''

        try:
            # self.ros_talker()
            if self.connection:
                # if we need to fix the headYaw, we need to change the value of x35 for every connect_ros
                if headYaw_fix_flag:
                    self.robotParams["x35"] = headYaw_fix

                # Start Record if isRecording
                # if isRecording:
                #     # Please set which recording system you want to use. Video or image.
                #     process = self.start_taking_video(isUsingCounter=False, appendix=appendix)
                #     # time.sleep(1)
                # Smoothly execute
                if isSmoothly:
                    print("[INFO]Smoothly execution activated")
                    # if isRecording:
                    #     time.sleep(timeIntervalBeforeExp) # Sleep 1 second by default to wait for the start of the video 
                    if not isUsingSigmoid:  
                        self.smooth_execution_mode(steps)
                    else:
                        # using Sigmoid
                        self.sigmoid_smooth_execution_mode(steps=steps, sigmoid_factor=sigmoid_factor)
                # Otherwise
                else:
                    self.normal_execution_mode()
                # self.client.close() # use ros now
                
                # # Close Record
                # if isRecording:
                #     process.wait()
                #     if process.returncode != 0:
                #         print(process.stdout.readlines())
                #         raise Exception("The subprocess does NOT end.")
            else:
                raise Exception("Connection Failed")
        except rospy.ROSInterruptException: 
            print(rospy.ROSInterruptException)

        return 0
        
        # '''
    def __check_robotParams(self):
        # This function cannot be called outside
        assert len(self.robotParams) == 35, "len(robotParams) != 35, {}".format(self.robotParams)
        # give some restriction here
        '''
        ### Axis (8, 9), (12, 13), (18, 19), (22, 23), 
        # we should use **a * b = 0 for each group. Which means, 
        # take (8, 9) for example. When axis 8 has value, we should make sure axis 9 is set to 0.**
        **a * b = 0**
        '''

        assert self.robotParams["x8"] * self.robotParams["x9"] == 0 , print("x8:", self.robotParams["x8"], "\nx9:", self.robotParams["x9"])
        assert self.robotParams["x12"] * self.robotParams["x13"] == 0, print("x12:", self.robotParams["x12"], "\nx13:", self.robotParams["x13"])
        assert self.robotParams["x18"] * self.robotParams["x19"] == 0, print("x18:", self.robotParams["x18"], "\nx19:", self.robotParams["x19"])
        assert self.robotParams["x22"] * self.robotParams["x23"] == 0, print("x22:", self.robotParams["x22"], "\nx23:", self.robotParams["x23"])
    def robotChecker(self):
        '''
            check the robot with a neutral -> smile -> neutral procedure
        '''
        assert self.connection == True, "Connection is bad" # make sure the connection is good
        
        self.return_to_stable_state() # Return to the stable state (標準Pose)

        self.switch_to_defaultPose(2) # Switch to default pose 2 笑顔
        self.connect_ros(isSmoothly=True, isRecording=False)     # connect server and send the command to change facial expression smoothly
        time.sleep(1)

        self.return_to_stable_state() # Return to the stable state (標準Pose)  
    def perform_openface(self, figure):
        # send figure to openface and get result
        # TODO: Do something with openface
        # Subprocess
        openfacePath = ""
        figurePath = ""
        return openfacePath

    def analysis(self, target_emotion_name):
        assert target_emotion_name in ["Anger", "Disgust", "Fear", "Happiness", "Sadness", "Surprise"], "You are not using the predefine name"
        py_feat_analysis(self.readablefileName, target_emotion_name)
    def feedback(self):
        pass
def list2string(data):
    strtmp = ""
    for val in data:
        strtmp += str(val) + ","
    retstr = strtmp.rstrip(',')
    return retstr
def basicRunningCell(robotObject, commandSet, isRecordingFlag=False, steps=20):
    
    rb = robotObject

    for k,v in commandSet.items():
        print("\n\n")
        # Return to Standard Pose
        rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
        rb.connect_ros(True, False)

        # Go to the facial expressions
        print("Switch to {}".format(k))
        rb.switch_to_customizedPose(v)
        rb.connect_ros(True, False, appendix="{}".format(k), steps=steps) # isSmoothly = True ,isRecording = True

    # Return to Standard Pose
    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)

def sigmoid(x):
    return 1 / (1 + math.exp(-x))

def get_target(emotion_name):
    # Anger, Disgust, Fear, Happiness, Sadness, Surprise, Neutral
    # or lowercase
    if emotion_name in ["Anger", "anger"]:
        return 0
    elif emotion_name in ["Disgust", "disgust"]:
        return 1
    elif emotion_name in ["Fear", "fear"]:
        return 2
    elif emotion_name in ["Happiness", "happiness"]:
        return 3
    elif emotion_name in ["Sadness", "sadness"]:
        return 4
    elif emotion_name in ["Surprise", "surprise"]:
        return 5
    elif emotion_name in ["Neutral", "neutral"]:
        return 6

def py_feat_analysis(img, target_emotion, is_save_csv=True):
    from feat import Detector
    '''
        @img: file name 
        @target_emotion: Anger, Disgust, Fear, Happiness, Sadness, Surprise
    '''
    global detector

    image_prediction = detector.detect_image(img)
    df = image_prediction.head()
    if FEAT_VERSION == 0:
        emo_df = df.iloc[-1:,-8:] # feat 0.3.7
    elif FEAT_VERSION == 1:
        emo_df = df.iloc[-1:,-9:-1] # feat 0.5.0
    else:
        raise Exception("FEAT_VERSION is not correct")

    if is_save_csv:
        csv_name = img[:-4]+".csv"
        csv_emotion_name = img[:-4]+"_emotion.csv"
        df.to_csv(csv_name)
        emo_df.to_csv(csv_emotion_name)
    targetID = get_target(target_emotion)
    if DEBUG > 0:
        print("[INFO]py_feat_analysis: {}".format(list(df[target_emotion])[0]))
    # return emo_df.iloc[0,targetID]
    return list(df[target_emotion])[0]

def checkParameters(robotParams):
    # Axis (8, 9), (12, 13), (18, 19), (22, 23), 
    # we should use a * b = 0 for each group. 
    # Which means, take (8, 9) for example. 
    # When axis 8 has value, we should make sure axis 9 is set to 0. 
    if robotParams[8-1] * robotParams[9-1] != 0:
        robotParams[np.random.choice([8-1, 9-1])] = 0
    # x12 = x8, # x13 = x9, no need to be different
    robotParams[12-1] = robotParams[8-1]
    robotParams[13-1] = robotParams[9-1]
    # if robotParams[12-1] * robotParams[13-1] != 0:
    #     robotParams[np.random.choice([12-1, 13-1])] = 0
    if robotParams[18-1] * robotParams[19-1] != 0:
        robotParams[np.random.choice([18-1, 19-1])] = 0

    # This part might cause left not equal to right
    # if robotParams[22-1] * robotParams[23-1] != 0:
    #     robotParams[np.random.choice([22-1, 23-1])] = 0
        
    # x22 = x18, x23 = x19
    robotParams[21] = robotParams[17]
    robotParams[22] = robotParams[18]


    assert robotParams[8-1] * robotParams[9-1] == 0
    assert robotParams[12-1] * robotParams[13-1] == 0
    assert robotParams[18-1] * robotParams[19-1] == 0
    assert robotParams[22-1] * robotParams[23-1] == 0
    return robotParams

def fix_robot_param(fixedrobotcode):
    # x2 = x1, use one axis for eyes upper lid
    # fixedrobotcode[0] = 0
    fixedrobotcode[1] = fixedrobotcode[0]
    # x7 = x6, use one axis for eyes lower lid
    fixedrobotcode[6] = fixedrobotcode[5]
    # x12 = x8
    fixedrobotcode[11] = fixedrobotcode[7]
    # x13 = x9
    fixedrobotcode[12] = fixedrobotcode[8]
    # x14  = x10
    fixedrobotcode[13] = fixedrobotcode[9]
    # x17 = x16
    fixedrobotcode[16] = fixedrobotcode[15]
    # x22 = x18
    fixedrobotcode[21] = fixedrobotcode[17]
    # x23 = x19
    fixedrobotcode[22] = fixedrobotcode[18]
    # x24 = x20
    fixedrobotcode[23] = fixedrobotcode[19]
    # To open all axes
    # x4 = x3
    fixedrobotcode[3] = fixedrobotcode[2]
    # x15 = x11
    fixedrobotcode[14] = fixedrobotcode[10]
    fixedrobotcode = checkParameters(fixedrobotcode)

    return fixedrobotcode

# BO
def target_function(**kwargs):
    """Pyfeat evaluation object
    
    Target
        Maxmize the result of Pyfeat
    """
    rb = kwargs['robot']
    target_emotion = kwargs['target_emotion']
    kwargs = kwargs["kwargs"]
    


    # Get robot parameters
    neutral = [86, 86, 128, 128, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, 122]
    if headYaw_fix_flag:
        # modify the headYaw if we need to fix it
        neutral[-1] = headYaw_fix
    fixedrobotcode = copy.copy(neutral)
    
    # dict = {}
    for k,v in kwargs.items():
        # print(k,v)
        if "x" in k:
            fixedrobotcode[int(k[1:])-1] = round(v)

    # Happy Constraints
    if fixedrobotcode[0] + fixedrobotcode[5] > 255 + 86 + 10:
        # if the upper lid and lower lid are too close to each other, we should return 0
        print("[INFO]Happy Constraints, give 0 score.")
        return 0

    # # x2 = x1, use one axis for eyes upper lid
    # # fixedrobotcode[0] = 0
    # fixedrobotcode[1] = fixedrobotcode[0]

    # # x7 = x6, use one axis for eyes lower lid
    # fixedrobotcode[6] = fixedrobotcode[5]

    # # x12 = x8
    # fixedrobotcode[11] = fixedrobotcode[7]

    # # x13 = x9
    # fixedrobotcode[12] = fixedrobotcode[8]

    # # x14  = x10
    # fixedrobotcode[13] = fixedrobotcode[9]


    # # x17 = x16
    # fixedrobotcode[16] = fixedrobotcode[15]

    # # x22 = x18
    # fixedrobotcode[21] = fixedrobotcode[17]

    # # x23 = x19
    # fixedrobotcode[22] = fixedrobotcode[18]

    # # x24 = x20
    # fixedrobotcode[23] = fixedrobotcode[19]

    # # To open all axes
    # # x4 = x3
    # fixedrobotcode[3] = fixedrobotcode[2]

    # # x15 = x11
    # fixedrobotcode[14] = fixedrobotcode[10]

    # ban x21 x25
    # ban x26 x27 
    # ban 31 34 33 35

    # check parameters
    # fixedrobotcode = checkParameters(fixedrobotcode)

    fixedrobotcode = fix_robot_param(fixedrobotcode)
    
    # control robot 
    if DEBUG > 0:
        print("[INFO]fixedrobotcode is", fixedrobotcode)
    rb.switch_to_customizedPose(fixedrobotcode)
    global MYSTEPS
    returncode = rb.connect_ros(isSmoothly=True, isRecording=False, steps=MYSTEPS) # isSmoothly = True ,isRecording = True
    time.sleep(0.5)
    # the sleep inside rb is not working for outside.

    # -------------
    # I need a feedback here!!
    # -------------
    if returncode == 0:
        print('[INFO]successfully return')
    

    
    output = 0
    global loopFlag
    global COUNTER
    # pyfeat_in_loop_output case
    if loopFlag == 0:
        # Take photo using cv2
        rb.take_picture_cv(isUsingCounter=False, appendix='{}_{}'.format(target_emotion, COUNTER), folder=target_emotion)
        
        # ------------------ deprecated ----------------
        # Take photo using ffmpeg (deprecated)
        # process = rb.take_picture(isUsingCounter=False, appendix='{}_{}'.format(target_emotion, COUNTER), folder=target_emotion)
        # process.wait()
        # if process.returncode != 0:
        #     print(process.stdout.readlines())
        #     raise Exception("The subprocess does NOT end.")
        # delete useless figure
        # folderPath = "image_analysis/{}/".format(target_emotion)
        # for i in os.listdir(folderPath):
        #     if '1.png' in i:
        #         os.remove(folderPath + i)
        # ------------------ deprecated ----------------
        COUNTER += 1
        
        # Py-feat Analysis
        print('[INFO]The {}th trial'.format(str(COUNTER + 1)))
        print('[INFO]target_emotion', target_emotion)

        # Use Py-Feat 0.3.7
        output = py_feat_analysis(img=rb.readablefileName, target_emotion=target_emotion)

        # Use SiameseRankNet
        # output = SiameseRankNet_analysis(img=rb.readablefileName, target_emotion=target_emotion)

        
        if CURBEST[1] < output:
            CURBEST[0] = rb.readablefileName
            CURBEST[1] = output
        print('[INFO]Current Best: {}, {}'.format(CURBEST[1], CURBEST[0]))
        # Save every parameters
        # construct the DataFrame
        df_dic = {}
        df_dic[target_emotion] = [output]
        for k,v in kwargs.items():
            df_dic[k] = [v]
        print('[INFO]df_dic', df_dic)
        df = pd.DataFrame(df_dic)
        df_name = rb.readablefileName[:-4]+"_axes_data.csv"

        df.to_csv(df_name, index=False, sep=',')

        # save robot param data
        robot_param = pd.DataFrame(rb.robotParams, index=[0])
        robot_param_name = rb.readablefileName[:-4]+"_rb_paramdata.csv"
        robot_param.to_csv(robot_param_name, index=False, sep=',')


    
    # Human optimization output case
    elif loopFlag == 1:
        # type 1, 1-7 rating
        # Human rating part

        instructionSentence = '\n\nThe {}/100 trials for {}.\nPlease watch the robot face directly and type an integer number within 1-7 (1 is Lowest, 7 is Largest) and Enter:\n'.format(COUNTER + 1, target_emotion)
        errorSentence = 'Input ERROR. Please try again.\nType an integer number within 1-7 (1 is Lowest, 7 is Largest) and Enter:\n'
        rating = input(instructionSentence)
        while True:
            if rating.isdigit():
                if int(rating) in range(1, 8):
                    output = (int(rating) - 1) / 6.0
                    break
                else:
                    rating = input(errorSentence)
            else:
                rating = input(errorSentence)
        
        # Take photo using cv2
        rb.take_picture_cv(isUsingCounter=False, appendix='{}_{}'.format(target_emotion, COUNTER), folder=target_emotion)
        
        # ------------------ deprecated ----------------
        # Take photo using ffmpeg (deprecated)
        # process = rb.take_picture(isUsingCounter=False, appendix='{}_{}'.format(target_emotion, COUNTER), folder=target_emotion)
        # process.wait()
        # if process.returncode != 0:
        #     print(process.stdout.readlines())
        #     raise Exception("The subprocess does NOT end.")
        # delete useless figure
        # folderPath = "image_analysis/{}/".format(target_emotion)
        # for i in os.listdir(folderPath):
        #     if '1.png' in i:
        #         os.remove(folderPath + i)
        # ------------------ deprecated ----------------

        COUNTER += 1

        # Save every parameters
        # construct the DataFrame
        df_dic = {}
        df_dic[target_emotion] = [output]
        for k,v in kwargs.items():
            df_dic[k] = [v]
        
        print('df_dic', df_dic)
        df = pd.DataFrame(df_dic)
        df_name = rb.readablefileName[:-4]+"_axes_data.csv"

        df.to_csv(df_name, index=False, sep=',')

        # type 2, true or false rating
        # PROBLEMATIC!!! What's the matric here, it's too hard to define the matric for running Bayesian Optimization. Otherwise just use random search.

        # global totalTrails
        # import matplotlib.pyplot as plt
        # img = plt.imread(rb.bestImg)
        # plt.imshow(img)
        # plt.pause(1)
        # instructionSentence = 'Choose Q or P.\n' + \
        #     'If you think the figure on the left shows better facial expressions than the right one, input [Q] or [q] and press Enter.\n' + \
        #     'Otherwise, input [P] or [p]and press Enter' 
        # rating = input(instructionSentence)
        # while rating.lower() not in ['q', 'p']:
        #     print('Input ERROR, please try again.\n')
        #     rating = input(instructionSentence)
        # plt.close()
        # if rating.lower() == 'p':
        #     newimg = rb.readablefileName
        #     rb.bestImg = newimg


    # --- TEST return to normal state every time---
    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False, steps=3)
    # --- TEST END ---  

    return output


def bayesian_optimization(baseline, target_emotion, robot):

    def middle_function(**kwargs):
        return target_function(robot=robot, target_emotion=target_emotion, kwargs=kwargs)

    def generate_pbounds(axes, target_emotion):
        pbounds_dic = {}
        for i in axes:
            if target_emotion == 'happiness' and i in [0, 1]:
                # TODO need to verify
                happy_upper = 120
                pbounds_dic['x{}'.format(i)] = (0, happy_upper)
            # elif i in [0, 1]:
            #     pbounds_dic['x{}'.format(i)] = (0, 160)
            # else:
            pbounds_dic['x{}'.format(i)] = (0, 255)

        return pbounds_dic

    # x2 = x1, use one axis for eyes upper lid
    # x7 = x6, use one axis for eyes lower lid
    # x13 = x9
    # x17 = x16
    # x22 = x18
    # x14  = x10
    # x23 = x19
    # x12 = x8
    # x24 = x20

    
    axes_for_emotions = [
        [1, 6, 11,15], # anger [1, 2, 6, 7, 11, 15]
        [30], # disgust [19, 23, 30]
        [10, 8, 11, 15, 1, 20], # fear [1, 2, 6, 7, 8, 10, 11, 12, 14, 15, 19, 23, 32]
        [6, 9, 16, 18], # happyness [6, 7, 9, 13, 16, 17, 18, 22]
        [10, 11, 15, 19], # sadness [10, 14, 11, 15, 19, 23]
        [10, 8, 1] # surprise [10, 14, 8, 12, 1, 2]
    ]
    # ban x3, x4, x5, x21 x25 x26 x27 31 33 35 
    # This one open 25 axes, but DOF = 15
    # all_axes_for_emotions = [1, 6, 8, 9, 10, 11, 16, 18, 19, 20, 28, 29, 30, 32, 34]
    # without head pitch
    all_axes_for_emotions = [1, 6, 8, 9, 10, 11, 16, 18, 19, 20, 28, 29, 30, 32]

    # Ekman FACS
    # code = get_target(target_emotion)
    # pbounds = generate_pbounds(axes_for_emotions[code])
    
    # open all axes
    pbounds = generate_pbounds(all_axes_for_emotions, target_emotion)


    # TODO SequentialDomainReductionTransformer
    # reference : https://colab.research.google.com/drive/1tfbLZRLZgUadM5jz4trFudzta58849JV#scrollTo=J7Lp_Q6cQcP5

    # from bayes_opt import SequentialDomainReductionTransformer
    # bounds_transformer = SequentialDomainReductionTransformer()
    # mutating_optimizer = BayesianOptimization(
    #     f=middle_function,
    #     pbounds=pbounds,
    #     verbose=2,
    #     random_state=1,
    #     bounds_transformer=bounds_transformer
    # )

    # print(pbounds)
    optimizer = BayesianOptimization(
        f=middle_function,
        # Define HyperParameter Space

        pbounds = pbounds,
        # -----------------------------------
        # # Anger 
        # pbounds={
        #     # "x1": (0, 255), 
        #     # "x2": (0, 255),
        #     # "x3": (0, 255), 
        #     # "x4": (0, 255), 
        #     # "x5": (0, 255), 
        #     "x6": (0, 255), 
        #     # "x7": (0, 255), 
        #     # "x8": (0, 255), 
        #     # "x9": (0, 255), 
        #     # "x10": (0, 255),
        #     "x11": (0, 255), 
        #     # "x12": (0, 255), 
        #     # "x13": (0, 255), 
        #     # "x14": (0, 255), 
        #     "x15": (0, 255), 
        #     # "x16": (0, 255), 
        #     # "x17": (0, 255), 
        #     # "x18": (0, 255), 
        #     # "x19": (0, 255), 
        #     # "x20": (0, 255), 
        #     # "x21": (0, 255), 
        #     # "x22": (0, 255), 
        #     # "x23": (0, 255), 
        #     # "x24": (0, 255), "x25": (0, 255), 
        #     # "x26": (0, 255), "x27": (0, 255), 
        #     # "x28": (0, 255), "x29": (0, 255), # hot
        #     # "x30": (0, 255), 
        #     # "x31": (0, 255), 
        #     # "x32": (0, 200),  #hot
        #     # "x33": (0, 255), "x34": (0, 255), "x35": (0, 255)
        # },
        

        random_state=1,
        verbose=2)
    # random_state is like set seed
    # verbose = 1 prints only when a maximum is observed, verbose = 0 is silent
    
    # initialization of pre-define facial expression
    neutral_baseline = defaultPose.prototypeFacialExpressions["neutral"]
    subtract = abs(np.array(neutral_baseline) - np.array(baseline)) 
    if subtract.sum() == 0:
        subtract += 1
    if list(subtract) != []:
        # print('subtract', subtract)
        probe_param = {}
        for i in range(len(subtract)):
            if subtract[i] != 0:
                probe_param["x{}".format(i+1)] = subtract[i]
        # print(probe_param)

    # logger
    logger = JSONLogger(path="./image_analysis/"+ target_emotion + "/logs.json")
    optimizer.subscribe(Events.OPTIMIZATION_STEP, logger)

    # ------------------- INFO  ----------------------------
    # x2 = x1, use one axis for eyes upper lid
    # x7 = x6, use one axis for eyes lower lid
    # x13 = x9
    # x17 = x16
    # x22 = x18
    # x14  = x10
    # x23 = x19
    # x12 = x8
    # x24 = x20
    # initialization
    # bug!! Lazy make the expression become the expression of next folder. 
    # The NEXT time I called optimizer.max will process. Which means it will skip this time.
    
    # --------------------------------------
    # NO initialization
    # if target_emotion == 'anger':
    #     # anger
    #     optimizer.probe(params={'x1':0 , 'x6': 255, 'x11': 255, 'x15': 255}, lazy=True,)

    # if target_emotion == 'disgust':
    #     # disgust
    #     optimizer.probe(params={'x30': 255}, lazy=True,)

    # if target_emotion == 'fear':
    #     # fear
    #     optimizer.probe(params={'x1': 0, 'x8': 255, 'x10': 255, 'x11': 255, 'x15': 255, 'x20': 255}, lazy=True,)

    # if target_emotion == 'happiness':
    #     # happiness
    #     optimizer.probe(params={'x6': 255, 'x9': 255, 'x16': 255, 'x18': 255,}, lazy=True,)

    # if target_emotion == 'sadness':
    #     # sadness
    #     optimizer.probe(params={'x10': 255, 'x11': 255, 'x15': 255, 'x19': 255}, lazy=True,)

    # if target_emotion == 'surprise':
    #     # surprise
    #     optimizer.probe(params={'x1': 0, 'x8': 255, 'x10': 128,}, lazy=True,)
    # --------------------------------------

    # Happiness prototype
    optimizer.probe(
        params={'x1': 86, 'x6': 255, 'x8': 0, 'x9': 255, 'x10': 0, 'x11': 0, 'x16': 255, 'x18': 255, 'x19': 0, 'x20': 0, 'x28': 255, 'x29': 255, 'x30': 0, 'x32': 0},
        lazy=False,
    )

    # Anger
    # optimizer.probe(
    #     params={"x1": 0, "x6": 255, "x11": 255, "x15": 255},
    #     lazy=False,
    # )
    global init_points
    global n_iter
    optimizer.maximize(init_points=init_points, n_iter=n_iter)
    # optimizer.maximize(alpha=1e-2) 
    # alpha is interpreted as the variance of additional Gaussian measurement noise 
    # on the training observations.

    print(target_emotion, "Max result:", optimizer.max)
    return optimizer

def check_folder(folderName):
    folderPath = "image_analysis/{}/".format(folderName)
    if not os.path.exists(folderPath):
        try:
            os.mkdir(folderPath)
        except Exception as e:
            print(e)

def recover_param_from_csv(csv_name, steps=15):
    df = pd.read_csv(csv_name)

    neutral = [86, 86, 128, 128, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, 95]
    fixedrobotcode = copy.copy(neutral)
    # dict = {}
    for k,v in df.to_dict().items():
    #     print('k,v', k,v)
        if "x" in k:
            fixedrobotcode[int(k[1:])-1] = round(v[0])
    
    fixedrobotcode = fix_robot_param(fixedrobotcode)

    # control robot
    rb.switch_to_customizedPose(fixedrobotcode)
    global smoothSleepTime
    smoothSleepTime = 0.02
    returncode = rb.connect_ros(isSmoothly=True, isRecording=False, steps=steps) # isSmoothly = True ,isRecording = True
    time.sleep(0.5)
    


# main
def main():
    global rb
    rb = robot(duration=2)
    assert rb.connection == True
    global COUNTER
    COUNTER = 0
    global init_points
    global n_iter
    global MYSTEPS
    # init_points = 2
    # n_iter = 2
    init_points = 10
    n_iter = 90

    # set headYaw_fix_flag
    global headYaw_fix_flag
    headYaw_fix_flag = True
    # set how much steps Nikola need to shift the axes from one to another
    MYSTEPS = 15
    # change workdir
    workdir = "/home/dongagent/github/CameraControl/ros_dongagent_ws/src/dongagent_package/scripts"
    os.chdir(workdir)
    rb.bestImg = workdir + '/image_analysis/temp/neutral.png'
    assert os.getcwd() == workdir, print(os.getcwd())
    
    global detector
    # landmark_model should be set to mobilefacenet in case you want to use pyfeat 0.3.7/0.5.0/0.6.1
    # Be care of FEAT_VERSION
    # detector = Detector(emotion_model = "resmasknet", landmark_model='mobilefacenet')
    
    global smoothSleepTime
    # res = subprocess.Popen("ls", cwd="/home/dongagent/github/CameraControl/ros_dongagent_ws/src/dongagent_package/scripts")
    # print(res)

    # assert res == 0, 'workdir err'
    # rb.transfer_robotParams_to_states(rb.lastParams, [x for x in range(36)])
    # Anger, Disgust, Fear, Happiness, Sadness, Surprise
    # anger, disgust, fear, happiness, sadness, surprise
    # defaultPose.prototypeFacialExpressions

    # Exp 19: test the head movement
    # collect data again confirming details
    for k,v in defaultPose.prototypeFacialExpressions.items():
        # if k != 'surprise':
        #     continue
        if k == 'neutral':
            continue
        rb.start_taking_video(appendix=k)
        time.sleep(1)

        
        smoothSleepTime = 0.02
        rb.switch_to_customizedPose(v)
        rb.connect_ros(True, False, steps=25, isUsingSigmoid=True, sigmoid_factor=7) # isSmoothly = True ,isRecording = True

        time.sleep(2)
        rb.stop_taking_video()
        
        rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
        rb.connect_ros(True, False)
        time.sleep(1)

    for k,v in defaultPose.hotExpressions.items():
        # if k != 'hotSurprise':
        #     continue
        if k == 'neutral':
            continue
        rb.start_taking_video(appendix=k)
        time.sleep(1)

        
        smoothSleepTime = 0.02
        rb.switch_to_customizedPose(v)
        rb.connect_ros(True, False, steps=25, isUsingSigmoid=True, sigmoid_factor=7) # isSmoothly = True ,isRecording = True

        time.sleep(2)
        rb.stop_taking_video()
        
        rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
        rb.connect_ros(True, False)
        time.sleep(1)

    
    # Exp 18: new environment basic video, sigmoid function
    # 
    # # record prototype video and prototype with mouth opening
    
    # for k,v in defaultPose.prototypeFacialExpressions.items():
    #     if k == 'neutral':
    #         continue
    #     rb.start_taking_video(appendix=k)
    #     time.sleep(1)

        
    #     smoothSleepTime = 0.04
    #     rb.switch_to_customizedPose(v)
    #     rb.connect_ros(True, False, steps=25, isUsingSigmoid=True) # isSmoothly = True ,isRecording = True

    #     time.sleep(2)
    #     rb.stop_taking_video()
        
    #     rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #     rb.connect_ros(True, False)
    #     time.sleep(1)
    
    # for k,v in defaultPose.hotExpressions.items():
    #     if k == 'neutral':
    #         continue
    #     rb.start_taking_video(appendix=k)
    #     time.sleep(1)

    #     # global smoothSleepTime
    #     smoothSleepTime = 0.03
    #     rb.switch_to_customizedPose(v)
    #     rb.connect_ros(True, False, steps=20, isUsingSigmoid=True) # isSmoothly = True ,isRecording = True

    #     time.sleep(2)
    #     rb.stop_taking_video()
        
    #     rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #     rb.connect_ros(True, False)
    #     time.sleep(1) 


    # Exp 17: record video, record prototype images
    # fl = os.listdir("image_analysis/231225Exp17NewFeat/top50anger")
    # fl = sorted(fl, key=lambda x: int(x.split('_')[0]))
    # for file in fl:
    #     # print(file)
    #     if file.endswith(".csv"):
    #         print(file)
    #         rb.start_taking_video(appendix=file[:-14])
    #         time.sleep(1)
    #         recover_param_from_csv("image_analysis/231225Exp17NewFeat/top50anger/" + file, 25)
    #         time.sleep(2)
    #         rb.stop_taking_video()
            
    #         rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #         rb.connect_ros(True, False)
    #         time.sleep(1)


    # # record prototype video and prototype with mouth opening
    # global smoothSleepTime
    # for k,v in defaultPose.prototypeFacialExpressions.items():
    #     if k == 'neutral':
    #         continue
    #     rb.start_taking_video(appendix=k)
    #     time.sleep(1)

        
    #     smoothSleepTime = 0.02
    #     rb.switch_to_customizedPose(v)
    #     rb.connect_ros(True, False, steps=25) # isSmoothly = True ,isRecording = True

    #     time.sleep(2)
    #     rb.stop_taking_video()
        
    #     rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #     rb.connect_ros(True, False)
    #     time.sleep(1)
    
    
    # for k,v in defaultPose.hotExpressions.items():
    #     if k == 'neutral':
    #         continue
    #     rb.start_taking_video(appendix=k)
    #     time.sleep(1)

    #     # global smoothSleepTime
    #     smoothSleepTime = 0.02
    #     rb.switch_to_customizedPose(v)
    #     rb.connect_ros(True, False, steps=25) # isSmoothly = True ,isRecording = True

    #     time.sleep(2)
    #     rb.stop_taking_video()
        
    #     rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #     rb.connect_ros(True, False)
    #     time.sleep(1)
    
    # record prototype with mouth opening video
    

            # break
    # ---------------------
    # prototype
    # ---------------------
    # for k,v in defaultPose.prototypeFacialExpressions.items():
    #     if k == 'neutral':
    #         continue
    #     print("switch to: ", k)
    #     # print(v)
    #     rb.switch_to_customizedPose(v)
    #     rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
    #     rb.take_picture_cv(isUsingCounter=False, appendix='{}_{}'.format(k, 'test'), folder=k)
    #     time.sleep(3)
    #     test = py_feat_analysis(rb.fileName, k)
    #     print("py_feat_analysis result is: ", test)
        
    #     # save robot param data
    #     robot_param = pd.DataFrame(rb.robotParams, index=[0])
    #     print(robot_param)
    #     robot_param_name = rb.readablefileName[:-4]+"_rb_paramdata.csv"
    #     robot_param.to_csv(robot_param_name, index=False, sep=',')

    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)

    # ------------------------------------
    # Prototype with mouth opening
    # ------------------------------------
    # for k,v in defaultPose.hotExpressions.items():
    #     if k == 'neutral':
    #         continue
    #     print("switch to: ", k)
    #     # print(v)
    #     rb.switch_to_customizedPose(v)
    #     rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
    #     rb.take_picture_cv(isUsingCounter=False, appendix='{}_{}'.format(k, 'test'), folder=k)
    #     time.sleep(3)
    # print("py_feat_analysis result is: ", py_feat_analysis(rb.fileName, k[3:].lower()))

    #     # save robot param data
    #     robot_param = pd.DataFrame({k:[v] for k, v in rb.robotParams.items()})
    #     robot_param_name = rb.readablefileName[:-4]+"_rb_paramdata.csv"
    #     robot_param.to_csv(robot_param_name, index=False, sep=',')

    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)
    


    # Exp 16: Use Py-Feat 0.6.1 for Nikola, with happy eye restriction
    # for target_emotion in ['happiness']:
    #     check_folder(target_emotion)
    #     COUNTER = 0
    #     print(target_emotion)
    #     global CURBEST
    #     CURBEST = ['', 0]
    #     # test photo
    #     rb.take_picture_cv(isUsingCounter=False, appendix='{}_{}'.format(target_emotion, 'test'), folder=target_emotion)
    #     optimizer = bayesian_optimization(
    #         baseline=defaultPose.prototypeFacialExpressions[target_emotion],
    #         target_emotion=target_emotion,
    #         robot=rb)
    #     print('\n')
    #     print("Current target emotion is: ", target_emotion, optimizer.res)
    #     print('\n')
    #     print(f"Best {target_emotion} is: {CURBEST[1]}, {CURBEST[0]}")
    #     print('\n')
    #     # Return to normal
    #     rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #     rb.connect_ros(True, False)

    # Exp 15: Use Py-Feat 0.6.1 for Nikola
    # for target_emotion in ['sadness', 'surprise', 'disgust', 'fear']:
    # for target_emotion in ['anger']:
    #     check_folder(target_emotion)
    #     COUNTER = 0
    #     print(target_emotion)
    #     global CURBEST
    #     CURBEST = ['', 0]
    #     # test photo
    #     rb.take_picture_cv(isUsingCounter=False, appendix='{}_{}'.format(target_emotion, 'test'), folder=target_emotion)
    #     optimizer = bayesian_optimization(
    #         baseline=defaultPose.prototypeFacialExpressions[target_emotion],
    #         target_emotion=target_emotion,
    #         robot=rb)
    #     print('\n')
    #     print("Current target emotion is: ", target_emotion, optimizer.res)
    #     print('\n')
    #     print(f"Best {target_emotion} is: {CURBEST[1]}, {CURBEST[0]}")
    #     print('\n')
    #     # Return to normal
    #     rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #     rb.connect_ros(True, False)




    # Exp 14: Use SiameseRanknet model for Nikola
    # Setup: lighting system, webcam, tripot,
    # Contents: BORFEO 100 trials, Prototype, Prototype with mouth opening

    # ---------------------
    # exp 14-1 prototype
    # ---------------------
    # for k,v in defaultPose.prototypeFacialExpressions.items():
    #     print("switch to: ", k)
    #     # print(v)
    #     rb.switch_to_customizedPose(v)
    #     rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
    #     rb.take_picture_cv(isUsingCounter=False, appendix='{}_{}'.format(k, 'test'), folder=k)
    #     time.sleep(3)
    #     print("py_feat_analysis result is: ", py_feat_analysis(rb.fileName, k))
    #     print("SiameseRankNet result is: ", SiameseRankNet_analysis(rb.fileName, k))

    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)

    # ------------------------------------
    # exp 14-2 prototype with mouth opening
    # ------------------------------------
    # for k,v in defaultPose.hotExpressions.items():
    #     print("switch to: ", k)
    #     # print(v)
    #     rb.switch_to_customizedPose(v)
    #     rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
    #     rb.take_picture_cv(isUsingCounter=False, appendix='{}_{}'.format(k, 'test'), folder=k)
    #     time.sleep(3)
    #     # print("py_feat_analysis result is: ", py_feat_analysis(rb.fileName, k))
    #     print("SiameseRankNet result is: ", SiameseRankNet_analysis(rb.fileName, k))

    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)

    # -------------------------------------
    # exp 14-3 BORFEO using SiameseRankNet
    # -------------------------------------

    # for target_emotion in ['anger']:
    #     check_folder(target_emotion)
    #     COUNTER = 0
    #     print(target_emotion)
    #     global CURBEST
    #     CURBEST = ['', 0]
    #     # test photo
    #     rb.take_picture_cv(isUsingCounter=False, appendix='{}_{}'.format(target_emotion, 'test'), folder=target_emotion)
    #     optimizer = bayesian_optimization(
    #         baseline=defaultPose.prototypeFacialExpressions[target_emotion],
    #         target_emotion=target_emotion,
    #         robot=rb)
    #     print('\n')
    #     print("Current target emotion is: ", target_emotion, optimizer.res)
    #     print('\n')
    #     # Return to normal
    #     rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #     rb.connect_ros(True, False)

    # # Return to normal
    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)
    # time.sleep(2)
    # rb.webcam_stream_widget.stop()
    # try:
    #     rb.webcam_stream_widget.vthread.join()
    # except Exception as e:
    #     print(e)




    
    # Exp 11 & 12: Use new threading for cv2 and do BORFEO for 500 trials
    # Exp 13: Use new lighting system and do BORFEO for 500 trials
    # Finished: anger, happiness, fear, disgust
    # sadness, surprise, neutral
    # for target_emotion in ['anger', 'disgust', 'fear', 'happiness', 'sadness', 'surprise', 'neutral']:
    # for target_emotion in ['anger', 'disgust', 'fear', 'happiness', 'sadness', 'surprise']:
    '''
    for target_emotion in ['anger']:
        check_folder(target_emotion)
        COUNTER = 0
        print(target_emotion)
        global CURBEST
        CURBEST = ['', 0]
        # test photo
        rb.take_picture_cv(isUsingCounter=False, appendix='{}_{}'.format(target_emotion, 'test'), folder=target_emotion)
        optimizer = bayesian_optimization(
            baseline=defaultPose.prototypeFacialExpressions[target_emotion],
            target_emotion=target_emotion,
            robot=rb)
        print('\n')
        print("Current target emotion is: ", target_emotion, optimizer.res)
        print('\n')
        # Return to normal
        rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
        rb.connect_ros(True, False)

    # Return to normal
    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)
    time.sleep(2)
    rb.webcam_stream_widget.stop()
    try:
        rb.webcam_stream_widget.vthread.join()
    except Exception as e:
        print(e)
    '''

    # Exp 10 end with bug


    # TODO
    # Exp 9: Psycho-physical optimization 100 trials Saori San 

    '''
    # 100 trials
    beginning_Instruction = "\n---------------INSTRUCTIONS--------------\nIn this experiment, you are required to watch the robot directly AFTER it changing the facial expressions. Then you need to rate the facial expressions from 1 to 7. 1 means you didn't feel the facial expression at all, and 7 means you think the robot represent the best facial expressions. \nPress Enter to continue."
 
    input(beginning_Instruction)

    # for target_emotion in ['anger']:
    for target_emotion in ['disgust', 'fear', 'happiness', 'sadness', 'surprise', 'neutral']:
    # for target_emotion in []:
    # for target_emotion in []:
    # for target_emotion in ['neutral']:
        check_folder(target_emotion)
        COUNTER = 0
        print(target_emotion)
        # target_emotion = "anger"
        optimizer = bayesian_optimization(
            baseline=defaultPose.prototypeFacialExpressions[target_emotion],
            target_emotion=target_emotion,
            robot=rb)
        print('\n')
        print("Current target emotion is: ", target_emotion, optimizer.res)
        print('\n')
        # Return to normal
        rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
        rb.connect_ros(True, False)
        # time.sleep(1)
    # # Return to normal
    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)
    '''

    # 2022.1.7
    # Exp 8: Namba Coding, Namba coding hot
    #       Procedure: Switch FE -> Take photo -> py-feat evaluation.

    # check_folder('prototypeFacialExpressions')
    # check_folder('hotFacialExpressions')
    # # ----------------------------
    # # prototypeFacialExpressions
    # # ----------------------------

    # global cap
    # cap = cv2.VideoCapture(2)
    # # open camera

    # for k,v in defaultPose.prototypeFacialExpressions.items():
    #     folderName = 'prototypeFacialExpressions'
    #     print("switch to: ",k)
    #     rb.switch_to_customizedPose(v)
    #     rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
    #     # Take photo
    #     # process = rb.take_picture(isUsingCounter=False, appendix='{}'.format(k), folder=folderName)
    #     # process.wait()
    #     # if process.returncode != 0:
    #     #     print(process.stdout.readlines())
    #     #     raise Exception("The subprocess does NOT end.")

    #     # Take cv photo
    #     rb.take_picture_cv(isUsingCounter=False, appendix='{}'.format(k), folder=folderName)

    #     # # delete useless figure
    #     # folderPath = "image_analysis/{}/".format(folderName)
    #     # for i in os.listdir(folderPath):
    #     #     if '1.png' in i:
    #     #         os.remove(folderPath + i)


    #     time.sleep(3)
    #     print("py_feat_analysis result is: ", py_feat_analysis(rb.readablefileName, k))

    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)

    # --------------
    # hotExpressions
    # --------------
    # for k,v in defaultPose.hotExpressions.items():
    #     print("switch to: ", k)
    #     folderName = 'hotFacialExpressions'
    #     targetEmotion = k[3:]
    #     print(targetEmotion)

    #     rb.switch_to_customizedPose(v)
    #     rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
    #     # Take photo
    #     process = rb.take_picture(isUsingCounter=False, appendix='{}'.format(targetEmotion), folder=folderName)
    #     process.wait()
    #     if process.returncode != 0:
    #         print(process.stdout.readlines())
    #         raise Exception("The subprocess does NOT end.")

    #     # delete useless figure
    #     folderPath = "image_analysis/{}/".format(folderName)
    #     for i in os.listdir(folderPath):
    #         if '1.png' in i:
    #             os.remove(folderPath + i)

    #     # Py-feat Analysis
    #     temp_pyfeat_result = py_feat_analysis(img=rb.readablefileName,  target_emotion = targetEmotion)

    #     # Save every parameters
    #     # construct the DataFrame
    #     df_dic = {}
    #     df_dic[k] = [temp_pyfeat_result]
    #     print(df_dic)
    #     # for k,v in kwargs.items():
    #     #     df_dic[k] = [v]
    #     # print(df_dic)
    #     # df = pd.DataFrame(df_dic)
    #     # df_name = rb.readablefileName[:-4]+"_axes_data.csv"

    #     # df.to_csv(df_name, index=False, sep=',')


    #     time.sleep(3)

    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)
    


    # 2021.12.23
    # Exp 6: Restrict search range of downward eye lid to avoid noise. # Seems OK!
    # for target_emotion in ['anger']:
    # Exp 7: All FEs: anger, disgust, fear, happiness, sadness, surprise, neutral
    
    # for target_emotion in ['anger']:
    # for target_emotion in ['disgust', 'fear']:
    # for target_emotion in ['happiness', 'sadness']:
    # for target_emotion in ['surprise', 'neutral']:
    # for target_emotion in ['neutral']:
    #     check_folder(target_emotion)
    #     COUNTER = 0
    #     print(target_emotion)
    #     # target_emotion = "anger"
    #     optimizer = bayesian_optimization(
    #         baseline=defaultPose.prototypeFacialExpressions[target_emotion], 
    #         target_emotion=target_emotion,
    #         robot=rb)
    #     print('\n')
    #     print("Current target emotion is: ", target_emotion, optimizer.res)
    #     print('\n')
    #     # Return to normal
    #     rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #     rb.connect_ros(True, False)
    #     # time.sleep(1)
    # # Return to normal
    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)


    # 2021.12.22
    # Exp 4: Open all axes except eye gaze, increase head pitch # 30
    # Exp 5: Open all axes except eye gaze, without head pitch # 30, 100 anger, 100 disgust
    # # for target_emotion in ['anger']:
    # for target_emotion in ['disgust']:
    #     check_folder(target_emotion)
    #     COUNTER = 0
    #     print(target_emotion)
    #     # target_emotion = "anger"
    #     optimizer = bayesian_optimization(
    #         baseline=defaultPose.prototypeFacialExpressions[target_emotion], 
    #         target_emotion=target_emotion,
    #         robot=rb)
    #     print('\n')
    #     print("Current target emotion is: ", target_emotion, optimizer.res)
    #     print('\n')
    #     # Return to normal
    #     rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #     rb.connect_ros(True, False)
    #     # time.sleep(1)
    # # Return to normal
    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)


    # 2021.12.16
    # Exp 3: Open all axes
    
    # for target_emotion in ['anger']:
    # # for target_emotion in 'anger, disgust, fear, happiness, sadness'.split(', '):
    # # for target_emotion in 'anger, disgust, fear, happiness, sadness, surprise'.split(', '):
    #     check_folder(target_emotion)
    #     COUNTER = 0
    #     print(target_emotion)
    #     # target_emotion = "anger"
    #     optimizer = bayesian_optimization(
    #         baseline=defaultPose.prototypeFacialExpressions[target_emotion], 
    #         target_emotion=target_emotion,
    #         robot=rb)
    #     print('\n')
    #     print("Current target emotion is: ", target_emotion, optimizer.res)
    #     print('\n')
    #     # Return to normal
    #     rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #     rb.connect_ros(True, False)
    #     # time.sleep(1)
    # # Return to normal
    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)

    # 2021.12.09
    # Exp 2: All emotions without initialization

    # for target_emotion in ['anger']:
    # # for target_emotion in 'anger, disgust, fear, happiness, sadness'.split(', '):
    # # for target_emotion in 'anger, disgust, fear, happiness, sadness, surprise'.split(', '):
    #     check_folder(target_emotion)
    #     COUNTER = 0
    #     print(target_emotion)
    #     # target_emotion = "anger"
    #     optimizer = bayesian_optimization(
    #         baseline=defaultPose.prototypeFacialExpressions[target_emotion], 
    #         target_emotion=target_emotion,
    #         robot=rb)
    #     print('\n')
    #     print("Current target emotion is: ", target_emotion, optimizer.res)
    #     print('\n')
    #     # Return to normal
    #     rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    #     rb.connect_ros(True, False)
    #     # time.sleep(1)
    # # Return to normal
    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)


    # 2021.11.26
    # Exp 1: Happiness without initialization

    '''
    for target_emotion in ['surprise']:
    # for target_emotion in 'anger, disgust, fear, happiness, sadness, surprise'.split(', '):
        COUNTER = 0
        print(target_emotion)
        # target_emotion = "anger"
        optimizer = bayesian_optimization(
            baseline=defaultPose.prototypeFacialExpressions[target_emotion], 
            target_emotion=target_emotion,
            robot=rb)
        print('\n')
        print("Current target emotion is: ", target_emotion, optimizer.res)
        print('\n')
        # Return to normal
        rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
        rb.connect_ros(True, False)
        time.sleep(1) # Super important
    # Return to normal
    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)
    '''

    # -----------------------------------
    # 2021.11.22
    # Finished BOA
    # Happiness Experiment
    # print('\n{}\n'.format("happiness"))
    # target_emotion = "happiness"

    # optimizer = bayesian_optimization(
    #     baseline=defaultPose.prototypeFacialExpressions[target_emotion], 
    #     target_emotion=target_emotion,
    #     robot=rb)
    # print(optimizer.res)
    # # Return to normal
    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)



    # Anger Experiment
    # print('\n{}\n'.format("anger"))
    # target_emotion = "anger"
    # global COUNTER
    # COUNTER = 0
    # optimizer = bayesian_optimization(
    #     baseline=defaultPose.prototypeFacialExpressions[target_emotion], 
    #     target_emotion=target_emotion,
    #     robot=rb)
    # print(optimizer.res)
    # # Return to normal
    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)

    # ------------------------
    # 2021.11.20
    # Switch post -> Take photo -> py-feat evaluation.
    '''
    print('\n{}\n'.format("happiness"))
    for k,v in defaultPose.prototypeFacialExpressions.items():
        print("switch to: ",k)
        # print(v)
        rb.switch_to_customizedPose(v)
        rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
        rb.take_picture(isUsingCounter=False, appendix=k+'_test')
        time.sleep(3)
        print("py_feat_analysis result is: ", py_feat_analysis(rb.fileName, 'Surprise'))

    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)

    '''

    '''
    # 2021.10.11

    print('\n{}\n'.format("lookDown"))
    lD = defaultPose.experiment1['lookDown']

    rb.switch_to_customizedPose(lD)
    rb.connect_socket(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
    time.sleep(3)

    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_socket(True, False)


    # 2021.06.30
    
    lD = defaultPose.experiment1['lookDown']
    cE = defaultPose.experiment1['closeEye']
    neuExp = defaultPose.experiment1['neutral']
    angExp = defaultPose.experiment1['anger']
    hapExp = defaultPose.experiment1['happiness']

    print(len(angExp))


    # 6 trials
    # subtle
    # middle
    # strong

    title = ["subtle1", "middle1", "strong1", "subtle2", "middle2", "strong2"]

    counter = 0
    

    for j in range(2):
        for i in [angExp, hapExp, angExp]:
            
            # look Down and close eyes
            rb.switch_to_customizedPose(lD)
            rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
            time.sleep(3)

            # look ahead but still close eyes
            rb.switch_to_customizedPose(cE)
            rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
            time.sleep(1)

            rb.switch_to_customizedPose(neuExp)
            rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
            time.sleep(1)

            rb.switch_to_customizedPose(i)
            rb.connect_ros(isSmoothly=True, isRecording=False, appendix="{}".format(title[counter] + "_1")) # isSmoothly = True ,isRecording = True

            rb.switch_to_customizedPose(neuExp)
            rb.connect_ros(isSmoothly=True, isRecording=False, appendix="{}".format(title[counter] + "_2")) # isSmoothly = True ,isRecording = True
            time.sleep(1)

            counter += 1
            
    rb.switch_to_customizedPose(lD)
    rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
    time.sleep(3)

    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)
    
    '''
    # 2021.06.23

    # LookDown first
    # Real trials
    
    '''
    lD = defaultPose.experiment1['lookDown']
    cE = defaultPose.experiment1['closeEye']

    for j in range(2):
        for i in ['anger', 'happiness']:
            # look Down and close eyes
            rb.switch_to_customizedPose(lD)
            rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
            time.sleep(3)

            # look ahead but still close eyes
            rb.switch_to_customizedPose(cE)
            rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
            time.sleep(1)

            rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
            rb.connect_ros(True, False)
            time.sleep(1)

            rb.switch_to_customizedPose(defaultPose.prototypeFacialExpressions[i])
            rb.connect_ros(isSmoothly=True, isRecording=False ) # isSmoothly = True ,isRecording = True
            time.sleep(1)

            rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
            rb.connect_ros(True, False)
            time.sleep(1)

    rb.switch_to_customizedPose(lD)
    rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
    time.sleep(3)

    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)

    '''
    
    
    # 250ms
    # rb.DURATION = 2.25
    # steps = 5

    # 500ms
    # rb.DURATION = 2.5
    # steps = 10

    # 1000ms
    
    # 2000ms
    # rb.DURATION = 4
    # steps = 40

    # Hot Expression set
    # hE = defaultPose.hotExpressions
    # basicRunningCell(rb, hE, True, steps)

    # Recording protoFacialExpressions
    # pFE = defaultPose.prototypeFacialExpressions
    # basicRunningCell(rb, pFE, True, steps)




    # 1000ms
    # Hot Expression set
    # hE = defaultPose.hotExpressions
    # basicRunningCell(rb, hE, True)

    # Recording protoFacialExpressions
    # pFE = defaultPose.prototypeFacialExpressions
    # basicRunningCell(rb, pFE, True)

    # # Recording actionUnit
    # aUP = defaultPose.actionUnitParams
    # basicRunningCell(rb, aUP, True)



    # 20210521

    '''
    # Compare Two happiness

    # Return to Standard Pose
    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)

    # Go to the facial expressions
    print("Switch to {}".format("happiness"))
    rb.switch_to_customizedPose(defaultPose.prototypeFacialExpressions['happiness'])
    rb.connect_ros(isSmoothly=True, isRecording=True) # isSmoothly = True ,isRecording = True

    # Return to Standard Pose
    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)

    # Go to the facial expressions
    print("Switch to {}".format("happinessResultFixed"))
    rb.switch_to_customizedPose(defaultPose.fixedprototypeFacialExpressions['happinessResultFixed'])
    rb.connect_ros(isSmoothly=True, isRecording=True ) # isSmoothly = True ,isRecording = True

    # Return to Standard Pose
    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)
    '''


    '''
    # Recording fixed protoFacialExpressions
    fpFE = defaultPose.fixedprototypeFacialExpressions

    for k,v in fpFE.items():
        print("\n\n")
        # Return to Standard Pose
        rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
        rb.connect_ros(True, False)

        # Go to the facial expressions
        print("Switch to {}".format(k))
        rb.switch_to_customizedPose(v)
        rb.connect_ros(isSmoothly=True, isRecording=True, appendix="{}".format(k)) # isSmoothly = True ,isRecording = True


    # Return to Standard Pose
    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)
    '''


    '''
    # Recording protoFacialExpressions
    pFE = defaultPose.prototypeFacialExpressions
    print(pFE)

    for k,v in pFE.items():
        print("\n\n")
        # Return to Standard Pose
        rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
        rb.connect_ros(True, False)

        # Go to the facial expressions
        print("Switch to {}".format(k))
        rb.switch_to_customizedPose(v)
        rb.connect_ros(isSmoothly=True, isRecording=True, appendix="{}".format(k)) # isSmoothly = True ,isRecording = True


    # Return to Standard Pose
    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)
    '''



    # 20210521 Recording AU videos
    '''
    temp_counter = 0
    # print(rb.AUPose.keys())
    # print(rb.AUPose)
    for k,v in rb.AUPose.items():
        print("\n\n")
        if k == "StandardPose":
            print("skip StandardPose")
            continue
        # Return to Standard Pose
        rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
        rb.connect_ros(True, False)

        # Go to the AU
        print("Switch to {}".format(k))
        rb.switch_to_customizedPose(v)
        rb.connect_ros(True, True, "{}".format(k)) # isSmoothly = True ,isRecording = False
        temp_counter += 1
        # if temp_counter > 2:
        #     break

    rb.return_to_stable_state() # Return to the stable state (標準Pose)

    '''



    # StandardPose
    # print("Switch to StandardPose")
    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)

    # AU1
    # print("Switch to AU1")
    # rb.switch_to_customizedPose(rb.AUPose['AU1'])

    # AU2

    # AU4

    # AU5 

    # AU6 
    
    # AU7
    
    # AU10
    
    # AU12
    
    # LAU12
    
    # AU14
    
    # AU15 (Use 4 second to generate this facial expression), 19, 23
    # print("Switch to AU15")
    # rb.switch_to_customizedPose(rb.AUPose['AU15'])
    # rb.connect_ros(True, False, steps = 40) 
    # time.sleep(5)
    # AU16
    
    # AU18
    
    # AU20
    
    # AU22
    
    # AU25
    
    # AU26
    
    # AU43

    # print("Switch to StandardPose")
    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    # rb.connect_ros(True, False)

    '''
    rb.return_to_stable_state() # Return to the stable state (標準Pose)

    rb.switch_to_defaultPose(2) # Switch to default pose 2 笑顔
    rb.connect_ros(True, False)     # connect server and send the command to change facial expression smoothly
    time.sleep(3)

    rb.return_to_stable_state() # Return to the stable state (標準Pose)    
    '''


    # rb.switch_to_defaultPose(1) # Switch to default pose 1 標準
    # rb.connect_ros()            # # connect server and send the command, but change facial expression quickly    
    # time.sleep(3)
    
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
    finally:
        global rb
        rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
        rb.connect_ros(True, False)
        rb.webcam_stream_widget.stop()



'''
# Usage Example:
# A loop execution for defaultPose 1~3
for i in range(1, 4):
    rb.switch_to_defaultPose(i)
    rb.connect_ros(True)
    time.sleep(4)        


rb.switch_to_defaultPose(1)
rb.connect_ros(True)

'''

# Reference

# Checker Demo in main()
#     for j in range(2):
#         for i in [1,2,3,4,5,6,8,9,10,12,13,14,15,16]:
#             rb.switch_to_defaultPose(i) # Switch to default pose 2 笑顔
#             rb.connect_ros(True)     # connect server and send the command to change facial expression smoothly
#             time.sleep(3)

#     rb.switch_to_defaultPose(1) # Switch to default pose 1 標準
#     rb.connect_ros()            # # connect server and send the command, but change facial expression quickly    
#     time.sleep(3)

'''
1    左上瞼開閉    マブタの開口度を制御
2    右上瞼開閉    マブタの開口度を制御
3    左眼左右    眼球左右（左右35°振り分け）
4    右眼左右    眼球左右（左右35°振り分け）
5    眼上下    眼球上下（上下14°振り分け）
6    左下瞼開閉    笑顔
7    右下瞼開閉    笑顔
8    左外眉上げ    驚いた顔
9    左外眉下げ    柔和な顔
10    左内眉上げ    困った顔
11    左内眉寄せ    怒った顔
12    右外眉上げ    驚いた顔
13    右外眉下げ    柔和な顔
14    右内眉上げ    困った顔
15    右内眉寄せ    怒った顔
16    頬引き左    ほうれい線や頬肉の動き
17    頬引き右    ほうれい線や頬肉の動き
18    左口角上げ    笑顔
19    左口角下げ    泣き顔
20    左口角外引き    『い』の動き
21    左口角後引き    口を引き締める
22    右口角上げ    笑顔
23    右口角下げ    泣き顔
24    右口角外引き    『い』の動き
25    右口角後引き    口を引き締める
26    上唇すぼめ    『う』の口の動き
27    下唇すぼめ    『う』の口の動き
28    上唇上げ    上の歯を見せる
29    下唇下げ    下の歯を見せる
30    鼻よせ    鼻にしわを寄せる
31    舌上げ    舌を見せる『え』の発音
32    口開閉    アゴ全体の動作
33    頭傾げ左    頭の左右傾げ
34    頭傾げ右    頭の前後
35    首旋回    首左右ひねり
'''