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
import time
import copy
# import cv2
import os, subprocess
import platform
import numpy as np
from bayes_opt import BayesianOptimization

# ----- for ros------
import rospy
from std_msgs.msg import String
import json
import base64
import time
import struct
# ----- for ros END------

# ----- for py-feat ------



# ----- for py-feat END ------


SPACE = ' '

DEBUG = 0 # 0 - Run; 1 - Debuging with robot; 2 - Debug WITHOUT robot 


class robot:
    def __init__(self, duration=3, fps=60):
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
            0, 32, 128, 128, 128
        ]

        # initialization of States, like [0, 0, 0, ... , 0]
        self.lastState = self.stableState # initialization
        self.nextState = self.stableState # initialization

        # initialization of lastParams
        self.lastParams = self.robotParams
        self.defaultPose = defaultPose.defaultPose
        self.AUPose = defaultPose.actionUnitParams

        # Camera Parameters
        self.DEVICE_ID = 1
        self.WIDTH = 1280
        self.HEIGHT = 720
        self.FPS = fps
        self.FRAMERATE = fps
        self.counter = 0 # used in the fileName
        self.fileName =  ""
        self.VIDEOSIZE = "1280x720"
        self.DURATION = duration
        self.client = ""

        # Final initialization
        self.initialize_robotParams()
        self.return_to_stable_state()

    def take_picture(self, isUsingCounter=True, appendix='', folder=''):
        # we don't need this
        # Allright, we need this 
        self.counter += 1
        if isUsingCounter:
            self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S_No", time.localtime()) + str(self.counter)
            if appendix:
                self.fileName += "_" + appendix + ".png"
            else:
                self.fileName += ".png"
        else:
            self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) 
            if appendix:
                self.fileName += "_" + appendix + ".png"
            else:
                self.fileName += ".png"
        if DEBUG == 2:
            print("Filename is {}".format(self.fileName))
            return
        if folder:
            folderPath = "/home/dongagent/github/CameraControl/algorithm/{}/".format(folder)
            if not os.path.exists(folderPath):
                try:
                    os.mkdir(folderPath)
                except Exception as e:
                    print(e)

        if not folder:
            self.fileName = "/home/dongagent/github/CameraControl/algorithm/tempimg/{}".format(self.fileName)
        else:
            self.fileName = folderPath + self.fileName

        if os.path.exists(self.fileName):
            raise Exception("Same File!")
        if "Linux" in platform.platform():
            # Remember to check the path everytime.
            videoPath = "/dev/video2"
            fParam = "v4l2"
            videoTypeParm = "-input_format"
        elif "Windows" in platform.platform():
            videoPath = "video='C922 Pro Stream Webcam'"
            fParam = "dshow" 
            videoTypeParm = "-vcodec"

        # only the command is different from take_video
        command = "ffmpeg -f {} -i {} -vframes 1 {}".format(
            fParam, 
            videoPath, 
            self.fileName)
        # ffmpeg -f v4l2 -i /dev/video2 -vframes 1 /home/dongagent/github/CameraControl/algorithm/test.png

        if "Linux" in platform.platform():
            # Linux
            return subprocess.Popen([command], stdout=subprocess.PIPE, shell=True)
        elif "Windows" in platform.platform():
            # Windows
            return subprocess.Popen(["pwsh", "-Command", command], stdout=subprocess.PIPE)

    def take_video(self, isUsingCounter=True, appendix=''):
        self.counter += 1
        if isUsingCounter:
            self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S_No", time.localtime()) + str(self.counter)
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
        if DEBUG == 2:
            print("Filename is {}".format(self.fileName))
            return

        if os.path.exists(self.fileName):
            raise Exception("Same File!")
        if "Linux" in platform.platform():
            # Remember to check the path everytime.
            videoPath = "/dev/video2"
            fParam = "v4l2"
            videoTypeParm = "-input_format"
        elif "Windows" in platform.platform():
            videoPath = "video='C922 Pro Stream Webcam'"
            fParam = "dshow" 
            videoTypeParm = "-vcodec"
        
        command = "ffmpeg -f {} -framerate {} -video_size {} {} mjpeg -t {} -i {} -t {} -c copy {}".format(
            fParam, 
            str(self.FRAMERATE), 
            self.VIDEOSIZE, 
            videoTypeParm, 
            str(self.DURATION), 
            videoPath, 
            str(self.DURATION), 
            self.fileName
        )
        # ffmpeg -f v4l2 -framerate 60 -video_size 1280x720 -input_format mjpeg -i /dev/video2 -vf vflip -c copy 1.mkv
        
        if "Linux" in platform.platform():
            # Linux
            return subprocess.Popen([command], stdout=subprocess.PIPE, shell=True)
        elif "Windows" in platform.platform():
            # Windows
            return subprocess.Popen(["pwsh", "-Command", command], stdout=subprocess.PIPE)
            
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
            0, 32, 128, 128, 128
        ]
        for i in range(1, 36):
            self.robotParams["x{}".format(i)] = stableState[i - 1]
        print("\n")
        self.__check_robotParams()
        # Drive the robot to the 
        self.connect_ros(isSmoothly=True)
        print("return_to_stable_state, self.robotParams are all set")

    def transfer_robotParams_to_states(self, params):
        states = [0 for x in range(36)]
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
        assert len(customizedPose) == 35, "ERROR! The customizedPose don't have 35 axes."
        # States
        self.lastState = self.nextState
        self.nextState = customizedPose
        print("self.lastState", self.lastState)
        print("self.nextState", self.nextState)

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

    # @deprecated
    def generate_execution_code(self, params):
        """Construct the action code as 'moveaxis [axis] [pos] [priority]' """
        # Generate execution code with given params
        actionStr = "moveaxes"
        try: 
            for i in range(1, 36):
                actionStr = actionStr + SPACE + str(i) + SPACE + str(params["x{}".format(i)]) + " 5 200"
            actionStr += '\n' 
            self.executionCode = actionStr
        except Exception as e:
            print("generate_execution_code ERROR")
            print(e)

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

                self.generate_execution_code(currentParams)
                self.nextState = self.transfer_robotParams_to_states(currentParams)
                # self.__sendExecutionCode() # Use socket
                self.ros_talker()# Use ROS

                time.sleep(0.05)

    def normal_execution_mode(self):
        self.generate_execution_code(self.robotParams)
        # self.__sendExecutionCode() # Use socket
        self.ros_talker()# Use ROS

    def ros_talker(self):
        pub = rospy.Publisher('rc/command', String, queue_size=10)
        sub = rospy.Subscriber('rc/return', String, self.sub_callback)
        rospy.init_node('rctest', anonymous=True)

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
            # r.sleep()
        
        # Now use ROS instead

    # @deprecated
    def __sendExecutionCode(self):
        # This function cannot be called outside
        assert "move" in self.executionCode
        msg = self.executionCode # message

        self.client.send(msg.encode('utf-8')) # send a message to server, python3 only receive byte data
        data = self.client.recv(1024) # receive a message, the maximum length is 1024 bytes
        # print('recv:', data.decode()) # print the data I received
        if "OK" not in data.decode():
            raise Exception("ERROR! Did NOT receive 'OK'")

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

    def connect_ros(self, isSmoothly=False, isRecording=False, appendix="", steps=20, timeIntervalBeforeExp=1):

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
                # Start Record if isRecording
                if isRecording:
                    # Please set which recording system you want to use. Video or image.
                    # For image, we need to take photos after conneect_socket
                    process = self.take_video(isUsingCounter=False, appendix=appendix)
                    # process = self.take_picture(isUsingCounter=False, appendix=appendix)
                    # time.sleep(1)
                # Smoothly execute
                if isSmoothly:
                    print("Smoothly execution activated")
                    if isRecording:
                        time.sleep(timeIntervalBeforeExp) # Sleep 1 second by default to wait for the start of the video 
                    self.smooth_execution_mode(steps)
                # Otherwise
                else:
                    self.normal_execution_mode()
                # self.client.close() # use ros now
                
                # Close Record
                if isRecording:
                    process.wait()
                    if process.returncode != 0:
                        print(process.stdout.readlines())
                        raise Exception("The subprocess does NOT end.")
            else:
                raise Exception("Connection Failed")
        except rospy.ROSInterruptException: 
            print(rospy.ROSInterruptException)


        
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
        py_feat_analysis(self.fileName, target_emotion_name)

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
        rb.connect_ros(isSmoothly=True, isRecording=isRecordingFlag, appendix="{}".format(k), steps=steps) # isSmoothly = True ,isRecording = True

    # Return to Standard Pose
    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)

def get_target(emotion_name):
    '''
    Anger, Disgust, Fear, Happiness, Sadness, Surprise
    or lowercase
    '''
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


def py_feat_analysis(img, target_emotion):
    '''
        @img: file name 
        @target_emotion: Anger, Disgust, Fear, Happiness, Sadness, Surprise
    '''
    from feat import Detector
    face_model = "retinaface"
    landmark_model = "mobilenet"
    au_model = "rf"
    # au_model = "JAANET"
    emotion_model = "resmasknet"
    # emotion_model = "rf"

    detector = Detector(au_model = au_model, emotion_model = emotion_model)

    image_prediction = detector.detect_image(img)
    df = image_prediction.head()
    print(df.iloc[:, -8:])
    csv_name = img[:-4]+".csv"
    csv_emotion_name = img[:-4]+"emotion.csv"
    df.to_csv(csv_name)
    df.iloc[:, -8:].to_csv(csv_emotion_name)
    targetID = get_target(target_emotion)
    return df.iloc[:, -8:].iloc[0,targetID]

def checkParameters(robotParams):
    # Axis (8, 9), (12, 13), (18, 19), (22, 23), 
    # we should use a * b = 0 for each group. 
    # Which means, take (8, 9) for example. 
    # When axis 8 has value, we should make sure axis 9 is set to 0. 
    if robotParams[8-1] * robotParams[9-1] != 0:
        robotParams[np.random.choice([8-1, 9-1])] = 0
    if robotParams[12-1] * robotParams[13-1] != 0:
        robotParams[np.random.choice([12-1, 13-1])] = 0
    if robotParams[18-1] * robotParams[19-1] != 0:
        robotParams[np.random.choice([18-1, 19-1])] = 0
    if robotParams[22-1] * robotParams[23-1] != 0:
        robotParams[np.random.choice([22-1, 23-1])] = 0
        
    assert robotParams[8-1] * robotParams[9-1] == 0
    assert robotParams[12-1] * robotParams[13-1] == 0
    assert robotParams[18-1] * robotParams[19-1] == 0
    assert robotParams[22-1] * robotParams[23-1] == 0
    return robotParams

def target_function(**kwargs):
    """Pyfeat evaluation object
    
    Target：
        Maxmize the result of Pyfeat
    """
    rb = kwargs['robot']
    target_emotion = kwargs['target_emotion']
    kwargs = kwargs["kwargs"]
    

    # Get robot parameters
    neutral = [86, 86, 128, 128, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, 122]
    fixedrobotcode = copy.copy(neutral)
    
    # dict = {}
    for k,v in kwargs.items():
        # print(k,v)
        if "x" in k:
            fixedrobotcode[int(k[1:])-1] = round(v)

    # x2 = x1, use one axis for eyes upper lid
    fixedrobotcode[0] = 0
    fixedrobotcode[1] = fixedrobotcode[0]

    # x7 = x6, use one axis for eyes lower lid
    fixedrobotcode[6] = fixedrobotcode[5]

    # x13 = x9
    fixedrobotcode[12] = fixedrobotcode[8]

    # x17 = x16
    fixedrobotcode[16] = fixedrobotcode[15]

    # x22 = x18
    fixedrobotcode[21] = fixedrobotcode[17]


    # check parameters
    fixedrobotcode = checkParameters(fixedrobotcode)
    
    # control robot 
    print("fixedrobotcode is", fixedrobotcode)
    rb.switch_to_customizedPose(fixedrobotcode)
    rb.connect_ros(isSmoothly=True, isRecording=False) # isSmoothly = True ,isRecording = True
    time.sleep(1.5)
    # Take photo
    global COUNTER
    rb.take_picture(isUsingCounter=False, appendix='{}_{}'.format(target_emotion, COUNTER), folder=target_emotion)
    COUNTER += 1

    # Py-feat Analysis
    # Temp: analyze Anger
    return py_feat_analysis(img=rb.fileName, target_emotion=target_emotion)    

def bayesian_optimization(baseline, target_emotion, robot):

    def middle_function(**kwargs):
        # parameter
        return target_function(robot=robot, target_emotion=target_emotion, kwargs=kwargs)


    optimizer = BayesianOptimization(
        f=middle_function,
        # Define HyperParameter Space

        # Anger 
        pbounds={
            # "x1": (0, 255), 
            # "x2": (0, 255),
            # "x3": (0, 255), 
            # "x4": (0, 255), 
            # "x5": (0, 255), 
            "x6": (0, 255), 
            # "x7": (0, 255), 
            # "x8": (0, 255), 
            # "x9": (0, 255), 
            # "x10": (0, 255),
            "x11": (0, 255), 
            # "x12": (0, 255), 
            # "x13": (0, 255), 
            # "x14": (0, 255), 
            "x15": (0, 255), 
            # "x16": (0, 255), 
            # "x17": (0, 255), 
            # "x18": (0, 255), 
            # "x19": (0, 255), 
            # "x20": (0, 255), 
            # "x21": (0, 255), 
            # "x22": (0, 255), 
            # "x23": (0, 255), 
            # "x24": (0, 255), "x25": (0, 255), 
            # "x26": (0, 255), "x27": (0, 255), 
            # "x28": (0, 255), "x29": (0, 255), # hot
            # "x30": (0, 255), 
            # "x31": (0, 255), 
            # "x32": (0, 200),  #hot
            # "x33": (0, 255), "x34": (0, 255), "x35": (0, 255)
        },
        

        # # happiness 
        # pbounds={
        #     # "x1": (0, 255), 
        #     # "x2": (0, 255),
        #     # "x3": (0, 255), 
        #     # "x4": (0, 255), 
        #     # "x5": (0, 255), 
        #     "x6": (0, 255), 
        #     # "x7": (0, 255), 
        #     # "x8": (0, 255), 
        #     "x9": (0, 255), 
        #     # "x10": (0, 255),
        #     # "x11": (0, 255), 
        #     # "x12": (0, 255), 
        #     # "x13": (0, 255), 
        #     # "x14": (0, 255), 
        #     # "x15": (0, 255), 
        #     "x16": (0, 255), 
        #     # "x17": (0, 255), 
        #     "x18": (0, 255), 
        #     # "x19": (0, 255), 
        #     # "x20": (0, 255), 
        #     # "x21": (0, 255), 
        #     # "x22": (0, 255), 
        #     # "x23": (0, 255), 
        #     # "x24": (0, 255), "x25": (0, 255), 
        #     # "x26": (0, 255), "x27": (0, 255), 
        #     "x28": (0, 255), "x29": (0, 255), # hot
        #     # "x30": (0, 255), 
        #     # "x31": (0, 255), 
        #     "x32": (0, 200),  #hot
        #     # "x33": (0, 255), "x34": (0, 255), "x35": (0, 255)
        # },

        random_state=42,
        verbose=2)
    # 2个初始化点和10轮优化，共12轮
    # initialization of pre-define facial expression
    neutral_baseline = defaultPose.prototypeFacialExpressions["netural"]

    neutral_baseline = defaultPose.prototypeFacialExpressions["netural"]
    subtract = abs(np.array(neutral_baseline) - np.array(baseline))
    probe_param = {}
    for i in range(len(subtract)):
        if subtract[i] != 0:
            probe_param["x{}".format(i+1)] = subtract[i]
    # print(probe_param)

    # # Happiness
    # optimizer.probe(
    #     params={
    #         'x6': 255, 'x9': 255, 'x16': 255, 'x18': 255, 'x28': 255, 'x29': 255, 'x32': 255},
    #     lazy=False,
    # )
    # optimizer.probe(
    #     params={
    #         'x16': 221, 'x18': 153, 'x28': 180, 'x29': 5, 'x32': 247, 'x6': 212, 'x9': 54},
    #     lazy=False,
    # )

    # Anger
    # optimizer.probe(
    #     params={"x1": 0, "x6": 255, "x11": 255, "x15": 255},
    #     lazy=False,
    # )

    optimizer.probe(
        params={"x6": 255, "x11": 255, "x15": 255},
        lazy=False,
    )

    

    # optimizer.maximize(init_points=2, n_iter=10)
    optimizer.maximize(alpha=1e-1)

    print("Final result:", optimizer.max)
    return optimizer



def main():
    # global rb
    rb = robot(duration=2)
    assert rb.connection == True
    # rb.transfer_robotParams_to_states(rb.lastParams, [x for x in range(36)])
    # print("rb.lastParams", rb.lastParams)
    # defaultPose.prototypeFacialExpressions

    # 2021.11.22
    # Happiness Experiment
    # print('\n{}\n'.format("happiness"))
    # target_emotion = "happiness"
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



    # Anger Experiment
    print('\n{}\n'.format("anger"))
    target_emotion = "anger"
    global COUNTER
    COUNTER = 0
    optimizer = bayesian_optimization(
        baseline=defaultPose.prototypeFacialExpressions[target_emotion], 
        target_emotion=target_emotion,
        robot=rb)
    print(optimizer.res)
    # Return to normal
    rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
    rb.connect_ros(True, False)


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
    neuExp = defaultPose.experiment1['netural']
    angExp = defaultPose.experiment1['anger']
    hapExp = defaultPose.experiment1['happiness']

    print(len(angExp))


    # 6 trails
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
    main()


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