# -*- coding: utf-8 -*-
# @Author: Dongsheng Yang
# @Email:  yang.dongsheng.46w@st.kyoto-u.ac.jp
# @Copyright = Copyright 2021, The Riken Robotics Project
# @Date:   2021-05-20 18:19:38
# @Last Modified by:   dongshengyang
# @Last Modified time: 2021-05-20 18:21:38

__author__ = "Dongsheng Yang"
__copyright__ = "Copyright 2021, The Riken Robotics Project"
__version__ = "1.0.0"
__maintainer__ = "Dongsheng Yang"
__email__ = "yang.dongsheng.46w@st.kyoto-u.ac.jp"
__status__ = "Developing"


import random
import defaultPose
import socket
import time
import copy
# import cv2
import os, subprocess
import platform

SPACE = ' '

DEBUG = 2 # 0 - Run; 1 - Debuging with robot; 2 - Debug WITHOUT robot 


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

    def take_picture(self):
        # C Program
        pass

    def take_video(self, isUsingCounter=True, apendix=''):
#             process.wait()
#             if process.returncode != 0:
#                 print(process.stdout.readlines())
#                 raise "The subprocess does NOT end."
        self.counter += 1
        if isUsingCounter:
            self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S_No", time.localtime()) + str(self.counter)
            if apendix:
                self.fileName += "_" + apendix + ".mkv"
            else:
                self.fileName += ".mkv"
        else:
            self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) 
            if apendix:
                self.fileName += "_" + apendix + ".mkv"
            else:
                self.fileName += ".mkv"
        if DEBUG == 2:
            print("Filename is {}".format(self.fileName))
            return

        if os.path.exists(self.fileName):
            raise Exception("Same File!")
        if "Linux" in platform.platform():
            videoPath = "/dev/video2"
            fParam = "v4l2"
            videoTypeParm = "-input_format"
        elif "Windows" in platform.platform():
            videoPath = "video='C922 Pro Stream Webcam'"
            fParam = "dshow" 
            videoTypeParm = "-vcodec"
        
        command = "ffmpeg -f {} -framerate {} -video_size {} {} mjpeg -t {} -i {} -t {} -c copy {}".format(fParam, str(self.FRAMERATE), self.VIDEOSIZE, videoTypeParm, str(self.DURATION), videoPath, str(self.DURATION), self.fileName)
        
        if "Linux" in platform.platform():
            # Linux
            return subprocess.Popen([command], stdout=subprocess.PIPE, shell=True)
        elif "Windows" in platform.platform():
            # Windows
            return subprocess.Popen(["pwsh", "-Command", command], stdout=subprocess.PIPE)
            
    def initialize_robotParams(self):
        # initialize robotParams like {"1":0, "2":0, ... , "35": 0}
        print("initialize_robotParams")
        for i in range(1, 36):
            codeNum = str(i)
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
            self.robotParams[str(i)] = stableState[i - 1]
        
        self.__check_robotParams()
        # Drive the robot to the 
        self.connect_socket(True)
        print("return_to_stable_state, self.robotParams are all set")

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
            self.robotParams[str(i)] = params[i - 1]
        self.__check_robotParams()

    def generate_execution_code(self, params):
        """Construct the action code as 'moveaxis [axis] [pos] [priority]' """
        # Generate execution code with given params
        actionStr = "moveaxes"
        try: 
            for i in range(1, 36):
                actionStr = actionStr + SPACE + str(i) + SPACE + str(params[str(i)]) + " 5 200"
            actionStr += '\n' 
            self.executionCode = actionStr
        except Exception as e:
            print("generate_execution_code ERROR")
            print(e)

    def smooth_execution_mode(self, steps = 10):
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
                self.__sendExecutionCode()
                time.sleep(0.1)

    def normal_execution_mode(self):
        self.generate_execution_code(self.robotParams)
        self.__sendExecutionCode()

    def connect_socket(self, isSmoothly=False, isRecording=False, apendix=""):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                # create socket object
        
        # Please use ipconfig on the server to check the ip first. It may change every time we open the server.
        host = '172.27.174.142'                                                      # set server address
        port = 12000                                                                 # set port

        if DEBUG == 2:
            print("function connect_socket:", self.robotParams)
            # Test fileName
            if isRecording:
                self.take_video(False, apendix)
            return
        try:
            self.client.connect((host, port))
            self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except Exception as e:
            print("Connection Failed, ERROR code is: ", e)
            self.connection = False

        if self.connection:
            # Start Record if isRecording
            if isRecording:
                process = self.take_video(apendix)
            # Smoothly execute
            if isSmoothly:
                print("Smoothly execution activated")
                time.sleep(1) # Sleep 1second to wait for the start of the video 
                self.smooth_execution_mode()
            # Otherwise
            else:
                self.normal_execution_mode()
                # self.generate_execution_code(self.robotParams)
                # self.__sendExecutionCode()
            self.client.close()
            
            # Close Record
            if isRecording:
                process.wait()
                if process.returncode != 0:
                    print(process.stdout.readlines())
                    raise Exception("The subprocess does NOT end.")
        else:
            raise Exception("Connection Failed")

    def __check_robotParams(self):
        # This function cannot be called outside
        assert len(self.robotParams) == 35, "len(robotParams) != 35"

    def __sendExecutionCode(self):
        # This function cannot be called outside
        assert "move" in self.executionCode
        msg = self.executionCode # message

        self.client.send(msg.encode('utf-8')) # send a message to server, python3 only receive byte data
        data = self.client.recv(1024) # receive a message, the maximum length is 1024 bytes
        # print('recv:', data.decode()) # print the data I received
        if "OK" not in data.decode():
            raise Exception("ERROR! Did NOT receive 'OK'")

    def perform_openface(self, figure):
        # send figure to openface and get result
        # TODO: Do something with openface
        # Subprocess
        openfacePath = ""
        figurePath = ""

        return openfacePath

    def analysis(self):
        pass

    def feedback(self):
        pass

    
def main():
    rb = robot()
    assert rb.connection == True
    
    

    # print(rb.AUPose.keys())
    # print(rb.AUPose)
    for k,v in rb.AUPose.items():
        print("\n\n")
        if k == "StandardPose":
            print("skip StandardPose")
            continue
        # Return to Standard Pose
        rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
        rb.connect_socket(True, False)

        # Go to the AU
        print("Switch to {}".format(k))
        rb.switch_to_customizedPose(v)
        rb.connect_socket(True, True, "{}".format(k))
        




    # StandardPose
    # print("Switch to StandardPose")
    # rb.switch_to_customizedPose(rb.AUPose['StandardPose'])

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
    
    # AU15
    
    # AU16
    
    # AU18
    
    # AU20
    
    # AU22
    
    # AU25
    
    # AU26
    
    # AU43


    '''
    rb.return_to_stable_state() # Return to the stable state (標準Pose)

    rb.switch_to_defaultPose(2) # Switch to default pose 2 笑顔
    rb.connect_socket(True, False)     # connect server and send the command to change facial expression smoothly
    time.sleep(3)

    rb.return_to_stable_state() # Return to the stable state (標準Pose)    
    '''


    # rb.switch_to_defaultPose(1) # Switch to default pose 1 標準
    # rb.connect_socket()            # # connect server and send the command, but change facial expression quickly    
    # time.sleep(3)

    

if __name__ == '__main__':
    main()


'''
# Usage Example:
# A loop execution for defaultPose 1~3
for i in range(1, 4):
    rb.switch_to_defaultPose(i)
    rb.connect_socket(True)
    time.sleep(4)        


rb.switch_to_defaultPose(1)
rb.connect_socket(True)

'''

# Reference

# Checker Demo in main()
#     for j in range(2):
#         for i in [1,2,3,4,5,6,8,9,10,12,13,14,15,16]:
#             rb.switch_to_defaultPose(i) # Switch to default pose 2 笑顔
#             rb.connect_socket(True)     # connect server and send the command to change facial expression smoothly
#             time.sleep(3)

#     rb.switch_to_defaultPose(1) # Switch to default pose 1 標準
#     rb.connect_socket()            # # connect server and send the command, but change facial expression quickly    
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