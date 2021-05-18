#! /usr/bin/python
# -*- coding:utf-8 -*-
import random
import defaultPose
import socket
import time
import copy
import cv2
import os, subprocess

SPACE = ' '


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

		self.initialize_robotParams()
		self.return_to_stable_state()

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

	def take_picture(self):
		# C Program
		pass

	def take_video(self):
# 			process.wait()
# 			if process.returncode != 0:
# 				print(process.stdout.readlines())
# 				raise "The subprocess does NOT end."
		self.counter += 1
		self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S_No", time.localtime()) + str(self.counter) + ".mkv"
		if os.path.exists(self.fileName):
			raise Exception("Same File!")
    	
		# command = "ffmpeg -i /dev/video2"
		command = "ffmpeg -f v4l2 -framerate " + str(self.FRAMERATE) + " -video_size " + self.VIDEOSIZE + " -input_format mjpeg -t " + str(self.DURATION) + " -i /dev/video2 -t " + str(self.DURATION) + " -c copy " + self.fileName

		return subprocess.Popen([command], stdout=subprocess.PIPE, shell=True)
            
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

	def smooth_execution_mode(self, steps = 5):
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
					# 	print(self.lastParams[k], self.robotParams[k], interval, currentParams[k])

				self.generate_execution_code(currentParams)
				self.__send()
				time.sleep(0.2)

	def connect_socket(self, isSmoothly=False, isRecording=False):
		self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)				# create socket object
		
		# Please use ipconfig on the server to check the ip first. It may change every time we open the server.
		host = '172.27.174.142'  													# set server address
		port = 12000			 													# set port

		try:
			self.client.connect((host, port))
			self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		except Exception as e:
			print("Connection Failed, ERROR code is: ", e)
			self.connection = False

		if self.connection:
			# Smoothly execute # TODO 回去重新整理这部分code
			if isRecording:
				process = self.take_video()
			if isSmoothly:
				print("Smoothly execution activated")
				time.sleep(1)
				self.smooth_execution_mode()
			# Otherwise
			else:
				self.generate_execution_code(self.robotParams)
				self.__send()
			self.client.close()
			if isRecording:
				process.wait()
				if process.returncode != 0:
					print(process.stdout.readlines())
					raise "The subprocess does NOT end."
		else:
			raise Exception("Connection Failed")

	def __check_robotParams(self):
		# This function cannot be called outside
		assert len(self.robotParams) == 35, "len(robotParams) != 35"

	def __send(self):
		# This function cannot be called outside
		assert "move" in self.executionCode
		msg = self.executionCode # message

		self.client.send(msg.encode('utf-8')) # send a message to server, python3 only receive byte data
		data = self.client.recv(1024) # receive a message, the maximum length is 1024 bytes
		# print('recv:', data.decode()) # print the data I received
		if "OK" not in data.decode():
			raise "ERROR! Did NOT receive 'OK'"

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
	
	rb.return_to_stable_state() # Return to the stable state (標準Pose)

# 	for j in range(2):
# 		for i in [1,2,3,4,5,6,8,9,10,12,13,14,15,16]:
# 			rb.switch_to_defaultPose(i) # Switch to default pose 2 笑顔
# 			rb.connect_socket(True)	 # connect server and send the command to change facial expression smoothly
# 			time.sleep(3)

# 	rb.switch_to_defaultPose(1) # Switch to default pose 1 標準
# 	rb.connect_socket()			# # connect server and send the command, but change facial expression quickly	
# 	time.sleep(3)



	rb.switch_to_defaultPose(2) # Switch to default pose 2 笑顔
	rb.connect_socket(True, True)	 # connect server and send the command to change facial expression smoothly
	time.sleep(3)

	rb.return_to_stable_state() # Return to the stable state (標準Pose)    

	# rb.switch_to_defaultPose(1) # Switch to default pose 1 標準
	# rb.connect_socket()			# # connect server and send the command, but change facial expression quickly	
	# time.sleep(3)

	'''
	# 
	# A loop execution for defaultPose 1~3
	for i in range(1, 4):
		rb.switch_to_defaultPose(i)
		rb.connect_socket(True)
		time.sleep(4)		


	rb.switch_to_defaultPose(1)
	rb.connect_socket(True)

	'''

if __name__ == '__main__':
	main()


# Reference

'''
1	左上瞼開閉	マブタの開口度を制御
2	右上瞼開閉	マブタの開口度を制御
3	左眼左右	眼球左右（左右35°振り分け）
4	右眼左右	眼球左右（左右35°振り分け）
5	眼上下	眼球上下（上下14°振り分け）
6	左下瞼開閉	笑顔
7	右下瞼開閉	笑顔
8	左外眉上げ	驚いた顔
9	左外眉下げ	柔和な顔
10	左内眉上げ	困った顔
11	左内眉寄せ	怒った顔
12	右外眉上げ	驚いた顔
13	右外眉下げ	柔和な顔
14	右内眉上げ	困った顔
15	右内眉寄せ	怒った顔
16	頬引き左	ほうれい線や頬肉の動き
17	頬引き右	ほうれい線や頬肉の動き
18	左口角上げ	笑顔
19	左口角下げ	泣き顔
20	左口角外引き	『い』の動き
21	左口角後引き	口を引き締める
22	右口角上げ	笑顔
23	右口角下げ	泣き顔
24	右口角外引き	『い』の動き
25	右口角後引き	口を引き締める
26	上唇すぼめ	『う』の口の動き
27	下唇すぼめ	『う』の口の動き
28	上唇上げ	上の歯を見せる
29	下唇下げ	下の歯を見せる
30	鼻よせ	鼻にしわを寄せる
31	舌上げ	舌を見せる『え』の発音
32	口開閉	アゴ全体の動作
33	頭傾げ左	頭の左右傾げ
34	頭傾げ右	頭の前後
35	首旋回	首左右ひねり
'''