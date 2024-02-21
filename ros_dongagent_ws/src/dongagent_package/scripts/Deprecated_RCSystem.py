# Deprecated
def take_video(self, isUsingCounter=True, appendix=''):
    pass
    #     self.counter += 1
    #     if isUsingCounter:
    #         self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S_No", time.localtime()) + str(self.counter)
    #         if appendix:
    #             self.fileName += "_" + appendix + ".mkv"
    #         else:
    #             self.fileName += ".mkv"
    #     else:
    #         self.fileName = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) 
    #         if appendix:
    #             self.fileName += "_" + appendix + ".mkv"
    #         else:
    #             self.fileName += ".mkv"
    #     if DEBUG == 2:
    #         print("Filename is {}".format(self.fileName))
    #         return

    #     if os.path.exists(self.fileName):
    #         raise Exception("Same File!")
    #     if "Linux" in platform.platform():
    #         # Remember to check the path everytime.
            
    #         videoPath = LINUXVIDEOPATH
    #         fParam = "v4l2"
    #         videoTypeParm = "-input_format"
    #     elif "Windows" in platform.platform():
    #         videoPath = "video='C920 Pro Stream Webcam'"
    #         fParam = "dshow" 
    #         videoTypeParm = "-vcodec"
        
    #     command = "ffmpeg -f {} -framerate {} -video_size {} {} mjpeg -t {} -i {} -t {} -c copy {}".format(
    #         fParam, 
    #         str(self.FRAMERATE), 
    #         self.VIDEOSIZE, 
    #         videoTypeParm, 
    #         str(self.DURATION), 
    #         videoPath, 
    #         str(self.DURATION), 
    #         self.fileName
    #     )
    #     # ffmpeg -f v4l2 -framerate 60 -video_size 1280x720 -input_format mjpeg -i /dev/video2 -vf vflip -c copy 1.mkv
        
    #     if "Linux" in platform.platform():
    #         # Linux
    #         return subprocess.Popen([command], stdout=subprocess.PIPE, shell=True)
    #     elif "Windows" in platform.platform():
    #         # Windows
    #         return subprocess.Popen(["pwsh", "-Command", command], stdout=subprocess.PIPE)

# @deprecated
def generate_execution_code(self, params):
    pass
    #     """Construct the action code as 'moveaxis [axis] [pos] [priority]' """
    #     # Generate execution code with given params
    #     actionStr = "moveaxes"
    #     try: 
    #         for i in range(1, 36):
    #             actionStr = actionStr + SPACE + str(i) + SPACE + str(params["x{}".format(i)]) + " 5 200"
    #         actionStr += '\n' 
    #         self.executionCode = actionStr
    #     except Exception as e:
    #         print("generate_execution_code ERROR")
    #         print(e)
















    