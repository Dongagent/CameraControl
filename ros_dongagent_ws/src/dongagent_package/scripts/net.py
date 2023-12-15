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
            exit(0)
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