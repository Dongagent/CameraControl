# from feat import Fex
# fex = Fex()

# from feat import Detector
# detector = Detector()

from feat import Detector
face_model = "retinaface"
landmark_model = "mobilenet"
au_model = "rf"
emotion_model = "resmasknet"
detector = Detector(face_model = face_model, landmark_model = landmark_model, au_model = au_model, emotion_model = emotion_model)

# image prediction
# for i in ["happyness", "sadness", "surprise"]:
# for i in ["anger", "disgust", "fear"]:
#     image_prediction = detector.detect_image(i + ".png")
#     image_prediction.plot_detections()

# video prediction
import os
filePath = "../Study3/"
videoNames = os.listdir(filePath)
flag = 0

# single file
tempVideoPath = '2021_05_14_19_44_14_No1.mkv'
print(tempVideoPath)
# video_prediction = detector.detect_video(tempVideoPath, skip_frames=0)
video_prediction = detector.detect_video(tempVideoPath)
video_prediction.emotions().plot()
print(video_prediction.head())

# multi files
'''
for i in videoNames:
    if flag < 1:
        tempVideoPath = os.path.join(filePath, i)
        print(tempVideoPath)
        # video_prediction = detector.detect_video(tempVideoPath, skip_frames=0)
        video_prediction = detector.detect_video(tempVideoPath)
        video_prediction.emotions().plot()
        print(video_prediction.head())

        flag += 1
'''