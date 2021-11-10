# Check Fex class installation.
# from feat import Fex
# fex = Fex()

# Check Detector class installation.
# from feat import Detector
# detector = Detector()

from feat import Detector
face_model = "retinaface"
landmark_model = "mobilenet"
au_model = "rf"
# au_model = "JAANET"
emotion_model = "resmasknet"
# emotion_model = "rf"

detector = Detector(au_model = au_model, emotion_model = emotion_model)
# detector = Detector(face_model = face_model, landmark_model = landmark_model, au_model = au_model, emotion_model = emotion_model)

# image prediction
# for i in ["happyness", "sadness", "surprise"]:
# for i in ["anger", "disgust", "fear"]:
#     image_prediction = detector.detect_image(i + ".png")
#     image_prediction.plot_detections()

# video prediction
import os
filePath = "./"
videoNames = os.listdir(filePath)
flag = 0

video_prediction = detector.detect_video(inputFname="250_disgust.mp4", skip_frames=0)
print(video_prediction.emotions())
print(video_prediction.aus())       


# for i in videoNames:
#     if ".mp4" not in i:
#         print(i)
#         continue
#     if flag < 1:
#         tempVideoPath = os.path.join(filePath, i)
#         print(tempVideoPath)
#         # video_prediction = detector.detect_video(tempVideoPath, skip_frames=0)
#         video_prediction = detector.detect_video(tempVideoPath)
        # print(video_prediction.emotions())
        # print(video_prediction.aus())

        # flag += 1