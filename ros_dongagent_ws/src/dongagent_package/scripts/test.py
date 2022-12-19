from feat import Detector
detector = Detector(emotion_model = 'resmasknet')
image_prediction = detector.detect_image('nikola.png')
print(image_prediction)