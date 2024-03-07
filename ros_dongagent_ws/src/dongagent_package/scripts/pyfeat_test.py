from feat import Detector
detector = Detector(au_model = 'xgb', emotion_model = "resmasknet", landmark_model='mobilefacenet')

filePath = "image_analysis/anger/2023_12_18_16_51_59_anger_32.png"
# filePath = "../../../../anger_human2.jpeg"
res = detector.detect_image(filePath)

keys = res.columns.tolist()
vals = res.values.tolist()[0]
print(keys)
print(vals)

for i in range(len(keys)):
    print(keys[i], vals[i])
print()
print(detector.au_model)
print(detector.emotion_model)
# print(res.iloc[-1:,-8:])
# print(res.iloc[-1:,-9:-1])

# print(res.aus)