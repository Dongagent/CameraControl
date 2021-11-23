from sklearn.datasets import make_classification
from sklearn.model_selection import cross_val_score
from sklearn.ensemble import RandomForestClassifier as RFC
from bayes_opt import BayesianOptimization
from feat import Detector
import copy
import RCSystem
import numpy as np

def get_target(emotion_name):
    
    # Anger, Disgust, Fear, Happiness, Sadness, Surprise
    
    if emotion_name == "Anger":
        return 0
    elif emotion_name == "Disgust":
        return 1
    elif emotion_name == "Fear":
        return 2
    elif emotion_name == "Happiness":
        return 3
    elif emotion_name == "Sadness":
        return 4
    elif emotion_name == "Surprise":
        return 5

def py_feat_analysis(**kwargs):
    
        # @img: file name 
        # @target_emotion: Anger, Disgust, Fear, Happiness, Sadness, Surprise
    
    from feat import Detector
    face_model = "retinaface"
    landmark_model = "mobilenet"
    au_model = "rf"
    # au_model = "JAANET"
    # emotion_model = "resmasknet"
    emotion_model = "rf"

    detector = Detector(au_model = au_model, emotion_model = emotion_model)

    image_prediction = detector.detect_image(kwargs["img"])
    df = image_prediction.head()
    print(df.iloc[:, -8:])
    # csv_name = img[:-4]+".csv"
    # csv_emotion_name = img[:-4]+"emotion.csv"
    # df.to_csv(csv_name)
    # df.iloc[:, -8:].to_csv(csv_emotion_name)
    targetID = get_target(kwargs["target_emotion"])
    return df.iloc[:, -8:].iloc[0,targetID]

def checkParameters(robotParam):
    # Axis (8, 9), (12, 13), (18, 19), (22, 23), 
    # we should use a * b = 0 for each group. 
    # Which means, take (8, 9) for example. 
    # When axis 8 has value, we should make sure axis 9 is set to 0. 
    if robotParam[8-1] * robotParam[9-1] != 0:
        robotParam[np.random.choice([8-1, 9-1])] = 0
    if robotParam[12-1] * robotParam[13-1] != 0:
        robotParam[np.random.choice([12-1, 13-1])] = 0
    if robotParam[18-1] * robotParam[19-1] != 0:
        robotParam[np.random.choice([18-1, 19-1])] = 0
    if robotParam[22-1] * robotParam[23-1] != 0:
        robotParam[np.random.choice([22-1, 23-1])] = 0
        
    assert robotParam[8-1] * robotParam[9-1] == 0
    assert robotParam[12-1] * robotParam[13-1] == 0
    assert robotParam[18-1] * robotParam[19-1] == 0
    assert robotParam[22-1] * robotParam[23-1] == 0
    return robotParam

def target_function(**kwargs):
    """Pyfeat evaluation object
    
    Target：
        Maxmize the result of Pyfeat
    """

    # Get robot parameters
    neutral = [86, 86, 128, 128, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, 122]
    fixedrobotcode = copy.copy(neutral)
    
    # dict = {}
    for k,v in kwargs.items():
        # print(k,v)
        fixedrobotcode[int(k[1:])-1] = round(v)

    # check parameters
    fixedrobotcode = checkParameters(fixedrobotcode)
    
    # control robot 
    # print(fixedrobotcode)
    # RCSystem.switch_to_customizedPose(fixedrobotcode)

    # Take photo


    # print(fixedrobotcode)

    # py_feat_result = py_feat_analysis(img=kwargs["imgname"], target_emotion="Anger")
    # assert py_feat_result >= 0 and py_feat_result <= 1
    # return py_feat_result
    return np.random.rand()

def optimize_pyfeat():
    # def pyfeat_object(py_feat_result):
    #     """rfc_cv的二次封装，保证：
    #         1.n_estimators和min_samples_split为整数
    #         2.避免max_features在(0, 1)范围之外
    #     """

    optimizer = BayesianOptimization(
        f=target_function,
        # Define HyperParameter Space
        # Anger 
        pbounds={
            "x1": (0, 255), "x2": (0, 255),
            # "x3": (0, 255), 
            # "x4": (0, 255), 
            # "x5": (0, 255), 
            # "x6": (0, 255), "x7": (0, 255), 
            "x8": (0, 255), "x9": (0, 255), 
            # "x10": (0, 255),
            "x11": (0, 255), 
            # "x12": (0, 255), "x13": (0, 255), "x14": (0, 255), 
            "x15": (0, 255), 
            # "x16": (0, 255), "x17": (0, 255), "x18": (0, 255), "x19": (0, 255), "x20": (0, 255), 
            # "x21": (0, 255), "x22": (0, 255), "x23": (0, 255), "x24": (0, 255), "x25": (0, 255), 
            # "x26": (0, 255), "x27": (0, 255), "x28": (0, 255), "x29": (0, 255), "x30": (0, 255), 
            # "x31": (0, 255), "x32": (0, 255), "x33": (0, 255), "x34": (0, 255), "x35": (0, 255)
        },
        random_state=42,
        verbose=2)
    # 2个初始化点和10轮优化，共12轮
    optimizer.maximize(init_points=2, n_iter=2)

    print("Final result:", optimizer.max)
    return optimizer

def main():
    # pbounds = {}
    # for i in range(1, 36):
    #     pbounds[i]=(0,255)

    # print("pbounds", pbounds)

    # target_function(x1=1, x10=3, x2=4, x3=2.8, x4=12.56)
    optimize_pyfeat()

main()