# use localhost:10088
import os, glob, json, cv2, torch
import numpy as np
from torchvision.transforms import transforms
from torch.hub import load_state_dict_from_url
import traceback
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image
from SiameseRankNet import *
import pandas as pd
class IntensityNet_type1(nn.Module):
    def __init__(self, model_path, facebox=None):
        super(IntensityNet_type1, self).__init__()
        # Load ResMaskNet model
        self.model = resmasking_dropout1(in_channels=3, num_classes=7)
        self.model_cls = resmasking_dropout1(in_channels=3, num_classes=7)

        self.model.fc = nn.Sequential(
            nn.Dropout(0.3),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, 7)
        )

        # softmax
        self.model = nn.Sequential(
            self.model,
            nn.Softmax(dim=1)
        )

        self.model_cls = nn.Sequential(
            self.model_cls,
            nn.Softmax(dim=1)
        )
        
        self.use_gpu = torch.cuda.is_available()
        self.image_size = 224
        
        # Define the fully connected layers on top of concatenated feature vectors
        
        self.FER_2013_EMO_DICT = {
            0: "angry",
            1: "disgust",
            2: "fear",
            3: "happy",
            4: "sad",
            5: "surprise",
            6: "neutral",
        }
        self.FER_2013_EMONUM = {v:k for k, v in self.FER_2013_EMO_DICT.items()}
        # self.emotion = CURR_EMO
        # self.idx = self.FER_2013_EMONUM[self.emotion]
        
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        ])
        
        self.sigmoid = nn.Sigmoid()
        # self.activation = nn.Tanh()
        # self.dropout = nn.Dropout(p=0.5)
        # self.relu = nn.ReLU()
        
        # for inference
        self.model_path = model_path
        if self.use_gpu:
            self.state_dict = torch.load(self.model_path)
            for key in list(self.state_dict.keys()):
                self.state_dict[key.replace("module.model.","")] = self.state_dict.pop(key)
            self.model.load_state_dict(self.state_dict, strict=False)
            self.model.cuda()

        self.facebox = facebox
        
    
    # _once
    def forward_once(self, x):
        # Forward pass through ResMaskNet
        y = self.model_cls(x)
        y = y.view(y.size()[0], -1)
        
        x = self.model(x)
        x = x.view(x.size()[0], -1)
        

        alpha = 0.2

        return y * (1 - alpha) + x * alpha
    
    def forward(self, x):
        # Pass each input image through ResMaskNet to obtain feature vectors
        x = self.forward_once(x)

        # x1 = self.forward_once(x[0])
        # print('x1:', x1)
        # x = self.sigmoid(x1)
        # print('x', x)
        return x
    
    def detect_emo(self, frame, detected_face=""):
        """Detect emotions.

        Args:
            frame ([type]): [description]

        Returns:
            List of predicted emotions in probability: [angry, disgust, fear, happy, sad, surprise, neutral]
        """

        with torch.no_grad():
            start_x, start_y, end_x, end_y = get_box(self.facebox[0], self.facebox[1], self.facebox[2], self.facebox[3])
            face = frame.crop([start_x, start_y, end_x, end_y])
            if self.use_gpu:
                face = self.transform(face).cuda()
            else:
                face = self.transform(face)
            face = torch.unsqueeze(face, dim=0)
            output = torch.squeeze(self.model(face), 0)

            return output

def get_box(start_x, start_y, end_x, end_y):
    center_x, center_y = (start_x + end_x) // 2, (start_y + end_y) // 2
    square_length = ((end_x - start_x) + (end_y - start_y)) // 2 // 2
    square_length *= 1.1
    start_x = int(center_x - square_length)
    start_y = int(center_y - square_length)
    end_x = int(center_x + square_length)
    end_y = int(center_y + square_length)
    return start_x, start_y, end_x, end_y


def get_target(emotion_name):
    # Anger, Disgust, Fear, Happiness, Sadness, Surprise, Neutral
    # or lowercase
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
    elif emotion_name in ["Neutral", "neutral"]:
        return 6


def intensityNet_analysis(img, target_emotion, facebox=None, is_save_csv=True):
    # Load the model
    # ['anger', 'disgust', 'fear', 'happiness', 'sadness', 'surprise']
    print("target_emotion:", target_emotion)
    if target_emotion.lower() in ["anger", 'angry']:
        model_path = "new_models/angry_fold3_epoch7.pt"
    elif target_emotion.lower() in ["disgust"]:
        model_path = "new_models/disgust_fold3_epoch7.pt"
    elif target_emotion.lower() in ["fear"]:
        model_path = "new_models/fear_fold3_epoch6.pt"
    elif target_emotion.lower() in ["happiness", "happy"]:
        model_path = "new_models/happy_fold2_epoch7.pt"
    elif target_emotion.lower() in ["sadness", "sad"]:
        model_path = "new_models/sad_fold2_epoch6.pt"
    elif target_emotion.lower() in ["surprise"]:
        model_path = "new_models/surprise_fold3_epoch5.pt"
    else:
        model_path = ""

    assert facebox is not None, "facebox is None"
    model = IntensityNet_type1(model_path, facebox)

    # use model to detect emo
    detection_res = model.detect_emo(Image.open(img))
    detection_res = detection_res.tolist()
    output = detection_res[get_target(target_emotion)]

    # create a pd dataframe
    detection_res = pd.DataFrame([detection_res], columns=["angry", "disgust", "fear", "happy", "sad", "surprise", "neutral"])
    # Create DataFrame

    if is_save_csv:
        csv_emotion_name = img[:-4]+"_intensitynet.csv"
        detection_res.to_csv(csv_emotion_name)

    # result = tmp_res[target_emotion]
    return output