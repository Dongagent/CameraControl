# modify by axes

import RCSystem
import defaultPose
import copy
import time

# -----------------------
# 211126
# happiness
emo_name = 'anger'
final_iter = 5
total_iter = 30

# def run_with_axes:

rb = RCSystem.robot(duration=2)
def run_with_axes(target_params):
    target_params = target_params
    neutral = [86, 86, 128, 128, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, 122]
    finalResult = copy.copy(neutral)
    for k,v in target_params["params"].items():
            # print(k,v)
            if "x" in k:
                finalResult[int(k[1:])-1] = round(v)
    # x2 = x1, use one axis for eyes upper lid
    finalResult[1] = finalResult[0]
    # x7 = x6, use one axis for eyes lower lid
    finalResult[6] = finalResult[5]
    # x13 = x9
    finalResult[12] = finalResult[8]
    # x17 = x16
    finalResult[16] = finalResult[15]
    # x22 = x18
    finalResult[21] = finalResult[17]
    print(finalResult)
    finalResult = RCSystem.checkParameters(finalResult)

    
    rb.switch_to_customizedPose(finalResult)
    rb.connect_ros(isSmoothly=True, isRecording=False, steps = 30) # isSmoothly = True ,isRecording = True

run_with_axes({'params': {'x24': 200, 'x25': 128,'x26':128}})
run_with_axes({'params': {'x24': 0}})


