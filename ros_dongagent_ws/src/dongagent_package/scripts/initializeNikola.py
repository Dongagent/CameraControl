#!/usr/bin/env python

# import RCSystem
# import defaultPose
# rb = RCSystem.robot(duration=2, webcam=False)
# rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
# rb.connect_ros(True, False)

# if rb.webcam:
#     rb.webcam_stream_widget.stop()

# rb.switch_to_customizedPose(defaultPose.prototypeFacialExpressions['happiness'])
# rb.connect_ros(True, False)
import rospy, json
from std_msgs.msg import String
import base64, struct


def sub_callback(data):
    recv_dict = json.loads(data.data)
    # print(recv_dict)
    if recv_dict['Message'] == "PotentioValsBase64":
        potval_bin = base64.b64decode(recv_dict['ValsBase64'])
        potval = struct.unpack('%sB' % len(potval_bin), potval_bin)
        potentio = list(potval)
        print(potentio)

    if recv_dict['Message'] == "PotentioVals":
        potvalstr = recv_dict['Vals'].split(',')
        potentio = map((lambda x: int(x)), potvalstr)
        print(potentio)

    if recv_dict['Message'] == "PotentioAxes":
        potaxisstr = recv_dict['Axes'].split(',')
        axiswithpotentio = map((lambda x: int(x)), potaxisstr)
        print(axiswithpotentio)

def list2string(data):
    strtmp = ""
    for val in data:
        strtmp += str(val) + ","
    retstr = strtmp.rstrip(',')
    return retstr

def ros_talker(target):
    rospy.init_node('rcpublisher', anonymous=True, disable_signals=True)
    pub = rospy.Publisher('rc/command', String, queue_size=10)
    sub = rospy.Subscriber('rc/return', String, sub_callback)
    r = rospy.Rate(10) # speed

    # if you wan to use MoveAllAxes
    dictdata = {
        "Command": "MoveAllAxes",
        "Vals": list2string(target)
    }
    
    if not rospy.is_shutdown():
        strdata = json.dumps(dictdata)
        # print("strdata", strdata)
        pub.publish(strdata)

headYaw_fix = 105
stableState = [
            64, 64, 128, 128, 128, 
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 32, 128, 128, headYaw_fix
]
robotParams = {}
for i in range(1, 36):
    robotParams["x{}".format(i)] = stableState[i - 1]
ros_talker(robotParams)