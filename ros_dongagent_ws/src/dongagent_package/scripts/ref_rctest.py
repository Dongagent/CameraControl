#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import json
import base64
import time
import struct

def list2string(data):
    strtmp = ""
    for val in data:
        strtmp += str(val) + ","
    retstr = strtmp.rstrip(',')
    return retstr

def sub_callback(data):
    recv_dict = json.loads(data.data)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

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

def talker():
    
    pub = rospy.Publisher('rc/command', String, queue_size=10)
    sub = rospy.Subscriber('rc/return', String, sub_callback)
    rospy.init_node('rctest', anonymous=True)
    
    r = rospy.Rate(1) # speed

    target = [64,64,128,128,128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,128,122]

    # if you want to use MoveAllAxesBase64
    target_bin = struct.pack('%sB' % len(target), *target)
    dictdata = {
        "Command": "MoveReadAllAxesBase64",
        "ValsBase64": base64.b64encode(bytes(target_bin)).decode('utf-8')
    }

    # if you wan to use MoveAllAxes
    dictdata2 = {
        "Command": "MoveAllAxes",
        "Vals": list2string(target)
    }

    # example to use MoveAxes
    eyelids_axes = [0, 1]
    eyelids_vals = [64, 64]
    dictdata3 = {
        "Command": "MoveAxes",
        "Axes": list2string(eyelids_axes),
        "Vals": list2string(eyelids_vals)
    }

    eyeflag = 1
    while not rospy.is_shutdown():

        if eyeflag == 1:
            target[0] = 254
            target[1] = 254
            # target_bin = struct.pack('%sB' % len(target), *target)
            # dictdata['ValsBase64'] = base64.b64encode(bytes(target_bin)).decode('utf-8')
            eyelids_vals[0] = 255
            eyelids_vals[1] = 255
            dictdata3['Vals'] = list2string(eyelids_vals)
            eyeflag = 0
        else:
            # target[0] = 64
            # target[1] = 64
            # target_bin = struct.pack('%sB' % len(target), *target)
            # dictdata['ValsBase64'] = base64.b64encode(bytes(target_bin)).decode('utf-8')
            eyelids_vals[0] = 64
            eyelids_vals[1] = 64
            dictdata3['Vals'] = list2string(eyelids_vals)
            eyeflag = 1

        strdata = json.dumps(dictdata2)
        # rospy.loginfo(strdata)

        pub.publish(strdata)

        r.sleep()

        # print(pub.get_num_connections())

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: 
        pass
