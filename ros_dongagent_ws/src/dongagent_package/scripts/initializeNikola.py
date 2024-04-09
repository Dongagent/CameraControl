import RCSystem
import defaultPose
rb = RCSystem.robot(duration=2)
rb.switch_to_customizedPose(rb.AUPose['StandardPose'])
rb.connect_ros(True, False)

# rb.switch_to_customizedPose(defaultPose.prototypeFacialExpressions['happiness'])
# rb.connect_ros(True, False)