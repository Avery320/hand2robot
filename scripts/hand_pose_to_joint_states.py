import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import json

def hand_callback(msg):
    hand_joints = json.loads(msg.data)
    angles = hand_to_joint_angles(hand_joints)
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    js.position = angles
    joint_pub.publish(js)

def hand_to_joint_angles(hand_joints):
    # 假設 hand_joints = {'joint_1': [x1, y1, z1], ..., 'joint_6': [x6, y6, z6]}
    # 這裡僅以 x 值做簡單線性映射，實際應根據你的需求調整
    angles = []
    for i in range(6):
        x = hand_joints[f'joint_{i+1}'][0]
        # 將 x (0~1) 映射到關節角度範圍（例如 -1.5 ~ 1.5 弧度）
        angle = (x - 0.5) * 3.0
        angles.append(angle)
    return angles

rospy.init_node('hand_to_joint_states')
joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
rospy.Subscriber('/hand_to_arm', String, hand_callback)
rospy.spin()
