import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Pose
import json
import moveit_commander
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

rospy.init_node('palm_to_target')
moveit_commander.roscpp_initialize([])
group = moveit_commander.MoveGroupCommander("manipulator")  # 根據你的 MoveIt group 名稱調整

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

def palm_callback(msg):
    palm = json.loads(msg.data)
    # 取得 link_6 在 base_link 下的 transform
    try:
        trans = tf_buffer.lookup_transform('base_link', 'link_6', rospy.Time(0), rospy.Duration(1.0))
        # 計算 link_6 前方 0.3m 的點
        p = geometry_msgs.msg.PointStamped()
        p.header.frame_id = 'link_6'
        p.point.x = 0
        p.point.y = 0
        p.point.z = 0.3  # 前方 30 公分
        # 轉換到 base_link
        p_base = tf2_geometry_msgs.do_transform_point(p, trans)
        target_x = p_base.point.x
        target_y = p_base.point.y
        target_z = p_base.point.z
    except Exception as e:
        rospy.logwarn(f"TF transform failed: {e}")
        return

    # 產生目標點
    pt = PointStamped()
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'base_link'
    pt.point.x = target_x
    pt.point.y = target_y
    pt.point.z = target_z
    pub.publish(pt)

    # 產生目標姿態
    pose_target = Pose()
    pose_target.position.x = target_x
    pose_target.position.y = target_y
    pose_target.position.z = target_z
    pose_target.orientation.w = 1.0
    group.set_pose_target(pose_target)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

pub = rospy.Publisher('/target_point', PointStamped, queue_size=10)
rospy.Subscriber('/palm_target', String, palm_callback)
rospy.spin()
