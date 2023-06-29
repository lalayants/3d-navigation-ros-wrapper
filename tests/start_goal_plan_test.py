import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import tf
def talker():
    robot_x, robot_y, robot_z = 0, 2.1, 0
    goal_x, goal_y, goal_z = 3, 1, 0
    print('from: ', robot_x, robot_y, robot_z)
    print('to: ', goal_x, goal_y, goal_z)
    pub = rospy.Publisher('/goal_pose_sub_topic_name', geometry_msgs.msg.Point, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        new_m = geometry_msgs.msg.Point(goal_x, goal_y, goal_z)
        br.sendTransform((robot_x, robot_y, robot_z),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'robot_frame', 'map')
        br.sendTransform((goal_x, goal_y, goal_z),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'goal_frame', 'map')
        pub.publish(new_m)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
