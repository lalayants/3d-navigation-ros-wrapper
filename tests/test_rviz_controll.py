import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import tf
def talker():
    robot_x, robot_y, robot_z = 0, 2.1, 0
    print('from: ', robot_x, robot_y, robot_z)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        br.sendTransform((robot_x, robot_y, robot_z),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'robot_frame', 'map')
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass