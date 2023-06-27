import rospy
from std_msgs.msg import String
import geometry_msgs.msg
def talker():
    pub = rospy.Publisher('/goal_pose_sub_topic_name', geometry_msgs.msg.Point, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        new_m = geometry_msgs.msg.Point(1,2,3)
        hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pub.publish(new_m)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass