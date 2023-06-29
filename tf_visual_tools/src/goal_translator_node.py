#!/usr/bin/python3

import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import tf
import sys
import yaml
print("!!!!!!!!!!!!!!!!",sys.argv[1])    
if not 'name' in sys.argv[1]:
    with open(sys.argv[1], 'r') as file:
        print('!!!!!!!!!!!!!!!!yaml')
        cfg = yaml.safe_load(file)
        pub_topic = cfg['goal_pose_sub_topic_name']
        map_frame = cfg['map_frame']
        robot_frame = cfg['robot_frame']
        goal_frame = cfg['goal_frame']  
else:
    pub_topic = 'goal_pose_sub_topic_name'
    map_frame = 'map'
    robot_frame = 'robot_frame'
    goal_frame = 'goal_frame'
    
def talker():
    pub = rospy.Publisher(pub_topic, geometry_msgs.msg.Point, queue_size=10)
    rospy.init_node('goal_translator', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    tf_listener = tf.TransformListener()
    prev_transform = [-1000, -1000, -1000]
    while not rospy.is_shutdown():
        try:
            trans, rot = tf_listener.lookupTransform(map_frame, goal_frame, rospy.Time(0))
            if trans[0] != prev_transform[0] or trans[1] != prev_transform[1] or trans[2] != prev_transform[2]:
                new_m = geometry_msgs.msg.Point(*trans)
                pub.publish(new_m)
                prev_transform = trans
                print(prev_transform)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
