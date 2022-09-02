#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

#The TF listener, that will listen to the TF and publish its contents in a topic
rospy.loginfo("TF Listener initialized!")

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    #Variables to initiate the listener and publisher
    listener = tf.TransformListener() #TF Listener
    position = PoseWithCovarianceStamped() #Position variable
    pub_pose = rospy.Publisher('move_base_pose',PoseWithCovarianceStamped,queue_size=1) #Publisher

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try: 
            #The values variable listens to what the map to base_link transform is reading
            values = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            #The according values will be transformed into the position variable, 
            # in a PoseWithCovarianceStamped() type message
            position.header.seq = 0
            position.header.stamp = rospy.Time.now()
            position.header.frame_id = "map"
            
            position.pose.pose.position.x = values[0][0]
            position.pose.pose.position.y = values[0][1]
            position.pose.pose.position.z = values[0][2]
            position.pose.pose.orientation.x = values[1][0]
            position.pose.pose.orientation.y = values[1][1]
            position.pose.pose.orientation.z = values[1][2]
            position.pose.pose.orientation.w = values[1][3]

            #The final message is published to the right topic
            pub_pose.publish(position)

        #Exception in case it's not able to read the TF
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        
        rate.sleep()