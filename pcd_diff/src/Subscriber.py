import rospy 
from std_msgs.msg import String #hadi librairie 
from sensor_msgs.msg import PointCloud2 #t2ekked men hadi wach besse7 - hadak l .msg fellekher mal9itch hedra 3lih mais kay compiler beha

def callback(PointCloud2): #Chouf chi callack kay ste3mlou fiha PointClouds nnit f github t2ekked menha
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", PointCloud2.PointCloud2)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/lio_sam/mapping/map_global", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()