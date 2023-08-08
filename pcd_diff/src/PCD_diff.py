import rospy
import open3d as o3d
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

def callback(PointCloud2): #Chouf chi callack kay ste3mlou fiha PointClouds nnit f github t2ekked menha
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", PointCloud2.PointCloud2)
    
def subscriber():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('subscriber', anonymous=True)

    rospy.Subscriber("/lio_sam/mapping/map_global", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()




# Load PointCloud2
pcd = o3d.io.read_point_cloud("")
dists = pcd.compute_point_cloud_distance(PointCloud2)
dists1 = np.asarray(dists)
ind = np.where(dists1 > 0.01)[0]

def subscriber_node():
    rospy.init_node('subscriber_node', anonymous=True)
    rospy.Subscriber('sub_diff_top', String, callback)
    rospy.spin()


def callback(PointCloud2):
    rospy.loginfo(rospy.get_caller_id() + "The pointcloud distance is: %s", PointCloud2)



if __name__ == '__main__':
    try:
        subscriber_node()
    except rospy.ROSInterruptException:
        pass

