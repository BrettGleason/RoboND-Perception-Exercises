#!/usr/bin/env python

# Import modules
from pcl_helper import *

# Define functions as required
def vox_downsample(cloud):
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size. Unit = meters
    LEAF_SIZE = 0.01
    
    # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
    
    return cloud_filtered

def passthrough_filter(cloud, filter_axis, axis_min, axis_max):
    # Create a PassThrough filter object
    passthrough = cloud.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object
    passthrough.set_filter_field_name (filter_axis)
    passthrough.set_filter_limits (axis_min, axis_max)

    # Use the filter function to obtain the resultant point cloud
    cloud_filtered = passthrough.filter()

    return cloud_filtered


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    point_cloud = ros_to_pcl(pcl_msg)

    #Voxel Grid Downsampling
    cloud_downsampled = vox_downsample(point_cloud)

    # RANSAC Plane Segmentation
    # Passthrough filtering
    cloud_filtered = passthrough_filter(cloud_downsampled, 'z', 0.6, 1.1)
    
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain a set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # TODO: Convert PCL data to ROS messages

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(pcl_msg)
    pcl_table_pub.publish(pcl_msg)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
