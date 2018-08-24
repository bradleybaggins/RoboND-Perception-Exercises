#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    # choose voxel (aka leaf) size
    LEAF_SIZE = .01
    # set the voxel/leaf size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # call .filter() to obtain the downsampled point cloud
    cloud_filtered = vox.filter()
    filename = 'voxel_downsampled.pcd'
    pcl.save(cloud_filtered, filename)
    
    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    # assign axis and range
    filter_axis = 'z'
    passthrough.set_fiter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    # call .filter() to obtain filtered point cloud
    cloud_filtered = passthrough.filter()
    filename = 'pass_through_filtered.pcd'
    pcl.save(cloud_filtered, filename)

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    # set model
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # max distance threshold for consideration
    max_distance = .01
    seg.set_distance_threshold(max_distance)
    # call .segment() to obtain set of inlier indices on d model coefficients
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    filename = 'extracted_inliers.pcd'
    pcl.save(extracted_inliers, filename)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    filename = 'extracted_outliers.pcd'
    pcl.save(extracted_outliers, filename)
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    # set number of neighboring points to analyze
    outlier_filter.set_mean_k(50)
    # set threshold scale factor
    x = 1.0
    # outlier is any point with a mean distance larger than global (mean distance + x*std_dev)
    outlier_filter.set_std_dev_mul_thresh(x)
    # call .filter()
    cloud_filtered = outlier_filter.filter()

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_filtered)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold and minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.001)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(250)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)    
    
    # TODO: Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cluster_cloud)
    pcl_table_pub.publish(ros_cluster_cloud)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud"), pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
    
