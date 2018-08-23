# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')


# Voxel Grid filter
vox = cloud.make_voxel_grid_filter()
# choose voxel (aka leaf) size
LEAF_SIZE = .01
# set the voxel/leaf size
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
# call .filter() to obtain the downsampled point cloud
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)

# PassThrough filter
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

# RANSAC plane segmentation
seg = cloud_filtered.make_segmenter()
# set model
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
#max distance threshold for consideration
max_distance = .01
seg.set_distance_threshold(max_distance)
# call .segment() to obtain set of inlier indices on d model coefficients
inliers, coefficients = seg.segment()

# Extract inliers
extracted_inliers = cloud_filtered.extract(inliers, negative=False)
filename = 'extracted_inliers.pcd'
pcl.save(extracted_inliers, filename)

# Save pcd for table
# pcl.save(cloud, filename)


# Extract outliers
extracted_outliers = cloud_filtered.extract(inliers, negative=True)
filename = 'extracted_outliers.pcd'
pcl.save(extracted_outliers, filename)

# Save pcd for tabletop objects


