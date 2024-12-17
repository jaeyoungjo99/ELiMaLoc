# PCM Matching README
### Package for Point Cloud Map Matching
### Made by Jaeyoung Jo <wodud3743@gmail.com>

## Purpose and Features
### Perform Point Cloud Map Matching
- Point Cloud Deskewing
- Map structuring and nearest point search based on Voxel Hashing
- Supports various ICP methods (P2P, GICP, VGICP, AVGICP)
- Supports initial position setting via Rviz click (/init_pose)

## I/O
- Input
  - Estimated position nav_msgs/Odometry
  - Point cloud sensor_msgs/PointCloud2
- Output
  - Estimated position nav_msgs/Odometry (Position, Covariance)
  - PCM visualization sensor_msgs/PointCloud2

## Files
- `pcm_matching.cpp`: ROS wrapper. Main function and process execution
- `voxel_hash_map.cpp`: Voxel Hashing map structuring and nearest point search. Structure definition
- `registration.cpp`: Perform ICP with various methods and calculate covariance

# Matching Process

### 1. Initial Validation
- Concurrency control through mutex lock
- Validate input point cloud

### 2. Preprocessing Stage
1. LiDAR message time correction
   - Apply configured time delay
   - Convert point cloud based on Ouster/General LiDAR type
2. Distance-based filtering
   - Remove points farther than the configured maximum distance

### 3. Deskewing (Point Cloud Distortion Correction)
- Motion correction using IMU data
- Correct point positions during scan start/end time. Move to scan end time

### 4. Pose Synchronization
- Calculate synchronized pose at scan end time
- Interpolation using IMU and Odometry data
  - If no matching pose, use integration

### 5. Point Cloud Downsampling
- Convert original points to PointStruct format
- Perform voxel-based downsampling

### 6. Perform Registration
- Input:
  - Source points (local)
  - Voxel map
  - Initial pose
  - Previous pose (for radar)
  - dt (for radar)
- Output:
  - Registration result pose
  - Success status
  - Fitness score

### 7. Result Validation and Publishing
1. Check registration success
2. Convert ICP pose data to ego coordinate system
3. Convert result point cloud to world coordinate system
4. Publish the following topics:
   - PCM odometry
   - ICP result point cloud (downsampled)
   - ICP result full point cloud

# ICP Methodology
### P2P (Point to Point)
- Search and associate nearest points in voxel map, then perform ICP

### **GICP (Generalized ICP)**
- In the Init function, search for points within gicp_cov_search_dist around each map point
- Calculate covariance of searched points and assign to each point
- During ICP, search and associate nearest points in voxel map, then perform GICP (Covariance not performed on source)

### VGICP (Voxel GICP)
- In the Init function, pre-calculate covariance representing each voxel in the voxel map
- During ICP, find nearest voxel covariance for each point and perform GICP

### AVGICP (All Voxel GICP)
- In the Init function, pre-calculate covariance representing each voxel in the voxel map
- During ICP, associate each point with all 27 surrounding voxel covariances and perform GICP

# ICP Covariance Calculation

1. Use the inverse of J^t J of Jacobian J during ICP as state covariance
2. Calculate Translation and Rotation separately
   - Translation: Uncertainty in x, y, z directions
   - Rotation: Uncertainty in roll, pitch, yaw angles
3. Normalize remaining elements of the matrix based on the minimum value of the diagonal matrix of covariance
4. Final Covariance Calculation
   - Translation: Multiply normalized matrix by fitness score
   - Rotation: Multiply normalized matrix by pre-configured angle_std