# PCL Pattern Matching

This repository uses PCL and OpenCV libraries to perform 3D pattern matching on an unorganized pointcloud. Inputs are an unorganized pointcloud and a 3D model of a pattern. Output is the pose of the matched pattern wrt. the pointcloud frame.

## MBZIR dataset

[Link](https://ferhr-my.sharepoint.com/:f:/g/personal/lmarkovic_fer_hr/EpsQbdSsHKxMrks9z5_Y69QBdrwx8bnhJUvFTZeasxTHxA?e=qW8aKf) to the MBZIRC ROS bags containing LiDAR measurements and map building bags. The goal is to detect the a zig-zag wall pattern. Wall pattern used for matching is located in *resources/wall_pattern_upscaled.ply* and presented as follows.

<img src="https://drive.google.com/uc?export=view&id=1MUr--ECJyKQDKcHw3LYQ4-AsB0wlNjL4"
     width="756" heigh="460" />
