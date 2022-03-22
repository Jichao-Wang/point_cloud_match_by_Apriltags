# point_cloud_match_by_Apriltags
通过bag中的Apriltag来配准两组点云，为点云比较做准备


# How to run
Open 11 terminal windows first. Then:
T1. roscore
T2. ~/python_ws/pointcloud_match$ python3 color4.py camera.yaml
T3. ~/python_ws/pointcloud_match$ /home/jichao/anaconda3/envs/pc/bin/python match.py  # 提取一个bag中全部的Apriltag
T4. rosrun image_transport republish compressed in:=camera/color/image_raw raw out:=camera/color/image_raw  # 将bag中的压缩图片解压
T5. ~/python_ws/pointcloud_match$ python3 cam_forward.py camera.yaml  # Apriltag的相机标定参数
T6. ~/python_ws/pointcloud_match$ python3 apriltag_sub.py  # 转发等待被识别的Apriltag的图像
T7. roslaunch fast_lio mapping_avia.launch  # 启动fast_lio建图
T8. roslaunch apriltag_ros continuous_detection.launch  # 启动apriltag detection
T9. ~/python_ws/pointcloud_match$ python3 pts_generate.py  # 保存上色点云，需要对bag1和bag2分别运行得到2个点云
T10. ~/ros_ws/data$ rosbag play apriltag3.bag.active  # 播放bag
T11. ~/python_ws/pointcloud_match$ /home/jichao/anaconda3/envs/pc/bin/python match_2_pandas_file.py pc1.pts modified_pc1.pts pc0_tags_location.csv pc1_tags_location.csv # 将运行2次T3后得到的2组Apriltag坐标区平均值匹配；并且将由T11录制的pc2转到pc1坐标系下

Ref:
1. https://blog.csdn.net/u012836279/article/details/80203170   求解transform matrix
