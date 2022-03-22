from image_geometry import PinholeCameraModel
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointField, CameraInfo
import numpy as np
import tf
import sensor_msgs.point_cloud2
import message_filters  # to synchronize topic subscription
import rospy
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs.point_cloud2 import read_points
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import pandas as pd
import os

cameraModel = PinholeCameraModel()

pub_image = {}
pub_pc, pub_pc_tag = {}, {}
pub_to_apriltag_image = {}
sub_to_apriltag_meg = {}
detected_tags_list = []  # all coordinates of detected tags so far
# pub_pose = rospy.Publisher('lidar_pose', Pose, queue_size=10)

isRotMatSet = False
# rotationMatrix_lidar_camera = np.array([[0.00561514, -0.999907, -0.0124428,-0.0171173],
#                  [0.0304767, 0.0126084, -0.999456, -0.0587173],
#                  [0.99952, 0.00523287, 0.0305447, -0.0324206],
#                  [0, 0, 0, 1]])
rotationMatrix_lidar_camera = np.array([[-0.0443173, -0.998888, -0.0160588, 0.0677557],
                                        [0.0297446, 0.0147482, -0.999449, -0.019818],
                                        [0.998575, -0.0447705, 0.0290579, 0.24684],
                                        [0, 0, 0, 1]])
cv_image = []
bridge = {}
saving_csv_index_i = 0  # used for csv file saving name
while ('pc' + str(saving_csv_index_i) + '_tags_location.csv') in os.listdir():
    saving_csv_index_i += 1
print('csv saving file is ready.')


def april_tag_ouput_decoder(april_tag_list_msg):
    ans = []
    april_tag_list = list(april_tag_list_msg.data)
    # print('april_tag_list', april_tag_list, type(april_tag_list), len(april_tag_list))
    for i in range(0, len(april_tag_list), 4):
        ans.append(april_tag_list[i:i + 4])  # ans is [[id,x,y,z]*n]
    return ans


def create_pc_fields():
    fields = []
    fields.append(PointField('x', 0, PointField.FLOAT32, 1))
    fields.append(PointField('y', 4, PointField.FLOAT32, 1))
    fields.append(PointField('z', 8, PointField.FLOAT32, 1))
    fields.append(PointField('intensity', 12, PointField.FLOAT32, 1))
    fields.append(PointField('r', 16, PointField.FLOAT32, 1))
    fields.append(PointField('g', 20, PointField.FLOAT32, 1))
    fields.append(PointField('b', 24, PointField.FLOAT32, 1))
    return fields


def RGBD_callback(image_data, pointCloud_data):
    global cv_image
    global bridge
    global cameraModel
    global isRotMatSet
    global rotationMatrix
    global pub_image
    global pub_pc
    global pub_pc_tag
    global transformMatrix
    global detected_tags_list
    tag_centers_points, new_points = [], []
    # timestr_image = "%.6f" % image_data.header.stamp.to_sec()
    # print(timestr_image)
    # timestr_point = "%.6f" % pointCloud_data.header.stamp.to_sec()
    # print(timestr_point)
    # print("new frame received.")

    try:
        cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")
        width, height = cv_image.shape[:2]
        # print "cv_image w h = "+str(width) +", "+ str(height)
    except CvBridgeError as e:
        print(e)

    if (isRotMatSet):
        # get location of each April_tag each time
        pub_to_apriltag_image.publish(image_data)
        april_tag_msg = rospy.wait_for_message("/pub_tag_msg", Float64MultiArray)
        april_tag_infos_one_frame = april_tag_ouput_decoder(april_tag_msg)  # info of tags in one frame
        # print(april_tag_infos_one_frame)

        # translate the coordinate
        for i in range(len(april_tag_infos_one_frame)):
            mid_coordinate = [april_tag_infos_one_frame[i][1], april_tag_infos_one_frame[i][2],
                              april_tag_infos_one_frame[i][3], 1.0]
            # transformedPoint = rotationMatrix_lidar_camera.dot(transformMatrix.dot(mid_coordinate))

            transformedPoint = np.linalg.inv(transformMatrix).dot(
                np.linalg.inv(rotationMatrix_lidar_camera).dot(mid_coordinate))
            april_tag_infos_one_frame[i] = [april_tag_infos_one_frame[i][0], transformedPoint[0], transformedPoint[1],
                                            transformedPoint[2]]
            tag_centers_points.append([transformedPoint[0], transformedPoint[1], transformedPoint[2], int(april_tag_infos_one_frame[i][0]), 255, 0, 0])

        # add new tag coordinate to the whole tag coordinate array.
        for i in range(len(april_tag_infos_one_frame)):
            detected_tags_list.append(april_tag_infos_one_frame[i])

        cv_temp = []
        cv_temp = cv_image.copy()
        width, height = cv_temp.shape[:2]

        for point in (read_points(pointCloud_data, skip_nans=True)):
            pointXYZ = [point[0], point[1], point[2], 1.0]
            intensity = point[3]
            intensityInt = int(intensity * intensity * intensity)
            transformedPoint = rotationMatrix_lidar_camera.dot(transformMatrix.dot(pointXYZ))
            if transformedPoint[2] < 0:
                continue
            projected_2d_point = cameraModel.project3dToPixel(transformedPoint)
            # projection
            if projected_2d_point[0] >= 10 and projected_2d_point[0] <= height - 10 and projected_2d_point[1] >= 10 and \
                    projected_2d_point[1] <= width - 10:
                cv2.circle(cv_temp, (int(projected_2d_point[0]), int(projected_2d_point[1])), 5,
                           (intensityInt % 255, (intensityInt / 255) % 255, (intensityInt / 255 / 255)), thickness=-1)
                [b, g, r] = cv_image[int(projected_2d_point[1]), int(projected_2d_point[0])]
                new_points.append([point[0], point[1], point[2], intensity, r, g, b])
        try:
            pub_image.publish(bridge.cv2_to_imgmsg(cv_temp, "bgr8"))
            new_pointCloud = sensor_msgs.point_cloud2.create_cloud(pointCloud_data.header, create_pc_fields(),
                                                                   new_points)
            pc_tag_centers = sensor_msgs.point_cloud2.create_cloud(pointCloud_data.header, create_pc_fields(),
                                                                   tag_centers_points)
            pub_pc.publish(new_pointCloud)
            pub_pc_tag.publish(pc_tag_centers)
        except CvBridgeError as e:
            print(e)

    else:
        print('Waiting for pose info from sub_pose')

    # save tag location of the point cloud into csv for matching (and debugging).
    detected_tags = pd.DataFrame(np.array(detected_tags_list).reshape((-1, 4)),
                                 columns=['tag_id', 'tag_x', 'tag_y', 'tag_z'])
    detected_tags.to_csv('pc' + str(saving_csv_index_i) + '_tags_location.csv', index=False)


def poseCallback(data):
    global isRotMatSet
    global rotationMatrix
    global transformMatrix
    pose = data
    # print("lidarToRGB, pose received")
    quaternion = (
        pose.pose.pose.orientation.x,
        pose.pose.pose.orientation.y,
        pose.pose.pose.orientation.z,
        pose.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    translation = [pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z, -1.0]
    rotationMatrix = tf.transformations.euler_matrix(roll, pitch, yaw)
    transformMatrix = rotationMatrix.transpose()
    transformMatrix[:, 3] = -rotationMatrix.transpose().dot(translation)
    # print(transformMatrix)
    isRotMatSet = True


def cameraCallback(data):
    global cameraModel
    cameraModel.fromCameraInfo(data)


def lidarToRGB():
    global pub_image
    global pub_pc
    global pub_pc_tag
    global bridge
    global pub_to_apriltag_image
    global sub_to_apriltag_meg
    rospy.init_node('lidar_to_rgb', anonymous=True)

    bridge = CvBridge()

    sub_pose = rospy.resolve_name('/Odometry')

    # subscribe to camera
    sub_pose = rospy.Subscriber(sub_pose, Odometry, callback=poseCallback, queue_size=1)
    camera = rospy.Subscriber(rospy.resolve_name('/camera/color/camera_info'), CameraInfo, callback=cameraCallback,
                              queue_size=1)

    pub_to_apriltag_image = rospy.Publisher("/input_apriltag_image", Image, queue_size=10)
    pub_image = rospy.Publisher("image_color_with_proj", Image, queue_size=1)
    pub_pc = rospy.Publisher("pointcloud_color", PointCloud2, queue_size=1)
    pub_pc_tag = rospy.Publisher("pointcloud_tags_center", PointCloud2, queue_size=1)

    sub_image = message_filters.Subscriber('/camera/color/image_raw', Image)
    sub_pointcloud = message_filters.Subscriber('/cloud_registered', PointCloud2)

    ts = message_filters.ApproximateTimeSynchronizer([sub_image, sub_pointcloud], 1, 1)
    ts.registerCallback(RGBD_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        lidarToRGB()
    except rospy.ROSInterruptException:
        pass
