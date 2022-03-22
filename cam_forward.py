import rospy
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


def yaml_to_CameraInfo(yaml_fname):
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    # print(camera_info_msg)
    return camera_info_msg


def callback(data):
    global pub_info, pub_image, camera_info_msg, bridge
    # print('forward')

    # raw_input()

    camera_info_msg.header = data.header
    # camera_info_msg.header.stamp.nsecs -= 10
    # camera_info_msg.header.seq = '0'

    '''
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
      cv2.imwrite("test/"+ str(camera_info_msg.header.seq) + ".jpeg",cv_image)
      print('show')
    except CvBridgeError as e:
      print(e)
    '''

    t = int(rospy.rostime.time.time())
    camera_info_msg.header.stamp.secs = t
    data.header.stamp.secs = t
    pub_image.publish(data)
    pub_info.publish(camera_info_msg)
    # print('camera_info_msg.header.stamp.nsecs',camera_info_msg.header.stamp.nsecs)
    print('camera_info_msg.header.seq', camera_info_msg.header.seq)
    # rospy.sleep(0.1)
    # print('.width',camera_info_msg.width)
    # print('.height',camera_info_msg.height)
    # print('data',data.header.stamp.nsecs)
    # print('done')
    # rospy.sleep(0.5)


if __name__ == "__main__":
    # Get fname from command line (cmd line input required)
    import argparse

    global pub_info, pub_image, camera_info_msg, bridge

    bridge = CvBridge()

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("filename", help="Path to yaml file containing " + \
                                             "camera calibration data")
    args = arg_parser.parse_args()
    filename = args.filename

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)

    rospy.sleep(0.5)

    ## ****************** output topics *************************
    pub_info = rospy.Publisher("/usb_cam/camera_info", CameraInfo, queue_size=10)
    pub_image = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=10)

    ## ****************** input topic *************************
    print('start')
    rospy.Subscriber("/input_apriltag_image", Image, callback, queue_size=1)
    # rospy.Subscriber("image_pub", Image, callback, queue_size=1)

    rospy.spin()
