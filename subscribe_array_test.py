import rospy
from std_msgs.msg import String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float64MultiArray


def num_param_callback(msg):
    print('num_palm_extras', msg, msg.data, type(msg.data))
    return msg.data


def callback(msgg):  # msgg的名字随便定义,保持一致即可
    print('msgg', msgg.data, len(msgg.data))


def listener():
    rospy.init_node('listener', anonymous=True)
    #k = num_param_callback(rospy.wait_for_message("/num_palm_extras", Int32))
    mid = rospy.wait_for_message("/my_tag_data", Float64MultiArray)
    print(mid)
    callback(mid)


if __name__ == '__main__':
    listener()
