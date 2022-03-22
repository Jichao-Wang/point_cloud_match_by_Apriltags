import rospy
from std_msgs.msg import String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float64MultiArray
import time

pub2 = rospy.Publisher('my_tag_data', Float64MultiArray, queue_size=10)

palmextras = Float64MultiArray()
dim = MultiArrayDimension()
dim.label = "tag_id"
dim.size = 1
palmextras.layout.dim.append(dim)

dim = MultiArrayDimension()
dim.label = "gyrometer"
dim.size = 3
palmextras.layout.dim.append(dim)

palmextras.data = [2, 0.0, 0.0, 0.0]

rospy.init_node('agni_tactile_test_sensors')

r = rospy.Rate(100)  # 100hz

# while not rospy.is_shutdown():
k = 4
time.sleep(3)
for i in range(k):
    pub2.publish(palmextras)
