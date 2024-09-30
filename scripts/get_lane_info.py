import carla
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

def get_lane_width(world, location):
    # 获取地图
    map = world.get_map()
    
    # 查找车辆当前位置的车道
    waypoint = map.get_waypoint(location)

    # 获取车道宽度
    lane_width = waypoint.lane_width

    return lane_width

def odom_callback(odom_msg):
    # 获取车辆的位置
    location = odom_msg.pose.pose.position

    # 获取当前车道的宽度
    lane_width = get_lane_width(world, carla.Location(location.x, location.y, location.z))

    # 发布车道宽度
    lane_width_msg = Float32()
    lane_width_msg.data = lane_width
    pub_lane_width.publish(lane_width_msg)

# 连接到 CARLA 服务器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 获取世界对象
world = client.get_world()

# 创建 ROS 节点
rospy.init_node('lane_width_publisher', anonymous=True)
pub_lane_width = rospy.Publisher('/lane/width', Float32, queue_size=10)

# 订阅 odom 主题
rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, odom_callback)

rospy.spin()  # 持续运行
