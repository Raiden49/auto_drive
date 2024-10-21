import rospy
from nav_msgs.msg import Odometry
import carla
import math
import tf.transformations as tft

# ROS 节点初始化
rospy.init_node('spawn_vehicle', anonymous=True)

# 连接到 CARLA 服务器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

def check_collision(spawn_point):
    # 获取当前世界
    world = client.get_world()
    
    # 尝试生成一个临时车辆以检测碰撞
    vehicle_bp = world.get_blueprint_library().filter('vehicle.*')[0]  # 获取第一辆可用车辆
    temp_vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    # 检查是否与任何物体发生碰撞
    collision = False
    if temp_vehicle is not None:
        # 在短时间内检查碰撞
        collision = temp_vehicle.is_colliding()
        # 销毁临时车辆
        temp_vehicle.destroy()

    return collision

def find_spawn_point(current_location, yaw, distance, offset):
    # 计算候选生成位置
    spawn_x = current_location.x + distance * math.cos(yaw) + offset[0]
    spawn_y = current_location.y + distance * math.sin(yaw) + offset[1]
    spawn_z = current_location.z  # 保持相同高度
    return carla.Transform(carla.Location(x=spawn_x, y=spawn_y + 4, z=spawn_z), carla.Rotation(yaw=math.degrees(yaw)))
  
def odometry_callback(msg):
    current_location = carla.Location(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    current_rotation = msg.pose.pose.orientation

    # 将四元数转换为欧拉角
    quat = (current_rotation.x, current_rotation.y, current_rotation.z, current_rotation.w)
    euler = tft.euler_from_quaternion(quat)
    yaw = euler[2]  # 获取偏航角

    # 定义生成车辆的距离
    distance = 10.0  # 在前方生成 10 米

    # 候选偏移位置
    offsets = [(0, 0)]  # 前方偏移位置

    for offset in offsets:
        spawn_point = find_spawn_point(current_location, yaw, distance, offset)

        # 碰撞检测
        if not check_collision(spawn_point):
            # 选择车辆蓝图
            blueprint_library = client.get_world().get_blueprint_library()
            vehicle_bp = blueprint_library.filter('vehicle.*')[0]  # 获取第一辆可用车辆

            # 生成车辆
            vehicle = client.get_world().spawn_actor(vehicle_bp, spawn_point)
            rospy.loginfo(f"Spawned vehicle: {vehicle.type_id} at {spawn_point.location}")
            return  # 生成成功，退出循环

    rospy.logwarn("Collision detected at all sampled points! Vehicle not spawned.")

# 订阅车辆里程计主题
rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, odometry_callback)

# 循环保持节点运行
rospy.spin()
