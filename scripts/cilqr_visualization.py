import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D
from matplotlib.animation import FuncAnimation

# 加载C++输出的CSV文件
optimal_states = np.loadtxt('optimal_states.csv', delimiter=',')

# def visualize(xs):
#     fig, ax = plt.subplots(figsize=(12, 6))

#     ax.set_xlim(0, 30)
#     ax.set_ylim(-3.1, 3.1)
#     ax.set_aspect('equal')
#     ax.axis("off")

#     for boundary_y in [-3, 3]:
#         ax.plot([0, 30], [boundary_y, boundary_y], 'k-', linewidth=1.0)

#     for lane_y in [0, 1.5]:
#         ax.plot([0, 30], [lane_y, lane_y], 'k--', linewidth=1.0)

#     ego_length = 2.0
#     ego_width = 1.0
#     other_length = 2.0
#     other_width = 1.0

#     ego_rect = patches.Rectangle((0, 0), ego_length, ego_width, fc='r', ec='r', alpha=0.5)
#     other_rect = patches.Rectangle((0, 0), other_length, other_width, fc='g', ec='g', alpha=0.5)
#     ax.add_patch(ego_rect)
#     ax.add_patch(other_rect)

#     ego_trajectory, = ax.plot([], [], 'r-', label='Ego vehicle trajectory')
#     other_trajectory, = ax.plot([], [], 'g-', label='Other vehicle trajectory')

#     def init():
#         ego_rect.set_xy((xs[0, 0] - ego_length / 2, xs[0, 1] - ego_width / 2))
#         ego_rect.angle = np.degrees(xs[0, 2])
#         other_rect.set_xy((xs[0, 5] - other_length / 2, xs[0, 6] - other_width / 2))
#         other_rect.angle = np.degrees(xs[0, 7])
#         ego_trajectory.set_data([], [])
#         other_trajectory.set_data([], [])
#         return ego_rect, other_rect, ego_trajectory, other_trajectory

#     def update(frame):
#         ego_center_x = xs[frame, 0]
#         ego_center_y = xs[frame, 1]
#         ego_angle = np.degrees(xs[frame, 2])

#         other_center_x = xs[frame, 5]
#         other_center_y = xs[frame, 6]
#         other_angle = np.degrees(xs[frame, 7])

#         ego_transform = Affine2D().rotate_deg_around(ego_center_x, ego_center_y, ego_angle) + ax.transData
#         ego_rect.set_transform(ego_transform)
#         ego_rect.set_xy((ego_center_x - ego_length / 2, ego_center_y - ego_width / 2))

#         other_transform = Affine2D().rotate_deg_around(other_center_x, other_center_y, other_angle) + ax.transData
#         other_rect.set_transform(other_transform)
#         other_rect.set_xy((other_center_x - other_length / 2, other_center_y - other_width / 2))

#         ego_trajectory.set_data(xs[:frame+1, 0], xs[:frame+1, 1])
#         other_trajectory.set_data(xs[:frame+1, 5], xs[:frame+1, 6])
#         return ego_rect, other_rect, ego_trajectory, other_trajectory

#     ani = FuncAnimation(fig, update, frames=range(len(xs)), init_func=init, blit=True, interval=50)

#     plt.xlabel('X position')
#     plt.ylabel('Y position')
#     plt.title('Vehicle Overtaking Visualization with Trajectories')
#     plt.legend()
#     plt.grid(True)
#     plt.show()

# # 可视化最优状态
# visualize(optimal_states)

def visualize2others(xs):
    fig, ax = plt.subplots(figsize=(12, 6))

    ax.set_xlim(0, 30)
    ax.set_ylim(-3.1, 3.1)
    ax.set_aspect('equal')
    ax.axis("off")

    # 画边界和车道线
    for boundary_y in [-3, 3]:
        ax.plot([0, 30], [boundary_y, boundary_y], 'k-', linewidth=1.0)

    for lane_y in [0, 1.5]:
        ax.plot([0, 30], [lane_y, lane_y], 'k--', linewidth=1.0)

    # 定义车辆尺寸
    ego_length = 2.0
    ego_width = 1.0
    other_length = 2.0
    other_width = 1.0

    # 创建车辆的矩形表示
    ego_rect = patches.Rectangle((0, 0), ego_length, ego_width, fc='r', ec='r', alpha=0.5)
    other_rect_1 = patches.Rectangle((0, 0), other_length, other_width, fc='g', ec='g', alpha=0.5)
    other_rect_2 = patches.Rectangle((0, 0), other_length, other_width, fc='b', ec='b', alpha=0.5)
    
    ax.add_patch(ego_rect)
    ax.add_patch(other_rect_1)
    ax.add_patch(other_rect_2)

    # 轨迹线条
    ego_trajectory, = ax.plot([], [], 'r-', label='Ego vehicle trajectory')
    other_trajectory_1, = ax.plot([], [], 'g-', label='Other vehicle 1 trajectory')
    other_trajectory_2, = ax.plot([], [], 'b-', label='Other vehicle 2 trajectory')

    def init():
        # 初始化位置
        ego_rect.set_xy((xs[0, 0] - ego_length / 2, xs[0, 1] - ego_width / 2))
        ego_rect.angle = np.degrees(xs[0, 2])
        
        other_rect_1.set_xy((xs[0, 5] - other_length / 2, xs[0, 6] - other_width / 2))
        other_rect_1.angle = np.degrees(xs[0, 7])
        
        other_rect_2.set_xy((xs[0, 10] - other_length / 2, xs[0, 11] - other_width / 2))
        other_rect_2.angle = np.degrees(xs[0, 12])
        
        ego_trajectory.set_data([], [])
        other_trajectory_1.set_data([], [])
        other_trajectory_2.set_data([], [])
        
        return ego_rect, other_rect_1, other_rect_2, ego_trajectory, other_trajectory_1, other_trajectory_2

    def update(frame):
        # 更新自车位置
        ego_center_x = xs[frame, 0]
        ego_center_y = xs[frame, 1]
        ego_angle = np.degrees(xs[frame, 2])

        # 更新其他车辆1位置
        other_center_x_1 = xs[frame, 5]
        other_center_y_1 = xs[frame, 6]
        other_angle_1 = np.degrees(xs[frame, 7])

        # 更新其他车辆2位置
        other_center_x_2 = xs[frame, 10]
        other_center_y_2 = xs[frame, 11]
        other_angle_2 = np.degrees(xs[frame, 12])

        # 更新自车矩形
        ego_transform = Affine2D().rotate_deg_around(ego_center_x, ego_center_y, ego_angle) + ax.transData
        ego_rect.set_transform(ego_transform)
        ego_rect.set_xy((ego_center_x - ego_length / 2, ego_center_y - ego_width / 2))

        # 更新其他车辆1矩形
        other_transform_1 = Affine2D().rotate_deg_around(other_center_x_1, other_center_y_1, other_angle_1) + ax.transData
        other_rect_1.set_transform(other_transform_1)
        other_rect_1.set_xy((other_center_x_1 - other_length / 2, other_center_y_1 - other_width / 2))

        # 更新其他车辆2矩形
        other_transform_2 = Affine2D().rotate_deg_around(other_center_x_2, other_center_y_2, other_angle_2) + ax.transData
        other_rect_2.set_transform(other_transform_2)
        other_rect_2.set_xy((other_center_x_2 - other_length / 2, other_center_y_2 - other_width / 2))

        # 更新轨迹
        ego_trajectory.set_data(xs[:frame+1, 0], xs[:frame+1, 1])
        other_trajectory_1.set_data(xs[:frame+1, 5], xs[:frame+1, 6])
        other_trajectory_2.set_data(xs[:frame+1, 10], xs[:frame+1, 11])
        
        return ego_rect, other_rect_1, other_rect_2, ego_trajectory, other_trajectory_1, other_trajectory_2

    # 动画
    ani = FuncAnimation(fig, update, frames=range(len(xs)), init_func=init, blit=True, interval=50)

    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Vehicle Overtaking Visualization with Trajectories')
    plt.legend()
    plt.grid(True)
    plt.show()

# 可视化最优状态
visualize2others(optimal_states)