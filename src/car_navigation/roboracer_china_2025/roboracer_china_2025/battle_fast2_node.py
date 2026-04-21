#! /usr/bin/env python3
#coding=utf-8

import copy
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

# === 保持原代码的常量定义不变 ===
DIR_DETECT_THRESHOLD = 2.5 # 方向探测距离,用于方向判断
OBS_DETECT_THRESHOLD = 5.0 # 障碍探测距离,用于障碍物判断

MAX_SPEED_RATE = 2.0

THRESHOLD_obs = 0.5
THRESHOLD_TURN = 0.5 # 转弯阈值
START_ANGLE = -60
END_ANGLE = 60
MIN_OBS_SPEED = 1.0


class BattleVehicleNode(Node):
    def __init__(self):
        super().__init__('wall_following2')

        # ---- 话题参数: scan前端输入、最终控制输出与可视化调试 ----
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('marker_topic', '/arrow_marker_02')
        self.declare_parameter('debug_scan_topic', '/front_scan_02')

        # ---- Phase 3诊断参数: 控制台打印频率与统计发布开关 ----
        self.declare_parameter('diag_print_hz', 2.0)
        self.declare_parameter('publish_diag_topics', True)


        # 读取话题参数（用于ROS接口接线）
        scan_topic = self.get_parameter('scan_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        debug_scan_topic = self.get_parameter('debug_scan_topic').value

        # Phase 3诊断参数读取: 用于实时观测打印与话题统计发布。
        self.diag_print_hz = max(0.2, float(self.get_parameter('diag_print_hz').value))
        self.publish_diag_topics = bool(self.get_parameter('publish_diag_topics').value)


        # ---- 反应式前端状态: 保持与原battle_fast2一致，用于方向选择/避障/Follow/chaoche ----
        # === 原逻辑成员变量 ===
        self.last_angle = 0.0
        self.last_max_dir_index = 0
        self.GO_STARIGHT = 0
        self.TRANSITION = 0
        self.last_in_normol = False
        self.last_in_straight = False
        self.speed_rate = 1.0
        self.straight_cnt = 0
        self.Follow = False
        self.turn_rate = 1.0
        self.P = 1.1
        self.D = 0.2
        self.dynamic_obs = False
        self.chaoche = False

        # 日志缓存: 仅用于状态摘要与调试观测，不参与控制决策。
        self.current_behavior_state = 'NORMAL'
        self.last_logged_state = None
        self.last_cmd_speed = 0.0


        # 激光链路使用BEST_EFFORT，降低高频扫描数据阻塞风险。
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 纯启发式版本只需要 scan 驱动前端与后端。
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.middle_line_callback, qos_profile)

        # 发布最终控制到drive_topic；其余发布器用于调试和可视化。
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.scan_pub = self.create_publisher(LaserScan, debug_scan_topic, 10)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 1)

        # 调试输出: 纯启发式控制命令与最终命令速度。
        self.heuristic_pub = self.create_publisher(AckermannDriveStamped, 'battle_fast2/drive_heuristic', 10)
        self.current_speed_pub = self.create_publisher(Float32, 'battle_fast2/current_speed_mps', 10)


        # 诊断定时器: 周期打印实时速度和统计信息，便于命令行观测。
        self.diag_timer = self.create_timer(1.0 / self.diag_print_hz, self.diagnostic_timer_callback)

    # Phase 3诊断回调: 以固定频率输出统一摘要日志，不在每个控制周期刷屏。
    def diagnostic_timer_callback(self):
        self.log_battle_summary()

        if self.publish_diag_topics:
            speed_msg = Float32()
            speed_msg.data = float(self.last_cmd_speed)
            self.current_speed_pub.publish(speed_msg)

    def get_behavior_state_for_log(self):
        if self.chaoche:
            return 'CHAOCHE'
        if self.Follow:
            return 'FOLLOW'
        return 'NORMAL'

    def format_battle_log(self, state, speed):
        return f"[Battle] state={state} controller=HEURISTIC speed={speed:.2f} m/s"

    def log_battle_summary(self):
        self.get_logger().info(self.format_battle_log(state=self.current_behavior_state, speed=self.last_cmd_speed))

    def publish_arrow_marker(self, max_dir_index, frame_id='laser'):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'direction_arrow'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        p0 = Point(x=0.0, y=0.0, z=0.0)
        angle_rad = math.radians(max_dir_index)
        p1 = Point(x=math.sin(angle_rad), y=math.cos(angle_rad), z=0.0)
        marker.points = [p0, p1]
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.lifetime = Duration(sec=0, nanosec=100000000)
        self.marker_pub.publish(marker)

    def get_dis(self, data, angle, deg=True, return_inten=True):
        if deg:
            angle = np.deg2rad(angle)
        dis = 0
        intensities = None
        temp = int((angle - data.angle_min) / data.angle_increment)

        start_idx = max(0, temp - 2)
        end_idx = min(len(data.ranges), temp + 2)

        data_tmp = data.ranges[start_idx:end_idx]
        inten_tmp = data.intensities[start_idx:end_idx] if len(data.intensities) > 0 else [0] * len(data_tmp)

        data_tmp = np.sort(np.array(data_tmp))
        inten_tmp = np.sort(np.array(inten_tmp))

        if len(data_tmp) > 2:
            dis = data_tmp[2]
            intensities = inten_tmp[2]
        elif len(data_tmp) > 0:
            dis = data_tmp[0]
            intensities = inten_tmp[0]

        if return_inten:
            return dis, intensities
        return dis

    def get_range(self, data, start_angle, end_engle, return_inten=False):
        all_dis = []
        all_inten = []
        for angle in range(start_angle, end_engle):
            tmp = self.get_dis(data, angle, return_inten=return_inten)
            all_dis.append(tmp[0])
            all_inten.append(tmp[1])
        if return_inten:
            return all_dis, all_inten
        return all_dis

    def fill_zeros_with_neighbors(self, data):
        result = list(data)
        n = len(result)

        for i in range(n):
            if result[i] == 0:
                left = next((result[j] for j in range(i - 1, -1, -1) if result[j] != 0), None)
                if left is not None:
                    result[i] = left
                    continue

                right = next((result[j] for j in range(i + 1, n) if result[j] != 0), None)
                if right is not None:
                    result[i] = right
                    continue

                result[i] = 0
        return result

    def filter_obstacles_by_variance(self, Left_obs_orig, dis_90, variance_threshold=1.0):
        Left_obs = []
        if len(Left_obs_orig) > 0:
            for i in range(0, int(len(Left_obs_orig) / 2), 1):
                idx_start = int(Left_obs_orig[2 * i])
                idx_end = int(Left_obs_orig[2 * i + 1])
                obstacle_range = dis_90[idx_start: idx_end]
                dis_obs_var = np.var(obstacle_range)
                Left_obs.append(idx_start)
                Left_obs.append(idx_end)
        return Left_obs

    def filter_anomalous_values(self, data, max_distance=4, angle_range=2):
        data = np.array(data)
        for i in range(1, len(data) - 1):
            if data[i] != max_distance:
                if all(data[j] == max_distance for j in range(i - angle_range, i + angle_range + 1) if 0 <= j < len(data)):
                    data[i] = (data[i - 1] + data[i + 1]) / 2
        return data.tolist()

    def filter_small_obstacles(self, Left_obs, min_obstacle_size=2):
        for i in range(int(len(Left_obs) / 2)):
            if abs(Left_obs[2 * i] - Left_obs[2 * i + 1]) <= min_obstacle_size:
                Left_obs[2 * i] = -1
                Left_obs[2 * i + 1] = -1
        Left_obs_temp = Left_obs
        Left_obs = [x for x in Left_obs_temp if x != -1]
        return Left_obs

    def pub_scan(self, dis_90, header):
        scan_msg = LaserScan()
        scan_msg.header = header
        scan_msg.angle_min = np.pi / 2
        scan_msg.angle_max = -np.pi / 2
        scan_msg.angle_increment = -np.pi / 180
        scan_msg.ranges = [float(x) for x in dis_90]
        scan_msg.intensities = []
        scan_msg.range_max = 100.0
        self.scan_pub.publish(scan_msg)

    def DynamicObastcle(self, dis_list, inten_list, max_dir_num, obs):
        if len(max_dir_num) < 3:
            return False
        max_range = [max_dir_num[0], max_dir_num[-1]]
        obs_range = [max_dir_num[1], max_dir_num[2]]

        range_list = inten_list[max_range[0]:max_range[1]]
        obs_intensity = inten_list[obs_range[0]:obs_range[1]]

        if len(range_list) == 0 or len(obs_intensity) == 0:
            return False

        average_obs_intensity = np.mean(obs_intensity)
        average_intensity = np.mean(range_list)
        abs_tmp = abs(average_obs_intensity - average_intensity)
        if abs_tmp > 5:
            return True
        return False

    def frontend_scan_process(self, data):
        self.dynamic_obs = False
        self.chaoche = False
        self.Follow = False
        self.D = 0.2

        dis_90, inten_90 = self.get_range(data, -89, 91, True)
        dis_90 = dis_90[::-1]
        inten_90 = inten_90[::-1]
        dis_obs_90 = copy.deepcopy(dis_90)
        lenth_dis = len(dis_90)

        left = 0
        right = 0
        Left_obs_orig = []
        Left_obs = []
        max_dis_num = []
        max_dir_num = []
        max_dis = 0
        max_dir_range = 0
        max_dis_index = 0
        max_dir_index = 0

        self.pub_scan(dis_90, data.header)

        dis_90 = self.fill_zeros_with_neighbors(dis_90)
        inten_90 = self.fill_zeros_with_neighbors(inten_90)
        dis_obs_90 = self.fill_zeros_with_neighbors(dis_obs_90)

        dis_90_copy = tuple(dis_90)

        for i in range(0, lenth_dis, 1):
            if dis_90[i] > max_dis and i > 20 and i < 160:
                max_dis = dis_90[i]
                max_dis_index = i
            if dis_90[i] > DIR_DETECT_THRESHOLD:
                dis_90[i] = DIR_DETECT_THRESHOLD
            if dis_obs_90[i] > OBS_DETECT_THRESHOLD:
                dis_obs_90[i] = OBS_DETECT_THRESHOLD

        dis_90 = self.filter_anomalous_values(dis_90, max_distance=DIR_DETECT_THRESHOLD, angle_range=2)
        dis_obs_90 = self.filter_anomalous_values(dis_obs_90, max_distance=OBS_DETECT_THRESHOLD, angle_range=2)

        if max_dis_index < 89:
            left = 1
        else:
            right = 1

        for i in range(0, lenth_dis - 2, 1):
            if dis_obs_90[i] - dis_obs_90[i + 1] > THRESHOLD_obs and len(Left_obs_orig) % 2 == 0:
                Left_obs_orig.append(i + 1)
            elif dis_obs_90[i + 1] - dis_obs_90[i] > THRESHOLD_obs and len(Left_obs_orig) % 2 == 1:
                Left_obs_orig.append(i)

        if len(Left_obs_orig) % 2 == 1:
            Left_obs_orig.pop()

        Left_obs = self.filter_small_obstacles(Left_obs_orig, min_obstacle_size=2)
        Left_obs = self.filter_obstacles_by_variance(Left_obs, dis_obs_90, variance_threshold=1.0)

        if len(Left_obs) > 0:
            for i in range(0, int(len(Left_obs) / 2), 1):
                idx_mid = int((Left_obs[2 * i] + Left_obs[2 * i + 1]) / 2)
                obs_middle = dis_obs_90[idx_mid]

                start_expand = int(max(Left_obs[2 * i] - min((Left_obs[2 * i + 1] - Left_obs[2 * i]) / 2 * (4 - obs_middle), 10), 0))
                end_expand = int(min(Left_obs[2 * i + 1] + min((Left_obs[2 * i + 1] - Left_obs[2 * i]) / 2 * (4 - obs_middle), 10), lenth_dis - 1))

                for j in range(start_expand, end_expand, 1):
                    dis_obs_90[j] = obs_middle

                Left_obs[2 * i] = start_expand
                Left_obs[2 * i + 1] = end_expand
        else:
            pass

        if len(Left_obs) > 0:
            for i in range(0, int(len(Left_obs) / 2) + 1, 1):
                if i == 0:
                    for j in range(Left_obs[0] - 1, 0, -1):
                        if dis_obs_90[j] <= dis_obs_90[Left_obs[0] + 1]:
                            max_dis_num.append(j)
                            max_dis_num.append(Left_obs[0] + 1)
                            break
                    if len(max_dis_num) == 0:
                        for j in range(0, Left_obs[0] - 1, 1):
                            if dis_obs_90[j] >= dis_obs_90[Left_obs[0] + 1]:
                                max_dis_num.append(j)
                                max_dis_num.append(Left_obs[0] + 1)
                                break
                elif i < int(len(Left_obs) / 2):
                    max_dis_num.append(Left_obs[2 * i - 1])
                    max_dis_num.append(Left_obs[2 * i])
                elif i == int(len(Left_obs) / 2):
                    for j in range(Left_obs[2 * i - 1] + 1, lenth_dis - 1, 1):
                        if dis_obs_90[j] <= dis_obs_90[Left_obs[2 * i - 1] - 1]:
                            max_dis_num.append(Left_obs[2 * i - 1] - 1)
                            max_dis_num.append(j)
                            break

            max_dis_val = 0
            max_dis_index_temp = max_dis_index

            for i in range(0, int(len(max_dis_num) / 2), 1):
                if max_dis_val < max_dis_num[2 * i + 1] - max_dis_num[2 * i]:
                    max_dis_val = max_dis_num[2 * i + 1] - max_dis_num[2 * i]
                    max_dis_index = (max_dis_num[2 * i + 1] + max_dis_num[2 * i]) / 2
                if left == 1 and max_dis_index < 90 and dis_obs_90[0] < dis_obs_90[lenth_dis - 1] - 1:
                    max_dis_index = max_dis_index + 5 * abs(dis_obs_90[lenth_dis - 1] - dis_obs_90[0])
                elif right == 1 and max_dis_index > 90 and dis_obs_90[0] - 1 > dis_obs_90[lenth_dis - 1]:
                    max_dis_index = max_dis_index - 5 * abs(dis_obs_90[lenth_dis - 1] - dis_obs_90[0])

            if len(Left_obs) == 2:
                if max_dis_index_temp >= 89:
                    if Left_obs[0] >= 89:
                        max_dis_index = int((89 + Left_obs[0]) / 2)
                    elif Left_obs[1] <= 89:
                        max_dis_index = max_dis_index_temp
                    else:
                        max_dis_index = max_dis_index_temp
                else:
                    if Left_obs[0] >= 89:
                        max_dis_index = max_dis_index_temp
                    elif Left_obs[1] <= 89:
                        max_dis_index = int((89 + Left_obs[1]) / 2)
                    else:
                        max_dis_index = max_dis_index_temp
            if len(Left_obs) == 4:
                middle_temp = int((Left_obs[1] + Left_obs[2]) / 2)
                if max_dis_index_temp >= 89:
                    if Left_obs[0] >= 89:
                        max_dis_index = int((89 + Left_obs[0]) / 2)
                    elif Left_obs[1] <= 89 and middle_temp > 89:
                        max_dis_index = int((Left_obs[1] + max_dis_index_temp) / 2)
                    elif middle_temp <= 89 and Left_obs[2] > 89:
                        max_dis_index = middle_temp
                    else:
                        max_dis_index = max_dis_index_temp
                else:
                    if Left_obs[0] > 89:
                        max_dis_index = max_dis_index_temp
                    elif Left_obs[0] <= 89 and middle_temp > 89:
                        max_dis_index = middle_temp
                    elif middle_temp <= 89 and Left_obs[3] > 89:
                        max_dis_index = middle_temp
                    else:
                        max_dis_index = int((Left_obs[3] + 89) / 2)

        for i in range(0, lenth_dis - 2, 1):
            if dis_90[i] < DIR_DETECT_THRESHOLD and dis_90[i + 1] == DIR_DETECT_THRESHOLD and len(max_dir_num) % 2 == 0:
                max_dir_num.append(i + 1)
            elif dis_90[i] == DIR_DETECT_THRESHOLD and dis_90[i + 1] < DIR_DETECT_THRESHOLD and len(max_dir_num) % 2 == 1:
                max_dir_num.append(i)

        if len(max_dir_num) % 2 == 1 and len(max_dir_num) != 1:
            pass

        if len(max_dir_num) == 1:
            if max_dir_num[0] < 90:
                max_dir_index = int((max_dir_num[0]) / 2)
            elif max_dir_num[0] > 90:
                max_dir_index = int((max_dir_num[0] + lenth_dis - 2) / 2)
            self.GO_STARIGHT = 0

        if len(max_dir_num) == 2:
            max_dir_index = int((max_dir_num[0] + max_dir_num[1]) / 2)
            max_dir_range = max_dir_num[1] - max_dir_num[0]

        if len(max_dir_num) > 2:

            if len(Left_obs) > 0:
                self.dynamic_obs = self.DynamicObastcle(dis_list=dis_90, inten_list=inten_90, max_dir_num=max_dir_num, obs=Left_obs)

            cand_space = []
            cand_dirs = []
            for i in range(0, int(len(max_dir_num) / 2), 1):
                cand_space.append(max_dir_num[2 * (i) + 1] - max_dir_num[2 * (i)])
                cand_dirs.append((max_dir_num[2 * (i) + 1] + max_dir_num[2 * (i)]) / 2)
            cand_dir_id = np.where(np.array(cand_space) > 18)[0]
            if len(cand_dir_id) != 0:
                selected_dirs = np.array(cand_dirs)[cand_dir_id].tolist()
                max_dir_idx = np.argmin(selected_dirs)
                selected_ranges = np.array(cand_space)[cand_dir_id].tolist()
                max_dir_index = selected_dirs[max_dir_idx]
                max_dir_range = selected_ranges[max_dir_idx]
                cand_dir_chaoche_idx = np.where(np.array(cand_space) > 30)[0]
                if self.dynamic_obs:
                    if cand_dir_chaoche_idx.size:
                        self.chaoche = True
                    else:
                        self.Follow = True
            else:
                cand_dir_id = np.argmax(np.array(cand_space))
                max_dir_index = cand_dirs[cand_dir_id]
                max_dir_range = cand_space[cand_dir_id]
                if self.dynamic_obs:
                    if len(max_dir_num) >= 3:
                        max_dir_index = int((max_dir_num[1] + max_dir_num[2]) / 2)
                    max_dir_range = max_dir_num[-1] - max_dir_num[0]
                    self.Follow = True

            if max_dir_index < 90:
                max_dir_index -= 2
            else:
                max_dir_index += 2

        if max_dir_index >= 75 and max_dir_index <= 105:
            mean_straight = np.mean(dis_90_copy[80:100])
            self.GO_STARIGHT = 1
            self.TRANSITION = 0
            if self.last_in_straight and max_dir_range > 20:
                self.speed_rate *= 1.05
                if mean_straight > 11 and len(Left_obs) == 0:
                    limit_rate = 1.8
                elif mean_straight > 8 and len(Left_obs) == 0:
                    limit_rate = 1.5
                elif mean_straight > 7 and len(Left_obs) == 0:
                    limit_rate = 1.3
                else:
                    limit_rate = 1.0
                if self.speed_rate > limit_rate:
                    self.speed_rate = limit_rate

            else:
                self.speed_rate = 1.1

            self.last_in_straight = True
        elif max_dir_index < 75 and max_dir_index > 0:
            self.P = 1.5
            self.speed_rate = 1.0
            self.turn_rate = 0.8
            self.last_in_straight = False
        elif max_dir_index > 105:
            self.P = 1.5
            self.speed_rate = 1.0
            self.turn_rate = 0.8
            self.last_in_straight = False

        normol = 1
        if len(max_dir_num) == 0:
            if self.GO_STARIGHT == 1 or self.TRANSITION == 1:
                for i in range(0, lenth_dis - 2, 1):
                    if dis_90[i + 1] - dis_90[i] > THRESHOLD_TURN:
                        max_dir_index = int((i + 1 + len(dis_90) / 2) / 2)
                        pass
                        self.P = 0.8
                        normol = 0
                    elif dis_90[i] - dis_90[i + 1] > THRESHOLD_TURN:
                        max_dir_index = int((i + len(dis_90) / 2) / 2)
                        pass
                        self.P = 0.8
                        normol = 0
                if normol == 1:
                    max_dir_index = self.last_max_dir_index

                    if self.last_in_normol:
                        self.speed_rate *= 0.9
                        self.turn_rate *= 1.2
                        if self.speed_rate < 0.5:
                            self.speed_rate = 0.5
                        if self.turn_rate > 2.5:
                            self.turn_rate = 2.5
                    else:
                        self.speed_rate = 0.9
                        self.turn_rate = 1.2
                    self.last_in_normol = True
                else:
                    self.speed_rate = 1.0
                    self.turn_rate = 1.0
                    self.last_in_normol = False

                self.TRANSITION = 1
                self.GO_STARIGHT = 0

        dis_90[0] = dis_90[0] + 0.00001
        dis_90[lenth_dis - 1] = dis_90[lenth_dis - 1] + 0.00001

        return {
            'dis_90': dis_90,
            'lenth_dis': lenth_dis,
            'max_dis': max_dis,
            'max_dir_index': max_dir_index,
            'max_dir_num': max_dir_num,
            'left_obs': Left_obs,
            'header': data.header,
        }

    def heuristic_backend_control(self, front_state):
        dis_90 = front_state['dis_90']
        lenth_dis = front_state['lenth_dis']
        max_dis = front_state['max_dis']
        max_dir_index = front_state['max_dir_index']


        angle = 0.0
        if max_dir_index != 0:
            term1 = -max(math.exp(-max_dis / DIR_DETECT_THRESHOLD), 0.7) * (max_dir_index - 90) / 360 * math.pi
            term2 = (dis_90[0] - dis_90[lenth_dis - 1]) / (dis_90[0] + dis_90[lenth_dis - 1])
            if dis_90[0] / dis_90[lenth_dis - 1] > 3 or dis_90[lenth_dis - 1] / dis_90[0] > 3:
                self.D = 0.5
                angle = 1.0 * term1 + 0.05 * term2
            else:
                angle = 1.0 * term1 + 0.02 * term2
        else:
            if dis_90[0] / dis_90[lenth_dis - 1] > 3 or dis_90[lenth_dis - 1] / dis_90[0] > 3:
                angle = -max(math.exp(-max_dis / DIR_DETECT_THRESHOLD), 0.7) * (max_dir_index - 90) / 360 * math.pi + 0.1 * (dis_90[0] - dis_90[lenth_dis - 1]) / (dis_90[0] + dis_90[lenth_dis - 1])
            else:
                angle = -max(math.exp(-max_dis / DIR_DETECT_THRESHOLD), 0.7) * (max_dir_index - 90) / 360 * math.pi + 0.05 * (dis_90[0] - dis_90[lenth_dis - 1]) / (dis_90[0] + dis_90[lenth_dis - 1])

        steering_angle = self.P * angle + self.D * (angle - self.last_angle)
        self.last_angle = angle

        speed = 1.8 * (0.3 * math.exp(-np.clip(abs(angle), 0, 0.5)) + 0.7)

        steering_angle = self.turn_rate * steering_angle
        steering_angle = np.clip(steering_angle, -math.pi / 4, math.pi / 4)

        return {
            'angle': angle,
            'steering': float(steering_angle),
            'speed': float(self.speed_rate * speed),
            'base_speed': float(speed),
        }

    def control_select_and_publish(self, data, current_frame, front_state, heuristic):
        drive_msg = AckermannDriveStamped()
        drive_msg.header = data.header

        steer_cmd = heuristic['steering']
        speed_cmd = heuristic['speed']

        # 保留启发式 Follow 语义，但调整速度限制以适应更激进的跟车行为。
        if self.Follow:
            speed_cmd = float(min(MIN_OBS_SPEED, speed_cmd))

        drive_msg.drive.steering_angle = float(np.clip(steer_cmd, -math.pi / 4, math.pi / 4))
        drive_msg.drive.speed = float(max(0.0, speed_cmd))

        self.publish_arrow_marker(front_state['max_dir_index'], current_frame)
        self.last_max_dir_index = front_state['max_dir_index']

        heuristic_msg = AckermannDriveStamped()
        heuristic_msg.header = data.header
        heuristic_msg.drive.steering_angle = drive_msg.drive.steering_angle
        heuristic_msg.drive.speed = drive_msg.drive.speed

        self.heuristic_pub.publish(heuristic_msg)

        self.current_behavior_state = self.get_behavior_state_for_log()
        self.last_cmd_speed = float(drive_msg.drive.speed)

        if self.current_behavior_state != self.last_logged_state:
            self.get_logger().info(self.format_battle_log(state=self.current_behavior_state, speed=self.last_cmd_speed))
            self.last_logged_state = self.current_behavior_state

        self.drive_pub.publish(drive_msg)

    def middle_line_callback(self, data):
        clean_ranges = np.nan_to_num(np.array(data.ranges), posinf=0.0, neginf=0.0)
        data.ranges = clean_ranges.tolist()

        current_frame = data.header.frame_id if data.header.frame_id else 'laser'

        front_state = self.frontend_scan_process(data)
        heuristic = self.heuristic_backend_control(front_state)
        self.control_select_and_publish(data, current_frame, front_state, heuristic)


def main(args=None):
    rclpy.init(args=args)
    battle_node = BattleVehicleNode()
    try:
        rclpy.spin(battle_node)
    except KeyboardInterrupt:
        pass
    finally:
        battle_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
