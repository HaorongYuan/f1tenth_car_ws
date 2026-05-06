#! /usr/bin/env python3
# coding=utf-8

import copy
import math
from dataclasses import dataclass

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


@dataclass(frozen=True)
class ReactiveRacingConfig:
    scan_topic: str
    drive_topic: str
    marker_topic: str
    debug_scan_topic: str
    heuristic_drive_topic: str
    current_speed_topic: str
    diag_print_hz: float
    publish_diag_topics: bool
    frontend_scan_start_deg: int
    frontend_scan_end_deg: int
    frontend_sample_window_left_points: int
    frontend_sample_window_right_points: int
    frontend_sample_order_index: int
    frontend_max_distance_search_margin: int
    frontend_dir_detect_threshold: float
    frontend_obs_detect_threshold: float
    frontend_anomaly_angle_range: int
    obstacle_edge_threshold: float
    obstacle_small_max_width: int
    obstacle_expand_distance_ref: float
    obstacle_expand_max_points: float
    dynamic_obs_intensity_delta_threshold: float
    selection_side_bias_distance_delta_threshold: float
    selection_side_bias_gain: float
    selection_candidate_gap_min_width: int
    selection_overtake_gap_min_width: int
    selection_direction_bias_offset: int
    control_normal_p_gain: float
    control_turn_p_gain: float
    control_transition_p_gain: float
    control_base_d_gain: float
    control_asymmetry_d_gain: float
    control_distance_ratio_threshold: float
    control_term1_decay_floor: float
    control_term2_gain_balanced: float
    control_term2_gain_unbalanced: float
    control_term2_gain_zero_balanced: float
    control_term2_gain_zero_unbalanced: float
    control_steering_limit_rad: float
    control_follow_speed_cap: float
    speed_model_scale: float
    speed_model_angle_decay_weight: float
    speed_model_base_weight: float
    speed_model_angle_clip: float
    straight_band_half_width: int
    straight_mean_window_half_width: int
    straight_gap_min_width: int
    straight_speed_growth: float
    straight_speed_rate_reset: float
    straight_distance_far: float
    straight_distance_mid: float
    straight_distance_near: float
    straight_speed_limit_far: float
    straight_speed_limit_mid: float
    straight_speed_limit_near: float
    straight_speed_limit_default: float
    turn_state_speed_rate: float
    turn_state_turn_rate: float
    transition_turn_threshold: float
    transition_speed_decay: float
    transition_speed_min: float
    transition_turn_rate_growth: float
    transition_turn_rate_max: float
    transition_speed_reset: float
    transition_turn_rate_reset: float
    transition_normal_speed_reset: float
    transition_normal_turn_rate_reset: float


class ReactiveRacingNode(Node):
    def __init__(self):
        super().__init__("reactive_racing_node")

        self._declare_parameters()
        self.cfg = self._load_parameters()
        self.diag_print_hz = max(0.2, self.cfg.diag_print_hz)
        self.publish_diag_topics = self.cfg.publish_diag_topics

        self.last_angle = 0.0
        self.last_max_dir_index = 0
        self.GO_STARIGHT = 0
        self.TRANSITION = 0
        self.last_in_normol = False
        self.last_in_straight = False
        self.speed_rate = self.cfg.transition_normal_speed_reset
        self.straight_cnt = 0
        self.Follow = False
        self.turn_rate = self.cfg.transition_normal_turn_rate_reset
        self.P = self.cfg.control_normal_p_gain
        self.D = self.cfg.control_base_d_gain
        self.dynamic_obs = False
        self.chaoche = False

        self.current_behavior_state = "NORMAL"
        self.last_logged_state = None
        self.last_cmd_speed = 0.0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.cfg.scan_topic,
            self.middle_line_callback,
            qos_profile,
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.cfg.drive_topic, 1
        )
        self.scan_pub = self.create_publisher(
            LaserScan, self.cfg.debug_scan_topic, 10
        )
        self.marker_pub = self.create_publisher(Marker, self.cfg.marker_topic, 1)
        self.heuristic_pub = self.create_publisher(
            AckermannDriveStamped, self.cfg.heuristic_drive_topic, 10
        )
        self.current_speed_pub = self.create_publisher(
            Float32, self.cfg.current_speed_topic, 10
        )

        self.diag_timer = self.create_timer(
            1.0 / self.diag_print_hz, self.diagnostic_timer_callback
        )

    def _declare_parameters(self):
        parameters = [
            ("scan_topic", "/scan"),
            ("drive_topic", "/drive"),
            ("marker_topic", "/reactive_racing/arrow_marker"),
            ("debug_scan_topic", "/reactive_racing/front_scan"),
            ("heuristic_drive_topic", "reactive_racing/drive_heuristic"),
            ("current_speed_topic", "reactive_racing/current_speed_mps"),
            ("diag_print_hz", 2.0),
            ("publish_diag_topics", True),
            ("frontend_scan_start_deg", -89),
            ("frontend_scan_end_deg", 91),
            ("frontend_sample_window_left_points", 2),
            ("frontend_sample_window_right_points", 2),
            ("frontend_sample_order_index", 2),
            ("frontend_max_distance_search_margin", 20),
            ("frontend_dir_detect_threshold", 2.5),
            ("frontend_obs_detect_threshold", 5.0),
            ("frontend_anomaly_angle_range", 2),
            ("obstacle_edge_threshold", 0.5),
            ("obstacle_small_max_width", 2),
            ("obstacle_expand_distance_ref", 4.0),
            ("obstacle_expand_max_points", 10.0),
            ("dynamic_obs_intensity_delta_threshold", 5.0),
            ("selection_side_bias_distance_delta_threshold", 1.0),
            ("selection_side_bias_gain", 5.0),
            ("selection_candidate_gap_min_width", 18),
            ("selection_overtake_gap_min_width", 30),
            ("selection_direction_bias_offset", 2),
            ("control_normal_p_gain", 1.1),
            ("control_turn_p_gain", 1.5),
            ("control_transition_p_gain", 0.8),
            ("control_base_d_gain", 0.2),
            ("control_asymmetry_d_gain", 0.5),
            ("control_distance_ratio_threshold", 3.0),
            ("control_term1_decay_floor", 0.7),
            ("control_term2_gain_balanced", 0.02),
            ("control_term2_gain_unbalanced", 0.05),
            ("control_term2_gain_zero_balanced", 0.05),
            ("control_term2_gain_zero_unbalanced", 0.1),
            ("control_steering_limit_rad", math.pi / 4),
            ("control_follow_speed_cap", 1.0),
            ("speed_model_scale", 1.8),
            ("speed_model_angle_decay_weight", 0.3),
            ("speed_model_base_weight", 0.7),
            ("speed_model_angle_clip", 0.5),
            ("straight_band_half_width", 15),
            ("straight_mean_window_half_width", 10),
            ("straight_gap_min_width", 20),
            ("straight_speed_growth", 1.05),
            ("straight_speed_rate_reset", 1.1),
            ("straight_distance_far", 11.0),
            ("straight_distance_mid", 8.0),
            ("straight_distance_near", 7.0),
            ("straight_speed_limit_far", 1.8),
            ("straight_speed_limit_mid", 1.5),
            ("straight_speed_limit_near", 1.3),
            ("straight_speed_limit_default", 1.0),
            ("turn_state_speed_rate", 1.0),
            ("turn_state_turn_rate", 0.8),
            ("transition_turn_threshold", 0.5),
            ("transition_speed_decay", 0.9),
            ("transition_speed_min", 0.5),
            ("transition_turn_rate_growth", 1.2),
            ("transition_turn_rate_max", 2.5),
            ("transition_speed_reset", 0.9),
            ("transition_turn_rate_reset", 1.2),
            ("transition_normal_speed_reset", 1.0),
            ("transition_normal_turn_rate_reset", 1.0),
        ]
        for name, default_value in parameters:
            self.declare_parameter(name, default_value)

    def _load_parameters(self):
        return ReactiveRacingConfig(
            scan_topic=str(self.get_parameter("scan_topic").value),
            drive_topic=str(self.get_parameter("drive_topic").value),
            marker_topic=str(self.get_parameter("marker_topic").value),
            debug_scan_topic=str(self.get_parameter("debug_scan_topic").value),
            heuristic_drive_topic=str(
                self.get_parameter("heuristic_drive_topic").value
            ),
            current_speed_topic=str(self.get_parameter("current_speed_topic").value),
            diag_print_hz=float(self.get_parameter("diag_print_hz").value),
            publish_diag_topics=bool(
                self.get_parameter("publish_diag_topics").value
            ),
            frontend_scan_start_deg=int(
                self.get_parameter("frontend_scan_start_deg").value
            ),
            frontend_scan_end_deg=int(
                self.get_parameter("frontend_scan_end_deg").value
            ),
            frontend_sample_window_left_points=int(
                self.get_parameter("frontend_sample_window_left_points").value
            ),
            frontend_sample_window_right_points=int(
                self.get_parameter("frontend_sample_window_right_points").value
            ),
            frontend_sample_order_index=int(
                self.get_parameter("frontend_sample_order_index").value
            ),
            frontend_max_distance_search_margin=int(
                self.get_parameter("frontend_max_distance_search_margin").value
            ),
            frontend_dir_detect_threshold=float(
                self.get_parameter("frontend_dir_detect_threshold").value
            ),
            frontend_obs_detect_threshold=float(
                self.get_parameter("frontend_obs_detect_threshold").value
            ),
            frontend_anomaly_angle_range=int(
                self.get_parameter("frontend_anomaly_angle_range").value
            ),
            obstacle_edge_threshold=float(
                self.get_parameter("obstacle_edge_threshold").value
            ),
            obstacle_small_max_width=int(
                self.get_parameter("obstacle_small_max_width").value
            ),
            obstacle_expand_distance_ref=float(
                self.get_parameter("obstacle_expand_distance_ref").value
            ),
            obstacle_expand_max_points=float(
                self.get_parameter("obstacle_expand_max_points").value
            ),
            dynamic_obs_intensity_delta_threshold=float(
                self.get_parameter(
                    "dynamic_obs_intensity_delta_threshold"
                ).value
            ),
            selection_side_bias_distance_delta_threshold=float(
                self.get_parameter(
                    "selection_side_bias_distance_delta_threshold"
                ).value
            ),
            selection_side_bias_gain=float(
                self.get_parameter("selection_side_bias_gain").value
            ),
            selection_candidate_gap_min_width=int(
                self.get_parameter("selection_candidate_gap_min_width").value
            ),
            selection_overtake_gap_min_width=int(
                self.get_parameter("selection_overtake_gap_min_width").value
            ),
            selection_direction_bias_offset=int(
                self.get_parameter("selection_direction_bias_offset").value
            ),
            control_normal_p_gain=float(
                self.get_parameter("control_normal_p_gain").value
            ),
            control_turn_p_gain=float(
                self.get_parameter("control_turn_p_gain").value
            ),
            control_transition_p_gain=float(
                self.get_parameter("control_transition_p_gain").value
            ),
            control_base_d_gain=float(
                self.get_parameter("control_base_d_gain").value
            ),
            control_asymmetry_d_gain=float(
                self.get_parameter("control_asymmetry_d_gain").value
            ),
            control_distance_ratio_threshold=float(
                self.get_parameter("control_distance_ratio_threshold").value
            ),
            control_term1_decay_floor=float(
                self.get_parameter("control_term1_decay_floor").value
            ),
            control_term2_gain_balanced=float(
                self.get_parameter("control_term2_gain_balanced").value
            ),
            control_term2_gain_unbalanced=float(
                self.get_parameter("control_term2_gain_unbalanced").value
            ),
            control_term2_gain_zero_balanced=float(
                self.get_parameter("control_term2_gain_zero_balanced").value
            ),
            control_term2_gain_zero_unbalanced=float(
                self.get_parameter("control_term2_gain_zero_unbalanced").value
            ),
            control_steering_limit_rad=float(
                self.get_parameter("control_steering_limit_rad").value
            ),
            control_follow_speed_cap=float(
                self.get_parameter("control_follow_speed_cap").value
            ),
            speed_model_scale=float(self.get_parameter("speed_model_scale").value),
            speed_model_angle_decay_weight=float(
                self.get_parameter("speed_model_angle_decay_weight").value
            ),
            speed_model_base_weight=float(
                self.get_parameter("speed_model_base_weight").value
            ),
            speed_model_angle_clip=float(
                self.get_parameter("speed_model_angle_clip").value
            ),
            straight_band_half_width=int(
                self.get_parameter("straight_band_half_width").value
            ),
            straight_mean_window_half_width=int(
                self.get_parameter("straight_mean_window_half_width").value
            ),
            straight_gap_min_width=int(
                self.get_parameter("straight_gap_min_width").value
            ),
            straight_speed_growth=float(
                self.get_parameter("straight_speed_growth").value
            ),
            straight_speed_rate_reset=float(
                self.get_parameter("straight_speed_rate_reset").value
            ),
            straight_distance_far=float(
                self.get_parameter("straight_distance_far").value
            ),
            straight_distance_mid=float(
                self.get_parameter("straight_distance_mid").value
            ),
            straight_distance_near=float(
                self.get_parameter("straight_distance_near").value
            ),
            straight_speed_limit_far=float(
                self.get_parameter("straight_speed_limit_far").value
            ),
            straight_speed_limit_mid=float(
                self.get_parameter("straight_speed_limit_mid").value
            ),
            straight_speed_limit_near=float(
                self.get_parameter("straight_speed_limit_near").value
            ),
            straight_speed_limit_default=float(
                self.get_parameter("straight_speed_limit_default").value
            ),
            turn_state_speed_rate=float(
                self.get_parameter("turn_state_speed_rate").value
            ),
            turn_state_turn_rate=float(
                self.get_parameter("turn_state_turn_rate").value
            ),
            transition_turn_threshold=float(
                self.get_parameter("transition_turn_threshold").value
            ),
            transition_speed_decay=float(
                self.get_parameter("transition_speed_decay").value
            ),
            transition_speed_min=float(
                self.get_parameter("transition_speed_min").value
            ),
            transition_turn_rate_growth=float(
                self.get_parameter("transition_turn_rate_growth").value
            ),
            transition_turn_rate_max=float(
                self.get_parameter("transition_turn_rate_max").value
            ),
            transition_speed_reset=float(
                self.get_parameter("transition_speed_reset").value
            ),
            transition_turn_rate_reset=float(
                self.get_parameter("transition_turn_rate_reset").value
            ),
            transition_normal_speed_reset=float(
                self.get_parameter("transition_normal_speed_reset").value
            ),
            transition_normal_turn_rate_reset=float(
                self.get_parameter("transition_normal_turn_rate_reset").value
            ),
        )

    def diagnostic_timer_callback(self):
        self.log_battle_summary()

        if self.publish_diag_topics:
            speed_msg = Float32()
            speed_msg.data = float(self.last_cmd_speed)
            self.current_speed_pub.publish(speed_msg)

    def get_behavior_state_for_log(self):
        if self.chaoche:
            return "CHAOCHE"
        if self.Follow:
            return "FOLLOW"
        return "NORMAL"

    def format_battle_log(self, state, speed):
        return (
            f"[ReactiveRacing] state={state} controller=HEURISTIC "
            f"speed={speed:.2f} m/s"
        )

    def log_battle_summary(self):
        self.get_logger().info(
            self.format_battle_log(
                state=self.current_behavior_state, speed=self.last_cmd_speed
            )
        )

    def publish_arrow_marker(self, max_dir_index, frame_id="laser"):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "direction_arrow"
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

        temp = int((angle - data.angle_min) / data.angle_increment)
        start_idx = max(0, temp - self.cfg.frontend_sample_window_left_points)
        end_idx = min(len(data.ranges), temp + self.cfg.frontend_sample_window_right_points)

        data_tmp = np.sort(np.array(data.ranges[start_idx:end_idx]))
        inten_tmp = data.intensities[start_idx:end_idx]
        if len(inten_tmp) > 0:
            inten_tmp = np.sort(np.array(inten_tmp))
        else:
            inten_tmp = np.zeros(len(data_tmp))

        dis = 0
        intensities = None
        sample_order_index = self.cfg.frontend_sample_order_index
        if len(data_tmp) > sample_order_index:
            dis = data_tmp[sample_order_index]
            intensities = inten_tmp[sample_order_index]
        elif len(data_tmp) > 0:
            dis = data_tmp[0]
            intensities = inten_tmp[0]

        if return_inten:
            return dis, intensities
        return dis

    def get_range(self, data, start_angle, end_angle, return_inten=False):
        all_dis = []
        all_inten = []
        for angle in range(start_angle, end_angle):
            tmp = self.get_dis(data, angle, return_inten=return_inten)
            if return_inten:
                all_dis.append(tmp[0])
                all_inten.append(tmp[1])
            else:
                all_dis.append(tmp)
        if return_inten:
            return all_dis, all_inten
        return all_dis

    def fill_zeros_with_neighbors(self, data):
        result = list(data)
        for i in range(len(result)):
            if result[i] != 0:
                continue

            left = next(
                (result[j] for j in range(i - 1, -1, -1) if result[j] != 0), None
            )
            if left is not None:
                result[i] = left
                continue

            right = next(
                (result[j] for j in range(i + 1, len(result)) if result[j] != 0),
                None,
            )
            if right is not None:
                result[i] = right
        return result

    def filter_anomalous_values(self, data, max_distance, angle_range):
        data = np.array(data)
        for i in range(1, len(data) - 1):
            if data[i] == max_distance:
                continue
            if all(
                data[j] == max_distance
                for j in range(i - angle_range, i + angle_range + 1)
                if 0 <= j < len(data)
            ):
                data[i] = (data[i - 1] + data[i + 1]) / 2
        return data.tolist()

    def filter_small_obstacles(self, left_obs, min_obstacle_size):
        for i in range(int(len(left_obs) / 2)):
            if abs(left_obs[2 * i] - left_obs[2 * i + 1]) <= min_obstacle_size:
                left_obs[2 * i] = -1
                left_obs[2 * i + 1] = -1
        return [x for x in left_obs if x != -1]

    def pub_scan(self, dis_90, header):
        scan_msg = LaserScan()
        scan_msg.header = header
        scan_msg.angle_min = math.radians(self.cfg.frontend_scan_end_deg - 1)
        scan_msg.angle_max = math.radians(self.cfg.frontend_scan_start_deg)
        scan_msg.angle_increment = -math.radians(1.0)
        scan_msg.ranges = [float(x) for x in dis_90]
        scan_msg.intensities = []
        scan_msg.range_max = 100.0
        self.scan_pub.publish(scan_msg)

    def DynamicObastcle(self, inten_list, max_dir_num):
        if len(max_dir_num) < 3:
            return False

        max_range = [max_dir_num[0], max_dir_num[-1]]
        obs_range = [max_dir_num[1], max_dir_num[2]]
        range_list = inten_list[max_range[0] : max_range[1]]
        obs_intensity = inten_list[obs_range[0] : obs_range[1]]

        if len(range_list) == 0 or len(obs_intensity) == 0:
            return False

        average_obs_intensity = np.mean(obs_intensity)
        average_intensity = np.mean(range_list)
        intensity_delta = abs(average_obs_intensity - average_intensity)
        return intensity_delta > self.cfg.dynamic_obs_intensity_delta_threshold

    def frontend_scan_process(self, data):
        self.dynamic_obs = False
        self.chaoche = False
        self.Follow = False
        self.D = self.cfg.control_base_d_gain

        dis_90, inten_90 = self.get_range(
            data,
            self.cfg.frontend_scan_start_deg,
            self.cfg.frontend_scan_end_deg,
            True,
        )
        dis_90 = dis_90[::-1]
        inten_90 = inten_90[::-1]
        dis_obs_90 = copy.deepcopy(dis_90)
        lenth_dis = len(dis_90)
        center_index = lenth_dis // 2
        center_left_index = max(0, center_index - 1)

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
        search_margin = self.cfg.frontend_max_distance_search_margin
        search_max_index = max(search_margin + 1, lenth_dis - search_margin)

        for i in range(lenth_dis):
            if dis_90[i] > max_dis and search_margin < i < search_max_index:
                max_dis = dis_90[i]
                max_dis_index = i
            if dis_90[i] > self.cfg.frontend_dir_detect_threshold:
                dis_90[i] = self.cfg.frontend_dir_detect_threshold
            if dis_obs_90[i] > self.cfg.frontend_obs_detect_threshold:
                dis_obs_90[i] = self.cfg.frontend_obs_detect_threshold

        dis_90 = self.filter_anomalous_values(
            dis_90,
            max_distance=self.cfg.frontend_dir_detect_threshold,
            angle_range=self.cfg.frontend_anomaly_angle_range,
        )
        dis_obs_90 = self.filter_anomalous_values(
            dis_obs_90,
            max_distance=self.cfg.frontend_obs_detect_threshold,
            angle_range=self.cfg.frontend_anomaly_angle_range,
        )

        if max_dis_index < center_left_index:
            left = 1
        else:
            right = 1

        for i in range(0, lenth_dis - 2):
            if (
                dis_obs_90[i] - dis_obs_90[i + 1] > self.cfg.obstacle_edge_threshold
                and len(Left_obs_orig) % 2 == 0
            ):
                Left_obs_orig.append(i + 1)
            elif (
                dis_obs_90[i + 1] - dis_obs_90[i] > self.cfg.obstacle_edge_threshold
                and len(Left_obs_orig) % 2 == 1
            ):
                Left_obs_orig.append(i)

        if len(Left_obs_orig) % 2 == 1:
            Left_obs_orig.pop()

        Left_obs = self.filter_small_obstacles(
            Left_obs_orig, min_obstacle_size=self.cfg.obstacle_small_max_width
        )

        if len(Left_obs) > 0:
            for i in range(0, int(len(Left_obs) / 2)):
                idx_mid = int((Left_obs[2 * i] + Left_obs[2 * i + 1]) / 2)
                obs_middle = dis_obs_90[idx_mid]
                expand_points = min(
                    (Left_obs[2 * i + 1] - Left_obs[2 * i]) / 2
                    * (self.cfg.obstacle_expand_distance_ref - obs_middle),
                    self.cfg.obstacle_expand_max_points,
                )
                start_expand = int(max(Left_obs[2 * i] - expand_points, 0))
                end_expand = int(
                    min(Left_obs[2 * i + 1] + expand_points, lenth_dis - 1)
                )

                for j in range(start_expand, end_expand):
                    dis_obs_90[j] = obs_middle

                Left_obs[2 * i] = start_expand
                Left_obs[2 * i + 1] = end_expand

        if len(Left_obs) > 0:
            for i in range(0, int(len(Left_obs) / 2) + 1):
                if i == 0:
                    for j in range(Left_obs[0] - 1, 0, -1):
                        if dis_obs_90[j] <= dis_obs_90[Left_obs[0] + 1]:
                            max_dis_num.append(j)
                            max_dis_num.append(Left_obs[0] + 1)
                            break
                    if len(max_dis_num) == 0:
                        for j in range(0, Left_obs[0] - 1):
                            if dis_obs_90[j] >= dis_obs_90[Left_obs[0] + 1]:
                                max_dis_num.append(j)
                                max_dis_num.append(Left_obs[0] + 1)
                                break
                elif i < int(len(Left_obs) / 2):
                    max_dis_num.append(Left_obs[2 * i - 1])
                    max_dis_num.append(Left_obs[2 * i])
                elif i == int(len(Left_obs) / 2):
                    for j in range(Left_obs[2 * i - 1] + 1, lenth_dis - 1):
                        if dis_obs_90[j] <= dis_obs_90[Left_obs[2 * i - 1] - 1]:
                            max_dis_num.append(Left_obs[2 * i - 1] - 1)
                            max_dis_num.append(j)
                            break

            max_dis_val = 0
            max_dis_index_temp = max_dis_index

            for i in range(0, int(len(max_dis_num) / 2)):
                if max_dis_val < max_dis_num[2 * i + 1] - max_dis_num[2 * i]:
                    max_dis_val = max_dis_num[2 * i + 1] - max_dis_num[2 * i]
                    max_dis_index = (
                        max_dis_num[2 * i + 1] + max_dis_num[2 * i]
                    ) / 2
                if (
                    left == 1
                    and max_dis_index < center_index
                    and dis_obs_90[0]
                    < dis_obs_90[lenth_dis - 1]
                    - self.cfg.selection_side_bias_distance_delta_threshold
                ):
                    max_dis_index = max_dis_index + self.cfg.selection_side_bias_gain * abs(
                        dis_obs_90[lenth_dis - 1] - dis_obs_90[0]
                    )
                elif (
                    right == 1
                    and max_dis_index > center_index
                    and dis_obs_90[0]
                    - self.cfg.selection_side_bias_distance_delta_threshold
                    > dis_obs_90[lenth_dis - 1]
                ):
                    max_dis_index = max_dis_index - self.cfg.selection_side_bias_gain * abs(
                        dis_obs_90[lenth_dis - 1] - dis_obs_90[0]
                    )

            if len(Left_obs) == 2:
                if max_dis_index_temp >= center_left_index:
                    if Left_obs[0] >= center_left_index:
                        max_dis_index = int((center_left_index + Left_obs[0]) / 2)
                    elif Left_obs[1] <= center_left_index:
                        max_dis_index = max_dis_index_temp
                    else:
                        max_dis_index = max_dis_index_temp
                else:
                    if Left_obs[0] >= center_left_index:
                        max_dis_index = max_dis_index_temp
                    elif Left_obs[1] <= center_left_index:
                        max_dis_index = int((center_left_index + Left_obs[1]) / 2)
                    else:
                        max_dis_index = max_dis_index_temp

            if len(Left_obs) == 4:
                middle_temp = int((Left_obs[1] + Left_obs[2]) / 2)
                if max_dis_index_temp >= center_left_index:
                    if Left_obs[0] >= center_left_index:
                        max_dis_index = int((center_left_index + Left_obs[0]) / 2)
                    elif Left_obs[1] <= center_left_index and middle_temp > center_left_index:
                        max_dis_index = int((Left_obs[1] + max_dis_index_temp) / 2)
                    elif middle_temp <= center_left_index and Left_obs[2] > center_left_index:
                        max_dis_index = middle_temp
                    else:
                        max_dis_index = max_dis_index_temp
                else:
                    if Left_obs[0] > center_left_index:
                        max_dis_index = max_dis_index_temp
                    elif Left_obs[0] <= center_left_index and middle_temp > center_left_index:
                        max_dis_index = middle_temp
                    elif middle_temp <= center_left_index and Left_obs[3] > center_left_index:
                        max_dis_index = middle_temp
                    else:
                        max_dis_index = int((Left_obs[3] + center_left_index) / 2)

        for i in range(0, lenth_dis - 2):
            if (
                dis_90[i] < self.cfg.frontend_dir_detect_threshold
                and dis_90[i + 1] == self.cfg.frontend_dir_detect_threshold
                and len(max_dir_num) % 2 == 0
            ):
                max_dir_num.append(i + 1)
            elif (
                dis_90[i] == self.cfg.frontend_dir_detect_threshold
                and dis_90[i + 1] < self.cfg.frontend_dir_detect_threshold
                and len(max_dir_num) % 2 == 1
            ):
                max_dir_num.append(i)

        if len(max_dir_num) == 1:
            if max_dir_num[0] < center_index:
                max_dir_index = int(max_dir_num[0] / 2)
            elif max_dir_num[0] > center_index:
                max_dir_index = int((max_dir_num[0] + lenth_dis - 2) / 2)
            self.GO_STARIGHT = 0

        if len(max_dir_num) == 2:
            max_dir_index = int((max_dir_num[0] + max_dir_num[1]) / 2)
            max_dir_range = max_dir_num[1] - max_dir_num[0]

        if len(max_dir_num) > 2:
            if len(Left_obs) > 0:
                self.dynamic_obs = self.DynamicObastcle(inten_90, max_dir_num)

            cand_space = []
            cand_dirs = []
            for i in range(0, int(len(max_dir_num) / 2)):
                cand_space.append(max_dir_num[2 * i + 1] - max_dir_num[2 * i])
                cand_dirs.append((max_dir_num[2 * i + 1] + max_dir_num[2 * i]) / 2)

            cand_dir_id = np.where(
                np.array(cand_space) > self.cfg.selection_candidate_gap_min_width
            )[0]
            if len(cand_dir_id) != 0:
                selected_dirs = np.array(cand_dirs)[cand_dir_id].tolist()
                max_dir_idx = np.argmin(selected_dirs)
                selected_ranges = np.array(cand_space)[cand_dir_id].tolist()
                max_dir_index = selected_dirs[max_dir_idx]
                max_dir_range = selected_ranges[max_dir_idx]
                cand_dir_chaoche_idx = np.where(
                    np.array(cand_space) > self.cfg.selection_overtake_gap_min_width
                )[0]
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

            if max_dir_index < center_index:
                max_dir_index -= self.cfg.selection_direction_bias_offset
            else:
                max_dir_index += self.cfg.selection_direction_bias_offset

        straight_min_index = center_index - self.cfg.straight_band_half_width
        straight_max_index = center_index + self.cfg.straight_band_half_width
        straight_window_start = center_index - self.cfg.straight_mean_window_half_width
        straight_window_end = center_index + self.cfg.straight_mean_window_half_width

        if straight_min_index <= max_dir_index <= straight_max_index:
            mean_straight = np.mean(dis_90_copy[straight_window_start:straight_window_end])
            self.GO_STARIGHT = 1
            self.TRANSITION = 0
            if self.last_in_straight and max_dir_range > self.cfg.straight_gap_min_width:
                self.speed_rate *= self.cfg.straight_speed_growth
                if mean_straight > self.cfg.straight_distance_far and len(Left_obs) == 0:
                    limit_rate = self.cfg.straight_speed_limit_far
                elif mean_straight > self.cfg.straight_distance_mid and len(Left_obs) == 0:
                    limit_rate = self.cfg.straight_speed_limit_mid
                elif mean_straight > self.cfg.straight_distance_near and len(Left_obs) == 0:
                    limit_rate = self.cfg.straight_speed_limit_near
                else:
                    limit_rate = self.cfg.straight_speed_limit_default
                if self.speed_rate > limit_rate:
                    self.speed_rate = limit_rate
            else:
                self.speed_rate = self.cfg.straight_speed_rate_reset

            self.last_in_straight = True
        elif 0 < max_dir_index < straight_min_index:
            self.P = self.cfg.control_turn_p_gain
            self.speed_rate = self.cfg.turn_state_speed_rate
            self.turn_rate = self.cfg.turn_state_turn_rate
            self.last_in_straight = False
        elif max_dir_index > straight_max_index:
            self.P = self.cfg.control_turn_p_gain
            self.speed_rate = self.cfg.turn_state_speed_rate
            self.turn_rate = self.cfg.turn_state_turn_rate
            self.last_in_straight = False

        normol = 1
        if len(max_dir_num) == 0 and (self.GO_STARIGHT == 1 or self.TRANSITION == 1):
            for i in range(0, lenth_dis - 2):
                if dis_90[i + 1] - dis_90[i] > self.cfg.transition_turn_threshold:
                    max_dir_index = int((i + 1 + center_index) / 2)
                    self.P = self.cfg.control_transition_p_gain
                    normol = 0
                elif dis_90[i] - dis_90[i + 1] > self.cfg.transition_turn_threshold:
                    max_dir_index = int((i + center_index) / 2)
                    self.P = self.cfg.control_transition_p_gain
                    normol = 0

            if normol == 1:
                max_dir_index = self.last_max_dir_index
                if self.last_in_normol:
                    self.speed_rate *= self.cfg.transition_speed_decay
                    self.turn_rate *= self.cfg.transition_turn_rate_growth
                    if self.speed_rate < self.cfg.transition_speed_min:
                        self.speed_rate = self.cfg.transition_speed_min
                    if self.turn_rate > self.cfg.transition_turn_rate_max:
                        self.turn_rate = self.cfg.transition_turn_rate_max
                else:
                    self.speed_rate = self.cfg.transition_speed_reset
                    self.turn_rate = self.cfg.transition_turn_rate_reset
                self.last_in_normol = True
            else:
                self.speed_rate = self.cfg.transition_normal_speed_reset
                self.turn_rate = self.cfg.transition_normal_turn_rate_reset
                self.last_in_normol = False

            self.TRANSITION = 1
            self.GO_STARIGHT = 0

        dis_90[0] = dis_90[0] + 0.00001
        dis_90[lenth_dis - 1] = dis_90[lenth_dis - 1] + 0.00001

        return {
            "dis_90": dis_90,
            "lenth_dis": lenth_dis,
            "max_dis": max_dis,
            "max_dir_index": max_dir_index,
            "max_dir_num": max_dir_num,
            "left_obs": Left_obs,
            "header": data.header,
        }

    def heuristic_backend_control(self, front_state):
        dis_90 = front_state["dis_90"]
        lenth_dis = front_state["lenth_dis"]
        max_dis = front_state["max_dis"]
        max_dir_index = front_state["max_dir_index"]

        angle = 0.0
        term1 = -max(
            math.exp(-max_dis / self.cfg.frontend_dir_detect_threshold),
            self.cfg.control_term1_decay_floor,
        ) * (max_dir_index - lenth_dis / 2) / 360 * math.pi
        term2 = (dis_90[0] - dis_90[lenth_dis - 1]) / (dis_90[0] + dis_90[lenth_dis - 1])
        distance_ratio_unbalanced = (
            dis_90[0] / dis_90[lenth_dis - 1] > self.cfg.control_distance_ratio_threshold
            or dis_90[lenth_dis - 1] / dis_90[0]
            > self.cfg.control_distance_ratio_threshold
        )

        if max_dir_index != 0:
            if distance_ratio_unbalanced:
                self.D = self.cfg.control_asymmetry_d_gain
                angle = term1 + self.cfg.control_term2_gain_unbalanced * term2
            else:
                angle = term1 + self.cfg.control_term2_gain_balanced * term2
        else:
            if distance_ratio_unbalanced:
                angle = term1 + self.cfg.control_term2_gain_zero_unbalanced * term2
            else:
                angle = term1 + self.cfg.control_term2_gain_zero_balanced * term2

        steering_angle = self.P * angle + self.D * (angle - self.last_angle)
        self.last_angle = angle

        speed = self.cfg.speed_model_scale * (
            self.cfg.speed_model_angle_decay_weight
            * math.exp(-np.clip(abs(angle), 0, self.cfg.speed_model_angle_clip))
            + self.cfg.speed_model_base_weight
        )

        steering_angle = self.turn_rate * steering_angle
        steering_angle = np.clip(
            steering_angle,
            -self.cfg.control_steering_limit_rad,
            self.cfg.control_steering_limit_rad,
        )

        return {
            "angle": angle,
            "steering": float(steering_angle),
            "speed": float(self.speed_rate * speed),
            "base_speed": float(speed),
        }

    def control_select_and_publish(self, data, current_frame, front_state, heuristic):
        drive_msg = AckermannDriveStamped()
        drive_msg.header = data.header

        steer_cmd = heuristic["steering"]
        speed_cmd = heuristic["speed"]

        if self.Follow:
            speed_cmd = float(min(self.cfg.control_follow_speed_cap, speed_cmd))

        drive_msg.drive.steering_angle = float(
            np.clip(
                steer_cmd,
                -self.cfg.control_steering_limit_rad,
                self.cfg.control_steering_limit_rad,
            )
        )
        drive_msg.drive.speed = float(max(0.0, speed_cmd))

        self.publish_arrow_marker(front_state["max_dir_index"], current_frame)
        self.last_max_dir_index = front_state["max_dir_index"]

        heuristic_msg = AckermannDriveStamped()
        heuristic_msg.header = data.header
        heuristic_msg.drive.steering_angle = drive_msg.drive.steering_angle
        heuristic_msg.drive.speed = drive_msg.drive.speed
        self.heuristic_pub.publish(heuristic_msg)

        self.current_behavior_state = self.get_behavior_state_for_log()
        self.last_cmd_speed = float(drive_msg.drive.speed)

        if self.current_behavior_state != self.last_logged_state:
            self.get_logger().info(
                self.format_battle_log(
                    state=self.current_behavior_state, speed=self.last_cmd_speed
                )
            )
            self.last_logged_state = self.current_behavior_state

        self.drive_pub.publish(drive_msg)

    def middle_line_callback(self, data):
        clean_ranges = np.nan_to_num(np.array(data.ranges), posinf=0.0, neginf=0.0)
        data.ranges = clean_ranges.tolist()

        current_frame = data.header.frame_id if data.header.frame_id else "laser"

        front_state = self.frontend_scan_process(data)
        heuristic = self.heuristic_backend_control(front_state)
        self.control_select_and_publish(data, current_frame, front_state, heuristic)


def main(args=None):
    rclpy.init(args=args)
    reactive_racing_node = ReactiveRacingNode()
    try:
        rclpy.spin(reactive_racing_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_racing_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
