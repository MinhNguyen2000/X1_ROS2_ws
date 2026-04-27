import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor       # to prevent blocking code while navigating to the goal
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from x1_drl_interfaces.action import NavigateToGoal

import numpy as np
# TODO - import torch related packages for DRL inference (torch and torch.nn)
import torch, torch.nn as nn
# from stable_baselines3 import TD3, SAC
import gymnasium as gym
from stable_baselines3.td3.policies import Actor

from ament_index_python.packages import get_package_share_directory
import os
import json
import time
import copy
import signal

class DRLPolicyNode(Node):
    def __init__(self):
        super().__init__('drl_policy_server')

        # TODO - declare and store ROS2 parameters/runtime arguments 
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('obstacle_tolerance', 0.20)
        self.declare_parameter('model_name', 'TD3_001')
        self.declare_parameter('max_lin_vel', 0.6)
        self.declare_parameter('max_angular_vel', 0.9)
        self.declare_parameter('goal_timeout', 30.0)

        self.default_goal_tolerance     = self.get_parameter('goal_tolerance').value
        self.default_obstacle_tolerance = self.get_parameter('obstacle_tolerance').value
        self.model_name                 = self.get_parameter('model_name').value
        self.model_type                 = self.model_name.split('_')[0]
        self.max_lin_vel                = self.get_parameter('max_lin_vel').value
        self.max_angular_vel            = self.get_parameter('max_angular_vel').value
        self.goal_timeout               = self.get_parameter('goal_timeout').value

        # TODO - load the model and extract the number of n_ray_groups for LiDAR group
        # Assume that the models and norm stats are stored under drl_policy/policy/TD3_xxx/model.zip and norm_stats.pkl
        pkg_dir = get_package_share_directory('x1_drl_policy')
        model_dir = os.path.join(pkg_dir, 'policies', self.model_name)
        # model_path = os.path.join(model_dir, "model")
        norm_stat_path = os.path.join(model_dir, "norm_stats.npz")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy = Actor(
            observation_space=gym.spaces.Box(low=-np.inf, high=np.inf, shape=(27,), dtype=np.float32),
            action_space=gym.spaces.Box(low=np.array([0.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32),
            net_arch=[512, 256],
            features_extractor=torch.nn.Identity(),   # MlpPolicy uses FlattenExtractor but weights already flattened
            features_dim=27,
            activation_fn=torch.nn.ReLU,
            normalize_images=False,
        ).to(self.device)
        self.policy.eval()

        # Load norm stats
        norm = np.load(norm_stat_path)
        self.obs_mean    = norm["mean"].astype(np.float32)
        self.obs_var     = norm["var"].astype(np.float32)
        self.clip_obs    = float(norm["clip_obs"][0])

        # TODO - read n_ray_groups and obs_space size dynamically from the loaded model
        self.n_ray_groups = 18
        self._obs_space_size = 27
        self._obs_buffer = np.zeros(self._obs_space_size, dtype=np.float32)

        # TODO - initialize DRL stuff
        self.action_last = np.zeros(2)
        self.action = np.zeros(2)
        self.d_goal_last = 0.0
        self.prev_abs_diff = 0.0
        self.min_dist_last = 0.0
        self.d_safe = 0.5
        self.lidar_idx_threshold = 4

        # intialize the scaled reward components
        self.rew_head_approach_scaled = 0   ; self.rew_head_approach_scale = 200.0
        self.rew_dist_approach_scaled = 0   ; self.rew_dist_approach_scale = 200.0
        self.rew_obs_dist_scaled = 0        ; self.rew_obs_dist_scale = 0.5
        self.rew_obs_align_scaled = 0       ; self.rew_obs_align_scale = 0.5
        self.rew_time = -0.5

        # --- State storage ---
        self.latest_odom: Odometry | None = None
        self.latest_scan: LaserScan | None = None

        # --- Subscribers & Publisher ---
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        # TODO - make the topic name dynamic according to /agent_name/odom namespace when this node is launched
        self.odom_sub = self.create_subscription(Odometry,'/odom', self.odom_callback, qos)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        # TODO - make the topic name dynamic according to /agent_name/cmd_vel namespace when this node is launched
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # --- Active goal handle ---
        self._current_goal_handle: ServerGoalHandle | None = None

        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # index of the zero angle in the (-π, π) scan
        zero_idx = round(-msg.angle_min / msg.angle_increment)

        # rotate the ranges from (-π, π) to (0, 2π)
        ranges_drl = np.roll(ranges, -zero_idx)

        self.latest_scan = copy.deepcopy(msg)
        self.latest_scan.ranges = ranges_drl.tolist()

    def goal_callback(self, goal_request: NavigateToGoal):
        '''
        Called when a new goal request arrives
        
        :param goal_request:
        '''
        self.get_logger().info(
            f"New goal received at: ({goal_request.target_pose.pose.position.x: 5.3f},"
            f"{goal_request.target_pose.pose.position.y: 5.3f})"
        )

        # Overwrite any current goal
        if self._current_goal_handle is not None and self._current_goal_handle.is_active:
            self.get_logger().info('Changing the current goal.')
            self._current_goal_handle.abort()

        return GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle):
        '''
        Called when a cancel request arrives 
        
        :param goal_handle: 
        '''
        self.get_logger().info('Cancel requested.')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle: ServerGoalHandle):
        '''
        Main control loop that runs while the goal is active
        
        :param self: Description
        :param goal_handle: Description
        :type goal_handle: ServerGoalHandle
        '''
        self._current_goal_handle = goal_handle

        target = goal_handle.request.target_pose
        self.goal_tolerance = (goal_handle.request.goal_tolerance 
                               if goal_handle.request.goal_tolerance > 0.0
                               else self.default_goal_tolerance)
        self.obstacle_tolerance = self.default_obstacle_tolerance
        start = time.time()

        # Initialize feedback and result message
        feedback_msg = NavigateToGoal.Feedback()
        result_msg = NavigateToGoal.Result()

        x_prev = self.latest_odom.pose.pose.position.x
        y_prev = self.latest_odom.pose.pose.position.y
        total_distance = 0.0

        ctrl_freq = 50
        ctrl_period = 1.0 / ctrl_freq

        while rclpy.ok():
            ctrl_iter_start = time.time()
            elapsed_time = time.time() - start

            # --- Check for cancellation ---
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                self.cmd_pub.publish(cmd)   # stop the robot
                result_msg.success = False
                result_msg.message = 'Cancelled by client.'
                return result_msg
            
            # --- Check for timeout ---
            if elapsed_time >= self.goal_timeout:
                goal_handle.abort()
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                self.cmd_pub.publish(cmd)   # stop the robot
                result_msg.success = False
                result_msg.message = f'Goal timeout after {elapsed_time:.1f}s'
                result_msg.total_distance=float(total_distance)
                self.get_logger().warn(f'Goal timed out after {elapsed_time:.1f}s')
                return result_msg

            # --- Wait for all data ---
            if (self.latest_odom is None) or (self.latest_scan is None):
                self.get_logger().warn('Waiting for odometry and LiDAR...')
                time.sleep(ctrl_period)
                continue

            # --- 1. Extract + Normalize observation ---
            obs = self._get_obs(self.latest_odom, target, self.latest_scan)

            obs_normed = self._normalize_obs(obs)
            
            # self.get_logger().info(
            #     f'obs → ({obs[0]: 5.3f} | {obs[1]: 5.3f} | {obs[2]: 5.3f})   '
            #     f'({np.arctan2(obs[4], obs[3]): 5.3f} | {np.arctan2(obs[6], obs[5]): 5.3f})   '
            #     f'({obs[7]: 5.3f}|{obs[8]: 5.3f})   '
            #     f'lidar:{obs[9:]}'
            # )

            # --- 2. Check termination conditions ---
            d_goal = obs[2]
            min_lidar = np.min(obs[9:])
            # self.get_logger().info(f'Minimum LiDAR reading: {min_lidar}')
            if d_goal <= self.goal_tolerance:
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                self.cmd_pub.publish(cmd)   # stop the robot
                goal_handle.succeed()
                result_msg.success=True
                result_msg.message='Goal reached.'
                result_msg.total_distance=float(total_distance)
                self.get_logger().info('Goal reached.')
                return result_msg
            
            if min_lidar <= self.obstacle_tolerance:
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                self.cmd_pub.publish(cmd)   # stop the robot
                goal_handle.abort()
                result_msg.success=False
                result_msg.message='Obstacle hit, mission aborted.'
                result_msg.total_distance=float(total_distance)
                self.get_logger().info(f'Obstacle hit with min_lidar={min_lidar: 5.3f}, mission aborted.')
                return result_msg
            
            # --- 3. DRL policy inference => action ---
            self.action = self._run_policy(obs_normed)          # vx, vyaw in moving agent frame

            self.action[0] = np.clip(self.action[0], 0.0, 1.0)
            self.action[1] = np.clip(self.action[1], -1.0, 1.0)

            self._get_rewards(obs)

            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'agent_base_link'
            cmd.twist.linear.x = float(self.action[0]) * self.max_lin_vel
            cmd.twist.angular.z = float(self.action[1]) * self.max_angular_vel
            # self.get_logger().info(f"Linear vel: {cmd.twist.linear.x: 5.3f} | Angular vel: {cmd.twist.angular.z: 5.3f}")
            self.cmd_pub.publish(cmd)

            # --- Extra: Publish Feedback
            feedback_msg.distance_to_goal = float(d_goal)
            feedback_msg.elapsed_time = float(elapsed_time)
            feedback_msg.current_pose = self.latest_odom.pose.pose
            goal_handle.publish_feedback(feedback_msg)

            # --- Extra: Find total distance travelled over the last step
            x = self.latest_odom.pose.pose.position.x
            y = self.latest_odom.pose.pose.position.y
            step_distance = np.sqrt((x - x_prev) ** 2 + (y - y_prev) ** 2)
            total_distance += step_distance
            x_prev, y_prev = x, y

            # --- Manage control frequency
            ctrl_iter_elapsed = time.time() - ctrl_iter_start
            ctrl_iter_remain = ctrl_period - ctrl_iter_elapsed
            if ctrl_iter_remain > 0:
                time.sleep(ctrl_iter_remain)

    def _yaw_from_quaternion(self, q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return np.arctan2(siny_cosp, cosy_cosp, dtype=np.float32)

    def _get_obs(self, odom: Odometry, target: PoseStamped, scan: LaserScan) -> np.ndarray:
        '''
        Build the observation from /odom messages to match the DRL policy's observation
        The observation space contains
        (dx, dy, dgoal, s_theta, c_theta, s_phi, c_phi, vx, vy, LiDAR scans)
        
        :param odom: odometry message from /odom (filtered wheel odometry, LiDAR matching, and IMU)
        :type odom: Odometry

        :return: Description
        :rtype: ndarray
        '''

        # --- Extract raw odometry and sensor data ---
        agent_pos = odom.pose.pose.position                 # agent (x,y,z) in initial odom frame
        ori_quat = odom.pose.pose.orientation
        agent_yaw = self._yaw_from_quaternion(ori_quat)     # agent yaw in initial odom frame
        # agent_vx = odom.twist.twist.linear.x                # agent's velocity in the moving frame TODO - verify whether this is in the agent frame or world frame
        # agent_vyaw = odom.twist.twist.angular.z             # agent's yaw velocity along z in the moving frame
        agent_vx = self.action[0]
        agent_vyaw = self.action[1]

        goal_pos = target.pose.position

        # TODO - perform the calculations required to form the observation
        dx = goal_pos.x - agent_pos.x
        dy = goal_pos.y - agent_pos.y
        dgoal = np.sqrt(dx**2 + dy**2)

        bearing = np.arctan2(dy, dx, dtype=np.float32) % (2 * np.pi)
        heading = agent_yaw
        rel_bearing = - ((bearing - heading + np.pi) % (2*np.pi) - np.pi)
        
        c_heading = np.cos(heading, dtype=np.float32)
        s_heading = np.sin(heading, dtype=np.float32) 
        c_bearing = np.cos(rel_bearing, dtype=np.float32)
        s_bearing = np.sin(rel_bearing, dtype=np.float32)

        # LIDAR min pooling
        raw = np.array(scan.ranges, dtype=np.float32)
        raw = np.where(np.isfinite(raw), raw, scan.range_max)       # replace inf/nan with max LiDAR range values
        antennae_mask = raw <= 0.20
        raw[antennae_mask] = scan.range_max                         # drop anything below the minimum LiDAR scan range 
        raw = np.clip(raw, 0.0, scan.range_max)
        raw = np.flip(raw)
        n_groups = 18           # TODO - dynamically modify this according to the model loaded
        lidar_groups    = np.array_split(raw, n_groups)
        lidar_obs       = np.array([g.min() for g in lidar_groups], dtype=np.float32)

        self._obs_buffer[0:3] = dx, dy, dgoal
        self._obs_buffer[3:5] = c_heading, s_heading
        self._obs_buffer[5:7] = c_bearing, s_bearing
        self._obs_buffer[7:9] = agent_vx, agent_vyaw
        self._obs_buffer[9:]  = lidar_obs

        return self._obs_buffer
        
    def _normalize_obs(self, obs: np.ndarray) -> np.ndarray:
        obs_normed = (obs - self.obs_mean) / np.sqrt(self.obs_var + 1e-8)
        return np.clip(obs_normed, -self.clip_obs, self.clip_obs).astype(np.float32)

    def _get_rewards(self, obs):
        d_goal = obs[2]
        abs_diff = np.arctan2(obs[6], obs[5], dtype=np.float32)
        lidar_obs = obs[9:]
        min_dist = np.min(lidar_obs)
        min_dist_idx = np.argmin(lidar_obs)
        v_lin, v_ang = obs[7:9]

        if min_dist >= self.d_safe:
            #--- DISTANCE APPROACH REWARD:
            # this reward term incentivizes approaching the goal, and rewards 0 otherwise:
            rew_dist_approach = max((self.d_goal_last - d_goal), 0)

            #--- HEADING APPROACH REWARD:
            # this reward term incentivizes approaching the required heading, and rewards 0 otherwise
            rew_head_approach = max((self.prev_abs_diff - abs_diff), 0) if self.action[0] >= 0.05 else 0

            # zero the obstacle terms:
            rew_obs_dist = 0
            rew_obs_align = 0
        
        # when near an obstacle, focus on moving away
        else:
            rew_dist_approach = 0
            rew_head_approach = 0
            #--- OBSTACLE APPROACH PENALTY:
            rew_obs_dist = min((min_dist / (self.min_dist_last + 1e-6) - 1), 0)

            # REWARD FOR NOT BEING ALIGNED WITH OBSTACLES:
            if min_dist_idx >= self.lidar_idx_threshold and min_dist_idx < self.n_ray_groups - self.lidar_idx_threshold and v_lin >= 0.05:
                rew_obs_align = 1
            else:
                if (min_dist_idx < self.lidar_idx_threshold and v_ang > 0) or (min_dist_idx >= self.n_ray_groups - self.lidar_idx_threshold and v_ang < 0):
                    rew_obs_align = min(1, np.abs(v_ang))
                else:
                    rew_obs_align = 0

        #--- PENALIZE ABRUPT CHANGES IN VELOCITY
        act_diff = np.abs(self.action - self.action_last)          # penalize both abrupt changes in linear and angular velocities
        rew_act_diff = -0.5 * np.sum(act_diff ** 2)

        #--- PENALIZE STALLING:
        if abs(self.action[0]) <= 0.05:
            rew_time = 2 * self.rew_time
        else:
            rew_time = self.rew_time

        # Scaling 
        self.rew_dist_approach_scaled   = self.rew_dist_approach_scale  * rew_dist_approach
        self.rew_head_approach_scaled   = self.rew_head_approach_scale  * rew_head_approach
        self.rew_obs_dist_scaled        = self.rew_obs_dist_scale       * rew_obs_dist
        self.rew_obs_align_scaled       = self.rew_obs_align_scale      * rew_obs_align

        #--- TOTAL REWARD:
        rew = 0
        rew += self.rew_dist_approach_scaled + self.rew_head_approach_scaled
        rew += self.rew_obs_dist_scaled + self.rew_obs_align_scaled
        rew += rew_act_diff
        rew += rew_time

        self.d_goal_last = d_goal
        self.prev_abs_diff = abs_diff
        self.min_dist_last = min_dist
        self.action_last = self.action

        # self.get_logger().info(f"r_dist_app: {self.rew_dist_approach_scaled: 5.3f} | "
        #                        f"r_head_app: {self.rew_head_approach_scaled: 5.3f} | "
        #                        f"r_obs_dist: {self.rew_obs_dist_scaled: 5.3f} | "
        #                        f"r_obs_align: {self.rew_obs_align_scaled: 5.3f} | "
        #                        f"rew_act_diff: {rew_act_diff: 5.3f} | "
        #                        f"r_time: {self.rew_time: 4.2f} | "
        #                        f"r_total: {rew: 5.3f}  ")

    def _run_policy(self, obs: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            obs_tensor = torch.from_numpy(obs).unsqueeze(0).to(self.device)
            action = self.policy(obs_tensor)
        return action.squeeze(0).cpu().numpy()

def main():
    rclpy.init()
    node = DRLPolicyNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # catch sigterm from GUI:
    signal.signal(signal.SIGTERM, lambda *args: executor.shutdown())

    try: 
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()