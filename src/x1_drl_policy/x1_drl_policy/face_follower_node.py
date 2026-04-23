import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from x1_drl_interfaces.action import NavigateToGoal

import tf2_ros
import tf2_geometry_msgs            # to support PoseStamped transformation

import numpy as np
import time

class FaceFollowerNode(Node):
    ''' Node that subscribes to /face_pose (PoseStamped of detected face in the
    camera_optical_color_frame), projects onto the ground place, applies a goal
    tolerance (so the robot don't drive into the target), then sends NavigateToGoal
    actions to the DRL policy node
    
    While the robot moves toward the target, the goal can be updated if
    - the face has moved for than a min_displacement distance, or
    - a min_time_between_goals has passed since the previous goal was sent'''

    def __init__(self):
        super().__init__('face_follower_node')

        # ===== DECLARE PARAMETERS =====
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("goal_tolerance", 0.50)          # (m) - stop this far in front of the face
        self.declare_parameter("min_displacement", 0.20)        # (m) - min face movement before sending a new goal
        self.declare_parameter("min_time_between_goals", 1.0)   # (s) - min time between face goal updates
        self.declare_parameter("tf_timeout", 0.1)               # (s) - time waiting for a tf_lookup

        self.odom_frame             = self.get_parameter("odom_frame").value
        self.robot_base_frame       = self.get_parameter("robot_base_frame").value
        self.goal_tolerance         = self.get_parameter("goal_tolerance").value
        self.min_displacement       = self.get_parameter("min_displacement").value
        self.min_time_between_goals = self.get_parameter("min_time_between_goals").value
        self.tf_timeout             = self.get_parameter("tf_timeout").value

        # ===== TF2 INTERFACE =====
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ===== ACTION CLIENT =====
        self._callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, 
            NavigateToGoal, 
            'navigate_to_goal',
            callback_group=self._callback_group
        )

        # ===== SUBSCRIBER =====
        self.face_pose_sub = self.create_subscription(
            PoseStamped,
            '/face_pose',
            self.face_pose_callback,
            10,
            callback_group=self._callback_group
        )

        # ===== STATE INIT =====
        self._current_goal_handle: ClientGoalHandle | None = None
        self._last_goal_odom: np.ndarray | None = None
        self._last_goal_time: float = 0.0
        self._cancel_in_flight: bool = False

        self.get_logger().info("FaceFollowerNode ready")

    def face_pose_callback(self, msg: PoseStamped):
        ''' Called every time a new pose is published on /face_pose to:
        1. Transform the camera-frame pose into the odom frame
        2. Project the 3D point onto the ground/odom plane 
        3. Check conditions before sending a new goal
        4. Cancel active goal and send the new one
        '''

        # --- 1. Transform face pose into odom frame
        pose_odom = self._transform_to_odom(msg)
        if pose_odom is None:
            return
        
        # --- 2. Project pose onto odom x-y plane
        goal_xy = np.array([
            pose_odom.pose.position.x,
            pose_odom.pose.position.y
        ])

        # --- 3. Check conditions to send goal (displacement + elapsed time)
        now = time.time()
        if self._last_goal_odom is not None:
            displacement = np.linalg.norm(goal_xy - self._last_goal_odom)
            time_elapsed = now - self._last_goal_time
            if displacement < self.min_displacement and time_elapsed <= self.min_time_between_goals:
                return
            
        # --- 4. Send the goal pose
        self._update_goal(goal_xy, msg.header.stamp)
        self._last_goal_odom = goal_xy
        self._last_goal_time = now

    def _update_goal(self, goal_xy: np.ndarray, stamp):
        if self._current_goal_handle is not None and not self._cancel_in_flight:
            self._cancel_in_flight = True
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self._on_cancel_done(f, goal_xy, stamp))
        else:   
            self._send_goal(goal_xy, stamp)

    def _on_cancel_done(self, future, goal_xy: np.ndarray, stamp):
        self._cancel_in_flight = False
        self._current_goal_handle = None
        self._send_goal(goal_xy, stamp)

    def _send_goal(self, goal_xy: np.ndarray, stamp):
        '''Create a NavigateToGoal action and send to the server'''
        if not self._action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('navigate_to_goal action server not available')
            return
    
        target = PoseStamped()
        target.header.stamp = stamp
        target.header.frame_id = self.odom_frame
        target.pose.position.x = float(goal_xy[0])
        target.pose.position.y = float(goal_xy[1])
        target.pose.position.z = 0.0
        target.pose.orientation.w = 1.0

        goal_msg = NavigateToGoal.Goal()
        goal_msg.target_pose = target
        goal_msg.goal_tolerance = self.goal_tolerance

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

        self.get_logger().info(f"Sending goal (odom frame): {goal_xy[0]: 5.3f}, {goal_xy[1]: 5.3f}")

    def _goal_response_callback(self, future):
        '''Called to acknowledge goal accept/reject from the server'''
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by the navigate_to_goal server.')
            return
        
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        '''Called when the current goal finishes'''
        result = future.result().result
        status = future.result().status

        from action_msgs.msg import GoalStatus
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal succeeded. Distance travelled: {result.total_distance:.2f}")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"Goal cancelled (preempted by new face pose)")
        else:
            self.get_logger().warn(f"Goal aborted {result.message}")

        self._current_goal_handle = None
    
    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Distance to face goal: {feedback.distance_to_goal:.2f}m | "
            f"Elasped time: {feedback.elapsed_time}s"
        )
        pass

    def _transform_to_odom(self, pose_stamped: PoseStamped) -> PoseStamped | None:
        ''' Transform a PoseStamped from the original frame to the odom frame
        using the tf tree'''

        try: 
            timeout = rclpy.duration.Duration(seconds=self.tf_timeout)
            pose_odom = self.tf_buffer.transform(pose_stamped, self.odom_frame, timeout=timeout)
            return pose_odom
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'TF LookupException: {e}', throttle_duration_sec=2.0)
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f'TF ConnectivityException: {e}', throttle_duration_sec=2.0)
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'TF ExtrapolationException: {e}', throttle_duration_sec=2.0)
        return None

def main():
    rclpy.init()
    node = FaceFollowerNode()
 
    executor = MultiThreadedExecutor()
    executor.add_node(node)
 
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()