import os
import csv
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, CancelResponse
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int32

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # Parameters
        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('experiment_result_csv_name', '')
        self.declare_parameter('loop_enable', False)
        self.declare_parameter('loop_count', 0)
        waypoints_csv = self.get_parameter('waypoints_csv').value
        experiment_result_csv = self.get_parameter('experiment_result_csv_name').value
        self.loop_enable = self.get_parameter('loop_enable').value
        self.loop_count = self.get_parameter('loop_count').value

        # 保存先CSVファイルのパスをメンバ変数に保持
        self.experiment_result_csv = experiment_result_csv

        # CSVファイルの存在チェック（なければ新規作成、存在すればそのファイルを利用）
        if not os.path.exists(self.experiment_result_csv):
            try:
                with open(self.experiment_result_csv, mode='w', newline='') as csv_file:
                    writer = csv.writer(csv_file)
                    header = [
                        "waypoint_index",
                        "stamp_sec",
                        "stamp_nanosec",
                        "position_x",
                        "position_y",
                        "position_z",
                        "orientation_x",
                        "orientation_y",
                        "orientation_z",
                        "orientation_w"
                    ]
                    writer.writerow(header)
                self.get_logger().info(f"Created experiment result CSV file: {self.experiment_result_csv}")
            except Exception as e:
                self.get_logger().error(f"Failed to create experiment result CSV file: {e}")
        else:
            self.get_logger().info(f"Loaded existing experiment result CSV file: {self.experiment_result_csv}")

        # Subscribers
        self.skip_flag_sub = self.create_subscription(Bool, 'skip_flag', self.skip_flag_callback, 10)
        self.event_flag_sub = self.create_subscription(Int32, 'event_flag', self.event_flag_callback, 10)
        self.current_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'current_pose', self.current_pose_callback, 10)

        # Publishers
        self.next_waypoint_id_pub = self.create_publisher(Int32, 'next_waypoint_id', 10)
        self.reached_waypoint_id_pub = self.create_publisher(Int32, 'reached_waypoint_id', 10)

        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Load waypoints from CSV file
        self.waypoints_data = self.load_waypoints_from_csv(waypoints_csv)
        if not self.waypoints_data:
            self.get_logger().error("No waypoints loaded. Please check the CSV file.")
            return

        # Variables to manage waypoints and navigation
        self.current_waypoint_index = 0
        self.current_loop_count = 0
        self.skip_flag = False
        self.waiting_for_event = False
        self._last_feedback_time = self.get_clock().now()
        self.retry_count = 0
        self.goal_handle = None

        # 最新の current_pose メッセージを保持する変数
        self.current_pose = None

    def load_waypoints_from_csv(self, filename):
        waypoints_data = []
        with open(filename, mode='r') as file:
            reader = csv.reader(file)
            header = next(reader)

            for row in reader:
                pose_stamped_msg = PoseStamped()
                pose_stamped_msg.header.frame_id = 'map'
                pose_stamped_msg.pose.position.x = float(row[1])
                pose_stamped_msg.pose.position.y = float(row[2])
                pose_stamped_msg.pose.position.z = float(row[3])
                pose_stamped_msg.pose.orientation.x = float(row[4])
                pose_stamped_msg.pose.orientation.y = float(row[5])
                pose_stamped_msg.pose.orientation.z = float(row[6])
                pose_stamped_msg.pose.orientation.w = float(row[7])

                waypoint_data = {
                    "pose": pose_stamped_msg,
                    "skip_flag": int(row[8]),
                    "event_flag": int(row[9])
                }
                
                waypoints_data.append(waypoint_data)

        return waypoints_data

    def skip_flag_callback(self, msg):
        # Handle skip flag messages
        if msg.data and self.current_waypoint_index < len(self.waypoints_data):
            if self.waypoints_data[self.current_waypoint_index]["skip_flag"] == 1:
                self.skip_flag = True
                if self.goal_handle is not None:
                    self.get_logger().info(f'Cancelling goal for waypoint {self.current_waypoint_index} due to skip.')
                    cancel_future = self.goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(self.cancel_done_callback)

    def event_flag_callback(self, msg):
        # イベント待機中に event_flag==1 を受信した場合、
        # 保存はすでに済んでいるので、単に次のウェイポイントへ進む。
        if self.waiting_for_event and msg.data == 1:
            self.get_logger().info('Event flag received. Proceeding to next waypoint.')
            self.waiting_for_event = False
            self.current_waypoint_index += 1
            self.advance_to_next_waypoint()

    def current_pose_callback(self, msg):
        # 最新の current_pose を保持する
        self.current_pose = msg

    def send_goal(self, waypoint_data):
        # Send a navigation goal to the action server
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint_data["pose"]

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available, waiting...')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Handle feedback from the action server
        current_time = self.get_clock().now()
        if (current_time - self._last_feedback_time).nanoseconds >= 3e9:
            # 例: フィードバック内容のログ出力（必要に応じて）
            # self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.distance_remaining))
            self._last_feedback_time = current_time

    def goal_response_callback(self, future):
        # Handle the response from the action server when a goal is sent
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server.')
            self.goal_handle = None
            return
        
        self.goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        
        # Publish the next waypoint ID
        if self.current_waypoint_index < len(self.waypoints_data):
            next_waypoint_id = self.current_waypoint_index
            self.next_waypoint_id_pub.publish(Int32(data=next_waypoint_id))

    def goal_result_callback(self, future):
        # Handle the result of the navigation goal
        status = future.result().status
        self.goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached successfully.')
            self.skip_flag = False
            self.retry_count = 0
            
            # Publish the reached waypoint ID
            reached_waypoint_id = self.current_waypoint_index
            self.reached_waypoint_id_pub.publish(Int32(data=reached_waypoint_id))

            current_waypoint_data = self.waypoints_data[self.current_waypoint_index]

            if current_waypoint_data["event_flag"] == 1:
                # ここで「待機開始」するタイミングで current_pose を保存する
                if self.current_pose is not None:
                    self.write_experiment_result_from_pose(self.current_pose)
                else:
                    self.get_logger().warn("No current_pose received yet at waiting start.")
                # イベント待機状態に入る
                self.waiting_for_event = True
                self.get_logger().info(f'Waiting for event to proceed from waypoint {self.current_waypoint_index}.')
            else:
                # イベント待機が不要な場合は次のウェイポイントへ進む
                self.current_waypoint_index += 1
                self.advance_to_next_waypoint()

        elif status == GoalStatus.STATUS_ABORTED:
            if self.retry_count < 1:
                self.retry_count += 1
                self.get_logger().warn(f'Navigation to waypoint {self.current_waypoint_index} aborted. Retrying ({self.retry_count}/1)...')
                if self.current_waypoint_index < len(self.waypoints_data):
                    self.send_goal(self.waypoints_data[self.current_waypoint_index])
                else:
                    self.get_logger().warn('No more waypoints to navigate to.')
            else:
                self.get_logger().warn(f'Navigation to waypoint {self.current_waypoint_index} aborted after retry. Moving to next waypoint.')
                self.retry_count = 0
                self.skip_flag = False
                self.current_waypoint_index += 1
                self.advance_to_next_waypoint()
                
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'Navigation to waypoint {self.current_waypoint_index} canceled.')
            self.skip_flag = False
            self.current_waypoint_index += 1
            self.advance_to_next_waypoint()
            
        else:
            self.get_logger().warn(f'Goal failed with status code: {status}. Not advancing to next waypoint.')

    def cancel_done_callback(self, future):
        # Handle the result of a goal cancellation
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(f'Goal cancel request accepted for waypoint {self.current_waypoint_index}.')
        else:
            self.get_logger().warn(f'Goal cancel request rejected for waypoint {self.current_waypoint_index}.')

    def advance_to_next_waypoint(self):
        # Proceed to the next waypoint
        if self.current_waypoint_index < len(self.waypoints_data):
            next_waypoint_data = self.waypoints_data[self.current_waypoint_index]
            self.retry_count = 0
            next_waypoint_data["pose"].header.stamp = self.get_clock().now().to_msg()
            self.send_goal(next_waypoint_data)
        else:
            # Handle looping or completion
            if self.loop_enable and self.current_loop_count < self.loop_count - 1:
                self.current_waypoint_index = 0
                self.current_loop_count += 1
                self.get_logger().info(f'Starting loop {self.current_loop_count + 1} / {self.loop_count}.')
                self.advance_to_next_waypoint()
            else:
                self.get_logger().info('Arrived at the last waypoint. Navigation complete.')
        self.skip_flag = False

    def write_experiment_result_from_pose(self, pose_msg):
        """
        current_pose (PoseWithCovarianceStamped) から取得した座標情報を
        experiment_result_csv に追記する。
        """
        pose = pose_msg.pose.pose
        row = [
            self.current_waypoint_index,
            pose_msg.header.stamp.sec,
            pose_msg.header.stamp.nanosec,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]
        try:
            with open(self.experiment_result_csv, mode='a', newline='') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(row)
            self.get_logger().info("Current pose coordinate saved to experiment result CSV.")
        except Exception as e:
            self.get_logger().error(f"Failed to write to CSV file: {e}")

    def run(self):
        # Start the navigation process
        if self.waypoints_data and self.current_waypoint_index < len(self.waypoints_data):
            self.send_goal(self.waypoints_data[self.current_waypoint_index])
        else:
            self.get_logger().info('No waypoints to navigate to.')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Received KeyboardInterrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
