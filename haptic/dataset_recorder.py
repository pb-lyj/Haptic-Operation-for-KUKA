import os
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from lbr_fri_idl.msg import LBRJointPositionCommand, LBRState
from sensor_msgs.msg import JointState
from tutorial_interfaces.msg import Array3, Cloud
from std_msgs.msg import Float32
import rosbag2_py
from datetime import datetime
from collections import deque

def open_writer(writer, abs_dir_path):
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=abs_dir_path,
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('dataset_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        current_workspace_path = os.getcwd()
        base_dir = os.path.join(current_workspace_path, 'training_data')
        data_dir = os.path.join(base_dir, 'multi_topic_recorder')
        os.makedirs(data_dir, exist_ok=True)

        unique_dir = datetime.now().strftime('%Y%m%d_%H%M%S')
        save_path = os.path.join(data_dir, unique_dir)

        open_writer(self.writer, save_path)

        # 所有话题（机械臂 + 左右传感器）
        self.topic_types = {
            # 机械臂话题
            '/lbr/command/joint_position': 'lbr_fri_idl/msg/LBRJointPositionCommand',
            '/lbr/state': 'lbr_fri_idl/msg/LBRState',
            '/lbr/joint_states': 'sensor_msgs/msg/JointState',

            # 右传感器（_r）
            '/positions_r': 'tutorial_interfaces/msg/Cloud',
            '/displacements_r': 'tutorial_interfaces/msg/Cloud',
            '/forces_r': 'tutorial_interfaces/msg/Array3',
            '/resultant_force_r': 'tutorial_interfaces/msg/Cloud',
            '/resultant_moment_r': 'tutorial_interfaces/msg/Cloud',
            '/index_r': 'std_msgs/msg/Float32',

            # 左传感器（_l）
            '/positions_l': 'tutorial_interfaces/msg/Cloud',
            '/displacements_l': 'tutorial_interfaces/msg/Cloud',
            '/forces_l': 'tutorial_interfaces/msg/Array3',
            '/resultant_force_l': 'tutorial_interfaces/msg/Cloud',
            '/resultant_moment_l': 'tutorial_interfaces/msg/Cloud',
            '/index_l': 'std_msgs/msg/Float32'
        }

        self._subscriptions = []
        self.msg_buffer = {
            topic: deque(maxlen=100) for topic in self.topic_types
        }

        topic_id = 0
        for topic_name, topic_type in self.topic_types.items():
            topic_info = rosbag2_py._storage.TopicMetadata(
                id=topic_id,
                name=topic_name,
                type=topic_type,
                serialization_format='cdr')
            self.writer.create_topic(topic_info)
            topic_id += 1

            msg_type = eval(topic_type.split('/')[2])  # 获取消息类型
            self._subscriptions.append(
                self.create_subscription(
                    msg_type,
                    topic_name,
                    self.msg_callback(topic_name),
                    10
                )
            )

        # 每 0.1 秒写入一次（统一时间戳）
        self.timer = self.create_timer(0.05, self.write_messages)

    def msg_callback(self, topic_name):
        def callback(msg):
            now = self.get_clock().now().nanoseconds
            self.msg_buffer[topic_name].append((now, msg))
        return callback

    def write_messages(self):
        now = self.get_clock().now().nanoseconds
        latest_msgs = {}

        for topic, buffer in self.msg_buffer.items():
            if not buffer:
                self.get_logger().warn(f"等待数据中: {topic}")
                return  # 任意话题缺失则跳过写入

            latest_msgs[topic] = buffer[-1][1]  # 最新一条消息

        # 统一写入全部话题
        for topic, msg in latest_msgs.items():
            self.writer.write(topic, serialize_message(msg), now)

        self.get_logger().info(f"写入完成 @ {now}")

def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
