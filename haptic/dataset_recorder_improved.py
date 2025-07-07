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
import json
import csv
import threading

def open_writer(writer, abs_dir_path):
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=abs_dir_path,
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

class AdvancedSyncBagRecorder(Node):
    """使用时间窗口进行精确同步的数据记录器，同时记录原始数据和时间戳"""
    def __init__(self, sync_window_ms=50):
        super().__init__('advanced_dataset_recorder')
        self.writer = rosbag2_py.SequentialWriter()
        self.sync_window_ns = sync_window_ms * 1e6  # 转换为纳秒
        
        # 使用缓冲区存储多个消息，用于时间同步
        self.msg_buffers = {}
        self.buffer_size = 100  # 每个话题保存最近100条消息
        
        # 创建唯一目录结构
        current_workspace_path = os.getcwd()
        base_dir = os.path.join(current_workspace_path, 'training_data')
        unique_dir_name = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.unique_dir = os.path.join(base_dir, 'sync_recorder', unique_dir_name)
        
        # 创建目录结构
        os.makedirs(self.unique_dir, exist_ok=True)
        
        # 创建子目录
        self.rosbag_dir = os.path.join(self.unique_dir, 'rosbag')
        self.raw_data_dir = os.path.join(self.unique_dir, 'raw_data')
        self.analysis_dir = os.path.join(self.unique_dir, 'analysis')
        
        os.makedirs(self.rosbag_dir, exist_ok=True)
        os.makedirs(self.raw_data_dir, exist_ok=True)
        os.makedirs(self.analysis_dir, exist_ok=True)
        
        # 初始化rosbag写入器
        open_writer(self.writer, self.rosbag_dir)
        
        # 创建原始数据记录文件
        self.raw_data_files = {}
        self.timing_file = open(os.path.join(self.analysis_dir, 'timing_analysis.csv'), 'w', newline='')
        self.timing_writer = csv.writer(self.timing_file)
        self.timing_writer.writerow([
            'timestamp', 'topic_name', 'receive_time', 'header_time', 
            'sync_group_id', 'time_diff_from_sync_target', 'data_summary'
        ])
        
        # 同步统计文件
        self.sync_stats_file = open(os.path.join(self.analysis_dir, 'sync_statistics.csv'), 'w', newline='')
        self.sync_stats_writer = csv.writer(self.sync_stats_file)
        self.sync_stats_writer.writerow([
            'sync_group_id', 'sync_target_time', 'max_time_diff_ms', 
            'num_topics_synced', 'sync_quality', 'timestamp'
        ])
        
        # 线程锁，确保文件写入安全
        self.file_lock = threading.Lock()
        self.sync_group_counter = 0

        self.topic_types = {
            # 机器人相关话题
            '/lbr/command/joint_position': 'lbr_fri_idl/msg/LBRJointPositionCommand',
            '/lbr/state': 'lbr_fri_idl/msg/LBRState',
            '/lbr/joint_states': 'sensor_msgs/msg/JointState',
            
            # Tac3D传感器话题
            '/positions_r': 'tutorial_interfaces/msg/Cloud',
            '/displacements_r': 'tutorial_interfaces/msg/Cloud',
            '/forces_r': 'tutorial_interfaces/msg/Array3',
            '/resultant_force_r': 'tutorial_interfaces/msg/Cloud',
            '/resultant_moment_r': 'tutorial_interfaces/msg/Cloud',
            '/index_r': 'std_msgs/msg/Float32',
            
            '/positions_l': 'tutorial_interfaces/msg/Cloud',
            '/displacements_l': 'tutorial_interfaces/msg/Cloud',
            '/forces_l': 'tutorial_interfaces/msg/Array3',
            '/resultant_force_l': 'tutorial_interfaces/msg/Cloud',
            '/resultant_moment_l': 'tutorial_interfaces/msg/Cloud',
            '/index_l': 'std_msgs/msg/Float32'
        }

        # 初始化消息缓冲区和原始数据文件
        for topic_name in self.topic_types.keys():
            self.msg_buffers[topic_name] = []
            
            # 为每个话题创建原始数据记录文件
            safe_topic_name = topic_name.replace('/', '_')
            raw_file_path = os.path.join(self.raw_data_dir, f'{safe_topic_name}_raw.csv')
            self.raw_data_files[topic_name] = open(raw_file_path, 'w', newline='')
            
            # 创建CSV写入器并写入头部
            writer = csv.writer(self.raw_data_files[topic_name])
            if 'Array3' in self.topic_types[topic_name]:
                writer.writerow(['timestamp', 'receive_time', 'header_time', 'x', 'y', 'z'])
            elif 'Cloud' in self.topic_types[topic_name]:
                writer.writerow(['timestamp', 'receive_time', 'header_time', 'num_points', 'sample_points'])
            elif 'Float32' in self.topic_types[topic_name]:
                writer.writerow(['timestamp', 'receive_time', 'header_time', 'value'])
            else:
                writer.writerow(['timestamp', 'receive_time', 'header_time', 'data_summary'])

        self._subscriptions = []

        self.get_logger().info(f"数据将保存到: {self.unique_dir}")
        self.get_logger().info(f"- ROS bag: {self.rosbag_dir}")
        self.get_logger().info(f"- 原始数据: {self.raw_data_dir}")
        self.get_logger().info(f"- 分析文件: {self.analysis_dir}")

        topic_id = 0
        for topic_name, topic_type in self.topic_types.items():
            topic_info = rosbag2_py._storage.TopicMetadata(
                id=topic_id,
                name=topic_name,
                type=topic_type,
                serialization_format='cdr')
            self.writer.create_topic(topic_info)
            topic_id += 1

            msg_type = eval(topic_type.split('/')[2])

            self._subscriptions.append(self.create_subscription(
                msg_type,
                topic_name,
                self.buffered_callback(topic_name),
                10
            ))

        # 使用更高频率进行同步处理
        self.timer = self.create_timer(0.02, self.sync_and_write)  # 50Hz

    def buffered_callback(self, topic_name):
        """创建带缓冲的回调函数，同时记录原始数据"""
        def callback(msg):
            receive_time = self.get_clock().now().nanoseconds
            
            # 获取消息时间戳
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                msg_time = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
                header_time = msg_time
            else:
                msg_time = receive_time
                header_time = None
            
            # 添加到缓冲区
            msg_entry = {
                'msg': msg,
                'timestamp': msg_time,
                'receive_time': receive_time,
                'header_time': header_time
            }
            
            self.msg_buffers[topic_name].append(msg_entry)
            
            # 保持缓冲区大小
            if len(self.msg_buffers[topic_name]) > self.buffer_size:
                self.msg_buffers[topic_name].pop(0)
            
            # 记录原始数据到CSV文件
            self.record_raw_data(topic_name, msg, receive_time, header_time)
                
        return callback
    
    def record_raw_data(self, topic_name, msg, receive_time, header_time):
        """记录原始数据到CSV文件"""
        try:
            with self.file_lock:
                writer = csv.writer(self.raw_data_files[topic_name])
                timestamp = datetime.now().isoformat()
                
                if hasattr(msg, 'x') and hasattr(msg, 'y') and hasattr(msg, 'z'):
                    # Array3 类型
                    writer.writerow([timestamp, receive_time, header_time, msg.x, msg.y, msg.z])
                elif hasattr(msg, 'row1') and hasattr(msg, 'row2') and hasattr(msg, 'row3'):
                    # Cloud 类型
                    num_points = len(msg.row1) if msg.row1 else 0
                    sample_points = f"[{msg.row1[:3] if msg.row1 else []}...]" if num_points > 0 else "[]"
                    writer.writerow([timestamp, receive_time, header_time, num_points, sample_points])
                elif hasattr(msg, 'data'):
                    # Float32 类型
                    writer.writerow([timestamp, receive_time, header_time, msg.data])
                else:
                    # 其他类型，记录摘要
                    data_summary = str(msg)[:100] + "..." if len(str(msg)) > 100 else str(msg)
                    writer.writerow([timestamp, receive_time, header_time, data_summary])
                
                # 刷新文件缓冲区
                self.raw_data_files[topic_name].flush()
        except Exception as e:
            self.get_logger().error(f"记录原始数据失败 ({topic_name}): {str(e)}")

    def sync_and_write(self):
        """基于时间窗口进行消息同步并写入，同时记录详细的同步分析"""
        # 找到最新的共同时间点
        latest_times = {}
        for topic_name, buffer in self.msg_buffers.items():
            if buffer:
                latest_times[topic_name] = buffer[-1]['timestamp']
        
        if len(latest_times) < len(self.topic_types):
            return  # 还没有所有话题的数据
        
        # 找到所有话题中最早的最新时间戳作为同步目标
        sync_target = min(latest_times.values())
        
        # 为每个话题找到最接近同步目标的消息
        synced_msgs = {}
        max_time_diff = 0
        timing_records = []
        
        for topic_name, buffer in self.msg_buffers.items():
            best_msg = None
            min_diff = float('inf')
            
            for msg_entry in reversed(buffer):  # 从最新开始搜索
                time_diff = abs(msg_entry['timestamp'] - sync_target)
                if time_diff < min_diff:
                    min_diff = time_diff
                    best_msg = msg_entry
                    
                # 如果时间差过大，停止搜索
                if time_diff > self.sync_window_ns:
                    break
            
            if best_msg and min_diff <= self.sync_window_ns:
                synced_msgs[topic_name] = best_msg
                max_time_diff = max(max_time_diff, min_diff)
                
                # 准备时间分析记录
                data_summary = self.get_data_summary(best_msg['msg'])
                timing_records.append({
                    'topic_name': topic_name,
                    'receive_time': best_msg['receive_time'],
                    'header_time': best_msg['header_time'],
                    'time_diff_from_sync_target': min_diff / 1e6,  # 转换为毫秒
                    'data_summary': data_summary
                })
        
        # 只有当所有话题都找到同步消息时才写入
        if len(synced_msgs) == len(self.topic_types):
            self.sync_group_counter += 1
            
            # 写入同步的rosbag数据
            for topic_name, msg_entry in synced_msgs.items():
                self.writer.write(
                    topic_name,
                    serialize_message(msg_entry['msg']),
                    msg_entry['timestamp'])
            
            # 记录时间分析数据
            self.record_timing_analysis(timing_records, sync_target, max_time_diff)
            
            # 记录同步质量
            max_diff_ms = max_time_diff / 1e6
            if max_diff_ms > 10:  # 超过10ms差异时警告
                self.get_logger().warn(f"同步时间差异: {max_diff_ms:.1f}ms")
            
            # 定期报告同步状态
            if self._sync_count % 100 == 0:
                self.get_logger().info(f"已同步写入 {self._sync_count} 组数据，最大时间差: {max_diff_ms:.1f}ms")
                self.get_logger().info(f"数据保存在: {self.unique_dir}")
    
    def get_data_summary(self, msg):
        """获取消息数据摘要"""
        if hasattr(msg, 'x') and hasattr(msg, 'y') and hasattr(msg, 'z'):
            return f"Array3({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})"
        elif hasattr(msg, 'row1') and hasattr(msg, 'row2') and hasattr(msg, 'row3'):
            num_points = len(msg.row1) if msg.row1 else 0
            return f"Cloud({num_points} points)"
        elif hasattr(msg, 'data'):
            return f"Float32({msg.data:.6f})"
        else:
            return str(type(msg).__name__)
    
    def record_timing_analysis(self, timing_records, sync_target, max_time_diff):
        """记录时间分析数据"""
        try:
            with self.file_lock:
                timestamp = datetime.now().isoformat()
                
                # 记录每个话题的时间信息
                for record in timing_records:
                    self.timing_writer.writerow([
                        timestamp,
                        record['topic_name'],
                        record['receive_time'],
                        record['header_time'],
                        self.sync_group_counter,
                        record['time_diff_from_sync_target'],
                        record['data_summary']
                    ])
                
                # 记录同步统计信息
                sync_quality = "excellent" if max_time_diff / 1e6 < 5 else \
                              "good" if max_time_diff / 1e6 < 10 else \
                              "fair" if max_time_diff / 1e6 < 20 else "poor"
                
                self.sync_stats_writer.writerow([
                    self.sync_group_counter,
                    sync_target,
                    max_time_diff / 1e6,
                    len(timing_records),
                    sync_quality,
                    timestamp
                ])
                
                # 刷新文件缓冲区
                self.timing_file.flush()
                self.sync_stats_file.flush()
                
        except Exception as e:
            self.get_logger().error(f"记录时间分析失败: {str(e)}")
    
    def __del__(self):
        """析构函数，确保文件正确关闭"""
        try:
            for file_handle in self.raw_data_files.values():
                if not file_handle.closed:
                    file_handle.close()
            
            if hasattr(self, 'timing_file') and not self.timing_file.closed:
                self.timing_file.close()
            
            if hasattr(self, 'sync_stats_file') and not self.sync_stats_file.closed:
                self.sync_stats_file.close()
                
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().error(f"关闭文件时出错: {str(e)}")


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

            msg_type = eval(topic_type.split('/')[2])

            self._subscriptions.append(self.create_subscription(
                msg_type,
                topic_name,
                self.msg_callback(topic_name),
                10
            ))

        self.timer = self.create_timer(0.1, self.write_messages)  # 10Hz 定时器

    def msg_callback(self, topic_name):
        """创建消息回调函数，保存消息的原始时间戳"""
        def callback(msg):
            # 保存消息及其接收时间戳
            receive_time = self.get_clock().now().nanoseconds
            msg_data = {
                'msg': msg,
                'receive_time': receive_time,
                'header_time': None
            }
            
            # 如果消息有header，提取其时间戳
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                header_time = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
                msg_data['header_time'] = header_time
            
            self.msg_buffer[topic_name].append(msg_data)
                
        return callback

    def write_messages(self):
        """将所有最新消息写入rosbag，使用消息的原始时间戳"""
        latest_msgs = {}
        now = self.get_clock().now().nanoseconds

        # 检查是否所有话题都有数据
        for topic in self.topic_types:
            buffer = self.msg_buffer[topic]
            if not buffer:
                return  # 任意话题缺失则跳过写入

            msg_data = buffer[-1]  # 最新一条消息
            msg = msg_data['msg']
            
            # 优先使用header中的时间戳，否则使用接收时间
            if msg_data['header_time'] is not None:
                timestamp = int(msg_data['header_time'])
            else:
                timestamp = msg_data['receive_time']
                
            self.writer.write(
                topic,
                serialize_message(msg),
                timestamp)

        self.get_logger().info(f"写入完成 @ {now}")


def main(args=None):
    rclpy.init(args=args)
    
    # 可以在这里选择使用哪种记录器
    print("选择数据记录方法:")
    print("1. 简单记录器 (使用消息原始时间戳)")
    print("2. 高级同步记录器 (基于时间窗口同步 + 详细分析)")
    
    try:
        choice = input("请输入选择 (1 或 2，默认为 2): ").strip()
        if choice == "1":
            recorder = SimpleBagRecorder()
            print("使用简单记录器")
        else:
            recorder = AdvancedSyncBagRecorder(sync_window_ms=50)  # 50ms同步窗口
            print("使用高级同步记录器 (50ms同步窗口)")
            print(f"数据将保存到: {recorder.unique_dir}")
    except (EOFError, KeyboardInterrupt):
        recorder = AdvancedSyncBagRecorder(sync_window_ms=50)
        print("使用默认的高级同步记录器")
        print(f"数据将保存到: {recorder.unique_dir}")
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        print("\n正在停止数据记录...")
    finally:
        # 如果是高级记录器，生成最终分析报告
        if isinstance(recorder, AdvancedSyncBagRecorder):
            print("正在生成分析报告...")
            recorder.generate_analysis_report()
            print(f"所有数据已保存到: {recorder.unique_dir}")
            print("包含以下内容:")
            print(f"  - ROS bag文件: {recorder.rosbag_dir}")
            print(f"  - 原始数据CSV: {recorder.raw_data_dir}")
            print(f"  - 分析文件: {recorder.analysis_dir}")
        
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
