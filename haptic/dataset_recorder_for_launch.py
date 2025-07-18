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
import select
import sys
import termios
import tty

def open_writer(writer, abs_dir_path):
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=abs_dir_path,
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

class LaunchControlledRecorder(Node):
    """通过Launch启动，等待用户输入才开始记录的数据记录器"""
    def __init__(self):
        super().__init__('launch_controlled_recorder')
        
        # 数据记录状态
        self.recording_active = False
        self.recorder = None
        
        # 显示启动信息git 
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 数据采集器已启动并等待指令")
        self.get_logger().info("=" * 60)
        self.get_logger().info("📝 指令说明:")
        self.get_logger().info("   输入 '1' + Enter: 开始数据采集")
        self.get_logger().info("   输入 '2' + Enter: 停止数据采集")
        self.get_logger().info("   输入 'q' + Enter: 退出程序")
        self.get_logger().info("=" * 60)
        self.get_logger().info("⏳ 等待您的指令...")
        
        # 启动输入监听线程
        self.input_thread = threading.Thread(target=self.input_listener, daemon=True)
        self.input_thread.start()
        
        # 定期检查状态
        self.timer = self.create_timer(1.0, self.status_check)
    
    def input_listener(self):
        """监听用户输入的线程"""
        while rclpy.ok():
            try:
                user_input = input().strip()
                
                if user_input == '1':
                    if not self.recording_active:
                        self.start_recording()
                    else:
                        self.get_logger().warn("❌ 数据采集已在进行中")
                        
                elif user_input == '2':
                    if self.recording_active:
                        self.stop_recording()
                    else:
                        self.get_logger().warn("❌ 数据采集未在进行中")
                        
                elif user_input.lower() == 'q':
                    self.get_logger().info("🛑 正在退出程序...")
                    if self.recording_active:
                        self.stop_recording()
                    rclpy.shutdown()
                    break
                    
                else:
                    self.get_logger().info("❓ 无效指令。请输入 '1'(开始), '2'(停止), 或 'q'(退出)")
                    
            except EOFError:
                # 处理Ctrl+D
                break
            except KeyboardInterrupt:
                # 处理Ctrl+C
                self.get_logger().info("🛑 收到中断信号，正在退出...")
                if self.recording_active:
                    self.stop_recording()
                rclpy.shutdown()
                break
    
    def start_recording(self):
        """开始数据记录"""
        try:
            self.get_logger().info("🟢 开始启动数据采集器...")
            
            # 创建高级同步记录器
            self.recorder = AdvancedSyncBagRecorder(sync_window_ms=50, parent_node=self)
            self.recording_active = True
            
            self.get_logger().info("✅ 数据采集已开始!")
            self.get_logger().info(f"📁 数据保存路径: {self.recorder.unique_dir}")
            self.get_logger().info("💡 输入 '2' 停止采集")
            
        except Exception as e:
            error_msg = str(e)
            if "already exists" in error_msg:
                self.get_logger().warn("⚠️ 目录已存在，正在创建新的时间戳目录...")
                try:
                    # 重新尝试创建记录器（会生成新的时间戳）
                    import time
                    time.sleep(1)  # 等待1秒确保时间戳不同
                    self.recorder = AdvancedSyncBagRecorder(sync_window_ms=50, parent_node=self)
                    self.recording_active = True
                    self.get_logger().info("✅ 数据采集已开始!")
                    self.get_logger().info(f"📁 数据保存路径: {self.recorder.unique_dir}")
                    self.get_logger().info("💡 输入 '2' 停止采集")
                except Exception as e2:
                    self.get_logger().error(f"❌ 重试后仍然失败: {str(e2)}")
                    self.recording_active = False
            else:
                self.get_logger().error(f"❌ 启动数据采集失败: {error_msg}")
                self.recording_active = False
    
    def stop_recording(self):
        """停止数据记录"""
        try:
            self.get_logger().info("🟡 正在停止数据采集...")
            
            if self.recorder:
                # 生成分析报告
                self.recorder.generate_analysis_report()
                
                # 清理资源
                self.recorder.cleanup()
                
                self.get_logger().info("✅ 数据采集已停止!")
                self.get_logger().info(f"📁 数据已保存到: {self.recorder.unique_dir}")
                self.get_logger().info("💡 输入 '1' 重新开始采集")
                
                self.recorder = None
            
            self.recording_active = False
            
        except Exception as e:
            self.get_logger().error(f"❌ 停止数据采集失败: {str(e)}")
    
    def status_check(self):
        """定期状态检查"""
        if self.recording_active and self.recorder:
            # 显示简单的状态信息
            if hasattr(self.recorder, 'sync_group_counter'):
                count = self.recorder.sync_group_counter
                if count > 0 and count % 100 == 0:
                    self.get_logger().info(f"📊 已采集 {count} 组同步数据")


class AdvancedSyncBagRecorder:
    """改进的同步数据记录器，适配Launch模式"""
    def __init__(self, sync_window_ms=50, parent_node=None):
        self.parent_node = parent_node
        self.writer = rosbag2_py.SequentialWriter()
        self.sync_window_ns = sync_window_ms * 1e6
        
        # 使用缓冲区存储多个消息，用于时间同步
        self.msg_buffers = {}
        self.buffer_size = 100
        
        # 创建唯一目录结构
        current_workspace_path = os.getcwd()
        base_dir = os.path.join(current_workspace_path, 'training_data')
        
        # 生成唯一目录名，避免冲突
        unique_dir_name = self.generate_unique_dirname()
        self.unique_dir = os.path.join(base_dir, 'launch_recorder', unique_dir_name)
        
        # 确保目录不存在，如果存在则生成新的
        while os.path.exists(self.unique_dir):
            unique_dir_name = self.generate_unique_dirname()
            self.unique_dir = os.path.join(base_dir, 'launch_recorder', unique_dir_name)
        
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
        
        # 话题类型定义
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
        
        # 线程锁和计数器
        self.file_lock = threading.Lock()
        self.sync_group_counter = 0
        
        # 初始化消息缓冲区和订阅器
        self.setup_subscribers()
        
        # 启动同步定时器
        if self.parent_node:
            self.timer = self.parent_node.create_timer(0.02, self.sync_and_write)
    
    def setup_subscribers(self):
        """设置订阅器和数据文件"""
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

            if self.parent_node:
                self._subscriptions.append(self.parent_node.create_subscription(
                    msg_type,
                    topic_name,
                    self.buffered_callback(topic_name),
                    10
                ))
    
    def buffered_callback(self, topic_name):
        """创建带缓冲的回调函数"""
        def callback(msg):
            receive_time = self.parent_node.get_clock().now().nanoseconds
            
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
                    writer.writerow([timestamp, receive_time, header_time, msg.x, msg.y, msg.z])
                elif hasattr(msg, 'row1') and hasattr(msg, 'row2') and hasattr(msg, 'row3'):
                    num_points = len(msg.row1) if msg.row1 else 0
                    sample_points = f"[{msg.row1[:3] if msg.row1 else []}...]" if num_points > 0 else "[]"
                    writer.writerow([timestamp, receive_time, header_time, num_points, sample_points])
                elif hasattr(msg, 'data'):
                    writer.writerow([timestamp, receive_time, header_time, msg.data])
                else:
                    data_summary = str(msg)[:100] + "..." if len(str(msg)) > 100 else str(msg)
                    writer.writerow([timestamp, receive_time, header_time, data_summary])
                
                self.raw_data_files[topic_name].flush()
        except Exception as e:
            if self.parent_node:
                self.parent_node.get_logger().error(f"记录原始数据失败 ({topic_name}): {str(e)}")
    
    def sync_and_write(self):
        """基于时间窗口进行消息同步并写入"""
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
            
            for msg_entry in reversed(buffer):
                time_diff = abs(msg_entry['timestamp'] - sync_target)
                if time_diff < min_diff:
                    min_diff = time_diff
                    best_msg = msg_entry
                    
                if time_diff > self.sync_window_ns:
                    break
            
            if best_msg and min_diff <= self.sync_window_ns:
                synced_msgs[topic_name] = best_msg
                max_time_diff = max(max_time_diff, min_diff)
                
                data_summary = self.get_data_summary(best_msg['msg'])
                timing_records.append({
                    'topic_name': topic_name,
                    'receive_time': best_msg['receive_time'],
                    'header_time': best_msg['header_time'],
                    'time_diff_from_sync_target': min_diff / 1e6,
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
                
                self.timing_file.flush()
                self.sync_stats_file.flush()
                
        except Exception as e:
            if self.parent_node:
                self.parent_node.get_logger().error(f"记录时间分析失败: {str(e)}")
    
    def generate_analysis_report(self):
        """生成数据分析报告"""
        try:
            report_path = os.path.join(self.analysis_dir, 'analysis_report.json')
            
            report = {
                'session_info': {
                    'end_time': datetime.now().isoformat(),
                    'unique_dir': self.unique_dir,
                    'sync_window_ms': self.sync_window_ns / 1e6,
                    'buffer_size': self.buffer_size,
                    'total_sync_groups': self.sync_group_counter
                },
                'topics': {},
                'file_structure': {
                    'rosbag_dir': self.rosbag_dir,
                    'raw_data_dir': self.raw_data_dir,
                    'analysis_dir': self.analysis_dir,
                    'timing_analysis_file': os.path.join(self.analysis_dir, 'timing_analysis.csv'),
                    'sync_statistics_file': os.path.join(self.analysis_dir, 'sync_statistics.csv')
                }
            }
            
            for topic_name, topic_type in self.topic_types.items():
                safe_topic_name = topic_name.replace('/', '_')
                report['topics'][topic_name] = {
                    'type': topic_type,
                    'raw_data_file': os.path.join(self.raw_data_dir, f'{safe_topic_name}_raw.csv'),
                    'buffer_current_size': len(self.msg_buffers.get(topic_name, [])),
                    'buffer_max_size': self.buffer_size
                }
            
            with open(report_path, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            
            if self.parent_node:
                self.parent_node.get_logger().info(f"📋 分析报告已生成: {report_path}")
            
        except Exception as e:
            if self.parent_node:
                self.parent_node.get_logger().error(f"生成分析报告失败: {str(e)}")
    
    def cleanup(self):
        """清理资源"""
        try:
            for file_handle in self.raw_data_files.values():
                if not file_handle.closed:
                    file_handle.close()
            
            if hasattr(self, 'timing_file') and not self.timing_file.closed:
                self.timing_file.close()
            
            if hasattr(self, 'sync_stats_file') and not self.sync_stats_file.closed:
                self.sync_stats_file.close()
                
        except Exception as e:
            if self.parent_node:
                self.parent_node.get_logger().error(f"清理资源时出错: {str(e)}")
    
    def generate_unique_dirname(self):
        """生成唯一的目录名"""
        import time
        now = datetime.now()
        # 使用更精确的时间戳，包含微秒
        return now.strftime('%Y%m%d_%H%M%S_%f')[:-3]  # 去掉最后3位微秒，保留毫秒


def main(args=None):
    rclpy.init(args=args)
    
    # 创建Launch控制的记录器
    recorder = LaunchControlledRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("🛑 收到键盘中断信号")
    finally:
        if recorder.recording_active:
            recorder.stop_recording()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
