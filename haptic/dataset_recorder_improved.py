import os
import rclpy
from rclpy.node import Node
from lbr_fri_idl.msg import LBRWrenchCommand, LBRState
from sensor_msgs.msg import JointState
from tutorial_interfaces.msg import Array3, Cloud
from std_msgs.msg import Float32
from datetime import datetime
import csv
import threading

import os
import rclpy
from rclpy.node import Node
from lbr_fri_idl.msg import LBRWrenchCommand, LBRState
from sensor_msgs.msg import JointState
from tutorial_interfaces.msg import Array3, Cloud
from std_msgs.msg import Float32
from datetime import datetime
import csv
import threading
import uuid
import time

class DatasetRecorder(Node):
    """简单的数据记录器，记录各个话题的数据和到达时间戳"""
    def __init__(self):
        super().__init__('dataset_recorder')
        
        # 创建唯一目录结构
        current_workspace_path = os.getcwd()
        base_dir = os.path.join(current_workspace_path, 'training_data')
        
        now = datetime.now()
        timestamp = now.strftime('%Y%m%d_%H%M%S_%f')[:-3]  # 包含毫秒
        process_id = os.getpid()
        unique_id = str(uuid.uuid4())[:8]
        unique_dir_name = f"{timestamp}_{process_id}_{unique_id}"
        
        self.unique_dir = os.path.join(base_dir, 'dataset_recorder', unique_dir_name)
        os.makedirs(self.unique_dir, exist_ok=True)
        
        # 线程锁，确保文件写入安全
        self.file_lock = threading.Lock()
        
        # 话题类型定义
        self.topic_types = {
            # 机器人相关话题
            '/lbr/command/wrench': 'lbr_fri_idl/msg/LBRWrenchCommand',
            '/lbr/state': 'lbr_fri_idl/msg/LBRState',
            '/lbr/joint_states': 'sensor_msgs/msg/JointState',
            
            # Tac3D传感器话题 - 根据修改后的tac3d_r和tac3d_l更新
            '/positions_r': 'tutorial_interfaces/msg/Cloud',
            '/displacements_r': 'tutorial_interfaces/msg/Cloud',
            '/forces_r': 'tutorial_interfaces/msg/Cloud',  # 修改为Cloud类型
            '/resultant_force_r': 'tutorial_interfaces/msg/Array3',  # 修改为Array3类型
            '/resultant_moment_r': 'tutorial_interfaces/msg/Array3',  # 修改为Array3类型
            '/index_r': 'std_msgs/msg/Float32',
            
            '/positions_l': 'tutorial_interfaces/msg/Cloud',
            '/displacements_l': 'tutorial_interfaces/msg/Cloud',
            '/forces_l': 'tutorial_interfaces/msg/Cloud',  # 修改为Cloud类型
            '/resultant_force_l': 'tutorial_interfaces/msg/Array3',  # 修改为Array3类型
            '/resultant_moment_l': 'tutorial_interfaces/msg/Array3',  # 修改为Array3类型
            '/index_l': 'std_msgs/msg/Float32'
        }
        
        # 为每个话题创建数据文件 - 全部使用txt格式
        self.data_files = {}
        
        for topic_name, topic_type in self.topic_types.items():
            # 创建安全的文件名
            safe_topic_name = topic_name.replace('/', '_').replace(':', '_')
            
            # 所有文件都使用txt格式
            file_path = os.path.join(self.unique_dir, f'{safe_topic_name}.txt')
            self.data_files[topic_name] = open(file_path, 'w')
        
        # 创建订阅
        self._subscriptions = []
        
        # 为每个话题创建订阅
        for topic_name, topic_type in self.topic_types.items():
            msg_type = eval(topic_type.split('/')[-1])
            
            self._subscriptions.append(self.create_subscription(
                msg_type,
                topic_name,
                self.create_callback(topic_name),
                10
            ))
        
        self.get_logger().info(f"数据记录器已启动，数据将保存到: {self.unique_dir}")
        self.get_logger().info(f"监控 {len(self.topic_types)} 个话题")
        
    def create_callback(self, topic_name):
        """为指定话题创建回调函数"""
        def callback(msg):
            receive_time = self.get_clock().now().nanoseconds
            
            # 获取消息时间戳
            header_time = None
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                header_time = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            
            # 根据消息类型使用不同的写入方式
            topic_type = self.topic_types[topic_name]
            
            with self.file_lock:
                if 'Cloud' in topic_type:
                    # Cloud类型写入txt格式
                    self.write_cloud_data(topic_name, msg, receive_time, header_time)
                elif 'Array3' in topic_type:
                    # Array3类型写入txt格式
                    self.write_array3_data(topic_name, msg, receive_time)
                else:
                    # 其他类型写入txt格式
                    self.write_other_data(topic_name, msg, receive_time)
                
        return callback
    
    def write_array3_data(self, topic_name, msg, receive_time):
        """写入Array3类型的数据到txt文件 - 格式: 时间戳 换行 X 换行 Y 换行 Z 换行 30个* 换行"""
        try:
            file_handle = self.data_files[topic_name]
            
            # 写入时间戳
            file_handle.write(f"{receive_time}\n")
            
            # 提取并清理XYZ数据
            if hasattr(msg, 'x') and hasattr(msg, 'y') and hasattr(msg, 'z'):
                x_val = self.clean_single_float(msg.x)
                y_val = self.clean_single_float(msg.y)
                z_val = self.clean_single_float(msg.z)
                
                # 写入X, Y, Z，每个数据单独一行
                file_handle.write(f"{x_val:.6f}\n")
                file_handle.write(f"{y_val:.6f}\n")
                file_handle.write(f"{z_val:.6f}\n")
            else:
                # 无效数据时写入-99
                file_handle.write("-99.000000\n")
                file_handle.write("-99.000000\n")
                file_handle.write("-99.000000\n")
            
            # 写入30个*号作为分隔符
            file_handle.write("*" * 30 + "\n")
            file_handle.flush()
            
        except Exception as e:
            # 出错时写入错误信息
            file_handle = self.data_files[topic_name]
            file_handle.write(f"{receive_time}\n")
            file_handle.write("-99.000000\n")
            file_handle.write("-99.000000\n") 
            file_handle.write("-99.000000\n")
            file_handle.write("*" * 30 + "\n")
            file_handle.flush()

    def write_other_data(self, topic_name, msg, receive_time):
        """写入其他类型数据到txt文件"""
        try:
            file_handle = self.data_files[topic_name]
            
            # 写入时间戳
            file_handle.write(f"{receive_time}\n")
            
            # 根据消息类型提取数据
            if hasattr(msg, 'joint_position') and hasattr(msg, 'wrench'):
                # LBRWrenchCommand类型 - 7个关节位置 + 6个力/力矩值
                # 先写入7个关节位置
                for pos in msg.joint_position:
                    cleaned_pos = self.clean_single_float(pos)
                    file_handle.write(f"{cleaned_pos:.6f}\n")
                
                # 写入分隔换行
                file_handle.write("\n")
                
                # 再写入6个扭矩值
                for wrench_val in msg.wrench:
                    cleaned_wrench = self.clean_single_float(wrench_val)
                    file_handle.write(f"{cleaned_wrench:.6f}\n")
            elif hasattr(msg, 'sample_time'):
                # LBRState类型 - 复杂的状态数据
                # 写入所有状态和关节数据
                data_list = [
                    msg.sample_time, msg.session_state, msg.connection_quality,
                    msg.safety_state, msg.operation_mode, msg.drive_state,
                    msg.client_command_mode, msg.overlay_type, msg.control_mode,
                    msg.time_stamp_sec, msg.time_stamp_nano_sec
                ]
                data_list.extend(msg.measured_joint_position)
                data_list.extend(msg.commanded_joint_position)
                data_list.extend(msg.measured_torque)
                data_list.extend(msg.commanded_torque)
                data_list.extend(msg.external_torque)
                data_list.extend(msg.ipo_joint_position)
                data_list.append(msg.tracking_performance)
                
                for val in data_list:
                    cleaned_val = self.clean_single_float(val)
                    file_handle.write(f"{cleaned_val:.6f}\n")
            elif hasattr(msg, 'position') and hasattr(msg, 'velocity'):
                # JointState类型
                # 处理位置数据
                pos = list(msg.position) if msg.position else []
                vel = list(msg.velocity) if msg.velocity else []
                eff = list(msg.effort) if hasattr(msg, 'effort') and msg.effort else []
                
                # 确保有7个关节的数据
                while len(pos) < 7:
                    pos.append(0.0)
                while len(vel) < 7:
                    vel.append(0.0)
                while len(eff) < 7:
                    eff.append(0.0)
                
                # 写入所有数据
                for val in (pos[:7] + vel[:7] + eff[:7]):
                    cleaned_val = self.clean_single_float(val)
                    file_handle.write(f"{cleaned_val:.6f}\n")
            elif hasattr(msg, 'data'):
                # Float32类型
                cleaned_val = self.clean_single_float(msg.data)
                file_handle.write(f"{cleaned_val:.6f}\n")
            else:
                # 其他未知类型
                file_handle.write("unknown_type\n")
            
            # 写入30个*号作为分隔符
            file_handle.write("*" * 30 + "\n")
            file_handle.flush()
            
        except Exception as e:
            # 出错时写入错误信息
            file_handle = self.data_files[topic_name]
            file_handle.write(f"{receive_time}\n")
            file_handle.write("error_data\n")
            file_handle.write("*" * 30 + "\n")
            file_handle.flush()

    def extract_csv_data(self, msg, receive_time):
        """提取消息数据为CSV行数据"""
        try:
            row = [receive_time]
            
            # 根据消息类型提取数据
            if hasattr(msg, 'joint_position') and hasattr(msg, 'wrench'):
                # LBRWrenchCommand类型
                row.extend(msg.joint_position)
                row.extend(msg.wrench)
            elif hasattr(msg, 'sample_time'):
                # LBRState类型
                row.extend([
                    msg.sample_time, msg.session_state, msg.connection_quality,
                    msg.safety_state, msg.operation_mode, msg.drive_state,
                    msg.client_command_mode, msg.overlay_type, msg.control_mode,
                    msg.time_stamp_sec, msg.time_stamp_nano_sec
                ])
                row.extend(msg.measured_joint_position)
                row.extend(msg.commanded_joint_position)
                row.extend(msg.measured_torque)
                row.extend(msg.commanded_torque)
                row.extend(msg.external_torque)
                row.extend(msg.ipo_joint_position)
                row.append(msg.tracking_performance)
            elif hasattr(msg, 'position') and hasattr(msg, 'velocity'):
                # JointState类型
                # 确保有7个关节的数据，不足的用0填充
                pos = list(msg.position) if msg.position else []
                vel = list(msg.velocity) if msg.velocity else []
                eff = list(msg.effort) if hasattr(msg, 'effort') and msg.effort else []
                
                # 填充到7个关节
                while len(pos) < 7:
                    pos.append(0.0)
                while len(vel) < 7:
                    vel.append(0.0)
                while len(eff) < 7:
                    eff.append(0.0)
                
                row.extend(pos[:7])  # 只取前7个
                row.extend(vel[:7])
                row.extend(eff[:7])
            elif hasattr(msg, 'x') and hasattr(msg, 'y') and hasattr(msg, 'z'):
                # Array3类型
                row.extend([msg.x, msg.y, msg.z])
            elif hasattr(msg, 'data'):
                # Float32类型
                row.append(msg.data)
            else:
                # 其他类型，作为字符串处理
                row.append(str(msg)[:200])
                
            return row
        except Exception as e:
            # 出错时返回错误信息
            return [receive_time, f"extract_error: {str(e)}"]
    
    def extract_data(self, msg):
        """提取消息数据为字符串（保留用于兼容性）"""
        try:
            if hasattr(msg, 'x') and hasattr(msg, 'y') and hasattr(msg, 'z'):
                # Array3 类型
                return f"x={msg.x}, y={msg.y}, z={msg.z}"
            elif hasattr(msg, 'row1') and hasattr(msg, 'row2') and hasattr(msg, 'row3'):
                # Cloud 类型
                try:
                    num_points = len(msg.row1) if msg.row1 is not None else 0
                    return f"points={num_points}"
                except:
                    return "points=0"
            elif hasattr(msg, 'data'):
                # Float32 类型
                return str(msg.data)
            elif hasattr(msg, 'position') and hasattr(msg, 'velocity'):
                # JointState 类型
                try:
                    pos_str = ','.join(map(str, msg.position)) if msg.position is not None else ""
                    vel_str = ','.join(map(str, msg.velocity)) if msg.velocity is not None else ""
                    return f"pos=[{pos_str}], vel=[{vel_str}]"
                except:
                    return "pos=[], vel=[]"
            else:
                # 其他类型，返回字符串表示
                return str(msg)[:200]  # 限制长度
        except Exception as e:
            return f"extract_error: {str(e)}"
    
    def write_cloud_data(self, topic_name, msg, receive_time, header_time):
        """写入Cloud类型的数据到txt文件 - 20*20*3点阵，时间戳+XYZ分别换行+*号分隔"""
        try:
            file_handle = self.data_files[topic_name]
            
            # 写入时间戳信息
            file_handle.write(f"{receive_time}\n")
            
            # 处理Cloud消息结构 - 每个row包含400个点的坐标
            x_data = []
            y_data = []
            z_data = []
            
            if hasattr(msg, 'row1') and hasattr(msg, 'row2') and hasattr(msg, 'row3'):
                # 处理X数据 (row1) - 400个X坐标
                if msg.row1 is not None:
                    x_raw = list(msg.row1)[:400]  # 取前400个点
                    x_data = self.clean_float_array(x_raw, 400)
                else:
                    x_data = [-99.0] * 400
                
                # 处理Y数据 (row2) - 400个Y坐标
                if msg.row2 is not None:
                    y_raw = list(msg.row2)[:400]  # 取前400个点
                    y_data = self.clean_float_array(y_raw, 400)
                else:
                    y_data = [-99.0] * 400
                
                # 处理Z数据 (row3) - 400个Z坐标
                if msg.row3 is not None:
                    z_raw = list(msg.row3)[:400]  # 取前400个点
                    z_data = self.clean_float_array(z_raw, 400)
                else:
                    z_data = [-99.0] * 400
            else:
                # 如果没有有效数据，创建三组400个-99
                x_data = [-99.0] * 400
                y_data = [-99.0] * 400
                z_data = [-99.0] * 400
            
            # 写入XYZ三行数据，每行400个点，用空格分隔
            x_str = ' '.join(f"{value:.6f}" for value in x_data)
            file_handle.write(f"{x_str}\n")
            
            y_str = ' '.join(f"{value:.6f}" for value in y_data)
            file_handle.write(f"{y_str}\n")
            
            z_str = ' '.join(f"{value:.6f}" for value in z_data)
            file_handle.write(f"{z_str}\n")
            
            # 写入*号分隔符
            file_handle.write("*\n")
            file_handle.flush()
            
        except Exception as e:
            # 出错时写入错误信息
            file_handle = self.data_files[topic_name]
            file_handle.write(f"{receive_time}\n")
            # 写入400个-99的XYZ数据
            error_line = ' '.join(["-99.000000"] * 400)
            file_handle.write(f"{error_line}\n")
            file_handle.write(f"{error_line}\n") 
            file_handle.write(f"{error_line}\n")
            file_handle.write("*\n")
            file_handle.flush()
    
    def clean_single_float(self, value):
        """清理单个浮点数，处理NaN和无穷大等奇异值"""
        import math
        
        try:
            # 转换为浮点数
            float_val = float(value)
            
            # 检查是否为NaN或无穷大
            if math.isnan(float_val) or math.isinf(float_val):
                return -99.0
            # 检查是否为异常大的数值（绝对值超过1e10）
            elif abs(float_val) > 1e10:
                return -99.0
            else:
                return float_val
        except (ValueError, TypeError):
            # 转换失败时返回-99
            return -99.0

    def clean_float_array(self, raw_data, target_length):
        """清理浮点数组，处理NaN和无穷大等奇异值"""
        import math
        
        cleaned_data = []
        for value in raw_data:
            try:
                # 转换为浮点数
                float_val = float(value)
                
                # 检查是否为NaN或无穷大
                if math.isnan(float_val) or math.isinf(float_val):
                    cleaned_data.append(-99.0)
                # 检查是否为异常大的数值（绝对值超过1e10）
                elif abs(float_val) > 1e10:
                    cleaned_data.append(-99.0)
                else:
                    cleaned_data.append(float_val)
            except (ValueError, TypeError):
                # 转换失败时用-99填充
                cleaned_data.append(-99.0)
        
        # 确保数组长度正确
        while len(cleaned_data) < target_length:
            cleaned_data.append(-99.0)
        
        return cleaned_data[:target_length]
    
    def generate_summary(self):
        """生成数据采集摘要"""
        summary_file = os.path.join(self.unique_dir, 'summary.txt')
        with open(summary_file, 'w') as f:
            f.write("数据采集摘要\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"采集时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"数据目录: {self.unique_dir}\n")
            f.write(f"监控话题数: {len(self.topic_types)}\n\n")
            
            f.write("话题列表和数据格式:\n")
            for topic_name, topic_type in self.topic_types.items():
                safe_name = topic_name.replace('/', '_').replace(':', '_')
                if 'Cloud' in topic_type:
                    f.write(f"  - {topic_name} -> {safe_name}.txt (Cloud数据: 时间戳+400个点的XYZ坐标，每组以*分隔)\n")
                elif 'Array3' in topic_type:
                    f.write(f"  - {topic_name} -> {safe_name}.txt (Array3数据: 时间戳+X+Y+Z，每组以30个*分隔)\n")
                elif 'LBRWrenchCommand' in topic_type:
                    f.write(f"  - {topic_name} -> {safe_name}.txt (Wrench数据: 时间戳+7个关节位置+换行+6个力/力矩值，每组以30个*分隔)\n")
                else:
                    f.write(f"  - {topic_name} -> {safe_name}.txt (其他数据: 时间戳+数据值，每组以30个*分隔)\n")
            
            f.write(f"\n数据格式说明:\n")
            f.write(f"- Cloud数据: 时间戳 换行 400个X坐标(空格分隔) 换行 400个Y坐标 换行 400个Z坐标 换行 * 换行\n")
            f.write(f"- Array3数据: 时间戳 换行 X 换行 Y 换行 Z 换行 30个* 换行\n")
            f.write(f"- Wrench数据: 时间戳 换行 7个关节位置(每行一个) 换行 换行 6个力/力矩值(每行一个) 换行 30个* 换行\n")
            f.write(f"- 其他数据: 时间戳 换行 数据值(每行一个) 换行 30个* 换行\n")
            f.write(f"- 所有非法数据(NaN, 无穷大, 绝对值>1e10)记录为-99\n")
        
        self.get_logger().info(f"摘要已生成: {summary_file}")

    def destroy_node(self):
        """重写销毁节点方法，确保正确清理订阅"""
        try:
            # 清理订阅
            if hasattr(self, '_subscriptions'):
                for subscription in self._subscriptions:
                    try:
                        self.destroy_subscription(subscription)
                    except Exception as e:
                        self.get_logger().warn(f"清理订阅时出错: {str(e)}")
                self._subscriptions.clear()
            
            # 关闭文件
            for file_handle in self.data_files.values():
                if not file_handle.closed:
                    file_handle.close()
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().error(f"销毁节点时出错: {str(e)}")
        finally:
            # 调用父类的销毁方法
            super().destroy_node()
    
    def __del__(self):
        """析构函数，确保文件正确关闭"""
        try:
            for file_handle in self.data_files.values():
                if not file_handle.closed:
                    file_handle.close()
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().error(f"关闭文件时出错: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    print("启动数据记录器...")
    print("功能: 记录各个话题的数据和到达时间戳")
    print()
    
    recorder = None
    try:
        recorder = DatasetRecorder()
        print(f"数据将保存到: {recorder.unique_dir}")
        print("按 Ctrl+C 停止记录")
        print()
        
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        print("\n正在停止数据记录...")
    except Exception as e:
        print(f"启动失败: {e}")
    finally:
        if recorder is not None:
            print("正在生成摘要...")
            recorder.generate_summary()
            print(f"所有数据已保存到: {recorder.unique_dir}")
            try:
                recorder.destroy_node()
            except Exception as e:
                print(f"销毁节点时出错: {e}")
        
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"关闭ROS时出错: {e}")


if __name__ == '__main__':
    main()
