import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from lbr_fri_idl.msg import LBRState, LBRWrenchCommand
import numpy as np
import math
from copy import deepcopy


class DynamicAdmittance(Node):
    def __init__(self):
        super().__init__("dynamic_admittance_controller")

        # --- 控制参数 ---
        self.teaching_threshold = 8.0  # 示教模式触发阈值 [N]
        self.normal_threshold = 3.0    # 正常操作阈值 [N]
        self.k_teaching = 0.05        # 示教模式下中心点移动速度
        self.k_normal = 0.01          # 正常模式下中心点移动速度
        self.k_stiff = 50.0            # 虚拟刚度
        self.d_damp = 5.0              # 虚拟阻尼
        self.teaching_decay = 0.95     # 示教信号衰减因子

        # 状态变量
        self.f_ext = np.zeros(6)
        self.f_bias = np.zeros(6)
        self.q = [0.0] * 7
        self.lbr_state = None
        self.lbr_wrench_command = LBRWrenchCommand()

        # 位置和速度估计
        self.vel = np.zeros(6)
        self.x_desired = np.zeros(6)  # 阻抗控制中心点（会根据示教缓慢移动）
        self.x_current = np.zeros(6)  # 当前估计位置
        
        # 示教相关
        self.teaching_signal = np.zeros(6)  # 示教信号强度
        self.is_teaching = False
        self.teaching_timer = 0

        # 获取控制器更新频率
        self._dt = None
        self._retrieve_update_rate()

        # 订阅和发布
        self.sub = self.create_subscription(LBRState, "state", self.state_cb, 1)
        self.pub = self.create_publisher(LBRWrenchCommand, "command/wrench", 1)
        self.srv = self.create_service(Trigger, "reset_zero", self.reset_cb)

        # 启动控制循环
        self.timer = self.create_timer(0.01, self.update)  # 临时使用0.01，等获取到真实频率后更新

    def state_cb(self, msg):
        """状态回调函数 - 与test_force.py相同的接口"""
        if self._dt is None:
            return
        
        # 更新状态
        if self.lbr_state is None:
            self.lbr_state = msg
        else:
            self.lbr_state = msg
            
        self.q = msg.measured_joint_position
        
        # 从外部扭矩估算末端外力
        tau = np.array(msg.external_torque)
        self.f_ext = tau[:6]  # 简化估算为末端外力
        
        # 复制关节位置到wrench命令
        self.lbr_wrench_command.joint_position = deepcopy(
            self.lbr_state.measured_joint_position
        )

    def reset_cb(self, request, response):
        """重置零点和示教状态"""
        self.f_bias = self.f_ext.copy()
        self.x_desired = self.x_current.copy()
        self.teaching_signal = np.zeros(6)
        self.is_teaching = False
        self.teaching_timer = 0
        self.vel = np.zeros(6)
        
        self.get_logger().info("🔄 手动重置: 零点、中心点、示教状态已重置")
        response.success = True
        response.message = "Zero point and teaching state reset successfully"
        return response



    def update(self):
        """主控制循环 - 阻抗控制 + 示教功能"""
        if self._dt is None or self.lbr_state is None:
            return
            
        # 只在COMMANDING_ACTIVE状态下进行控制
        if self.lbr_state.session_state != 4:  # KUKA::FRI::COMMANDING_ACTIVE == 4
            return
            
        # 计算净外力
        f_net = self.f_ext - self.f_bias
        f_magnitude = np.linalg.norm(f_net[:3])
        
        # 判断是否进入示教模式
        if f_magnitude > self.teaching_threshold:
            self.is_teaching = True
            self.teaching_timer = 0
            # 示教信号强度与外力成正比
            self.teaching_signal = f_net * min(1.0, f_magnitude / (2 * self.teaching_threshold))
            
            # 在示教模式下，中心点快速跟随外力方向移动
            self.x_desired += self.k_teaching * self.teaching_signal * self._dt
            
            # self.get_logger().info(f"🎯 示教模式: F_mag={f_magnitude:.2f}N, 中心点移动: {self.x_desired[:3]}")
            
        elif f_magnitude > self.normal_threshold:
            # 正常操作模式 - 中心点缓慢移动
            if self.is_teaching:
                self.teaching_timer += self._dt
                # 示教信号逐渐衰减
                self.teaching_signal *= self.teaching_decay
                
                # 如果示教信号衰减到很小，退出示教模式
                if np.linalg.norm(self.teaching_signal) < 0.1:
                    self.is_teaching = False
                    self.get_logger().info("🔄 退出示教模式，进入正常操作")
            
            # 正常模式下的中心点慢速调整
            self.x_desired += self.k_normal * f_net * self._dt
            
        else:
            # 外力很小，逐渐停止示教
            if self.is_teaching:
                self.teaching_timer += self._dt
                self.teaching_signal *= self.teaching_decay
                
                if self.teaching_timer > 1.0:  # 1秒后退出示教模式
                    self.is_teaching = False
                    self.teaching_signal = np.zeros(6)
                    self.get_logger().info("⏹️ 示教结束，保持当前中心点")

        # 阻抗控制计算
        # 目标：让当前位置跟随阻抗控制中心点
        position_error = self.x_desired - self.x_current
        
        # 标准阻抗控制
        f_cmd = self.k_stiff * position_error - self.d_damp * self.vel
            
        # 限制输出力，避免系统不稳定
        f_cmd = np.clip(f_cmd, -100.0, 100.0)
        
        # 更新系统状态
        self.vel += f_cmd * self._dt
        self.vel = np.clip(self.vel, -0.5, 0.5)
        self.x_current += self.vel * self._dt
        
        # 防止数值异常
        if np.any(np.isnan(f_cmd)) or np.any(np.isinf(f_cmd)):
            self.get_logger().error("❌ 数值异常！重置系统状态")
            self.vel = np.zeros(6)
            self.x_current = np.zeros(6)
            return
        
        # 发布控制命令
        self.lbr_wrench_command.wrench = f_cmd.tolist()
        self.pub.publish(self.lbr_wrench_command)
        
        # 定期状态输出
        if int(self.get_clock().now().nanoseconds / 1e9) % 2 == 0:
            mode = "🎯示教" if self.is_teaching else "🔧正常"
            
            # 格式化外力显示
            f_ext_str = f"[{f_net[0]:.2f}, {f_net[1]:.2f}, {f_net[2]:.2f}]"
            
            # 格式化控制输出显示
            f_cmd_str = f"[{f_cmd[0]:.2f}, {f_cmd[1]:.2f}, {f_cmd[2]:.2f}]"
            
            self.get_logger().info(
                f"{mode} | 外力: {f_ext_str}N (幅值: {f_magnitude:.2f}N) | "
                f"控制输出: {f_cmd_str}N"
            )

    def _retrieve_update_rate(self) -> None:
        """获取控制器更新频率"""
        try:
            parameter_client = self.create_client(
                GetParameters, "controller_manager/get_parameters"
            )
            parameter_name = "update_rate"
            
            # 等待服务可用
            while not parameter_client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    raise RuntimeError("Interrupted while waiting for service.")
                self.get_logger().info(f"Waiting for {parameter_client.srv_name}...")
            
            # 调用服务获取参数
            future = parameter_client.call_async(
                GetParameters.Request(names=[parameter_name])
            )
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is None:
                self.get_logger().warn(f"Failed to get parameter '{parameter_name}', using default 100 Hz")
                self._dt = 0.01
            else:
                update_rate = future.result().values[0].integer_value
                self.get_logger().info(f"{parameter_name}: {update_rate} Hz")
                self._dt = 1.0 / float(update_rate)
                
                # 更新定时器频率
                if hasattr(self, 'timer'):
                    self.timer.cancel()
                self.timer = self.create_timer(self._dt, self.update)
                
        except Exception as e:
            self.get_logger().warn(f"Error retrieving update rate: {str(e)}, using default 100 Hz")
            self._dt = 0.005


def main(args=None):
    rclpy.init(args=args)
    node = DynamicAdmittance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

