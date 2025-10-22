#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState

class CameraServoControlNode(Node):
    def __init__(self):
        super().__init__('camera_servo_control_node')
        
        # 发布者
        self.servo_pub = self.create_publisher(
            SetPWMServoState, 
            'ros_robot_controller/pwm_servo/set_state', 
            1
        )
        self.angles_pub = self.create_publisher(
            Int32MultiArray, 
            '/camera_servo/angles', 
            1
        )
        
        # 订阅手柄输入
        self.joy_sub = self.create_subscription(
            Joy, 
            '/joy', 
            self.joy_callback, 
            10
        )

        # 舵机参数设置
        self.tilt = 1500  # 当前俯仰角位置
        self.pan = 1500   # 当前平移角位置
        
        # 舵机范围
        self.tilt_min = 1000
        self.tilt_max = 2000
        self.pan_min = 1000
        self.pan_max = 2000
        
        # 舵机ID
        self.tilt_id = 1  # 俯仰舵机（垂直，Z轴）
        self.pan_id = 2   # 平移舵机（水平，Y轴）
        
        # 死区阈值（避免摇杆漂移）
        self.deadzone = 0.1
        
        # 运动持续时间（秒）- 控制最大速度
        # 值越小速度越快，建议范围：0.1-0.5
        self.movement_duration = 0.2
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('摄像头云台控制节点已启动')
        self.get_logger().info('=' * 60)
        self.get_logger().info('控制方式：右摇杆定位控制')
        self.get_logger().info('  - 右摇杆X轴 → 云台水平转动（Pan）')
        self.get_logger().info('  - 右摇杆Y轴 → 云台垂直转动（Tilt）')
        self.get_logger().info(f'舵机范围：{self.tilt_min}-{self.tilt_max}')
        self.get_logger().info(f'运动速度：{self.movement_duration}s 到达目标')
        self.get_logger().info(f'死区阈值：{self.deadzone}')
        self.get_logger().info('=' * 60)

    def joy_callback(self, msg):
        """
        处理手柄输入回调
        右摇杆轴索引（标准手柄）：
        - axes[2]: 右摇杆X轴（左右）范围 [-1.0, 1.0]
        - axes[3]: 右摇杆Y轴（上下）范围 [-1.0, 1.0]
        """
        
        # 检查 axes 数组长度是否足够
        if len(msg.axes) < 4:
            self.get_logger().warn(f'手柄axes数量不足：{len(msg.axes)}，需要至少4个')
            return
        
        # 读取右摇杆位置
        right_stick_x = msg.axes[2]  # 右摇杆X轴（左负右正）
        right_stick_y = msg.axes[3]  # 右摇杆Y轴（上负下正）
        
        # 应用死区（避免摇杆中心位置的微小漂移）
        if abs(right_stick_x) < self.deadzone:
            right_stick_x = 0.0
        if abs(right_stick_y) < self.deadzone:
            right_stick_y = 0.0
        
        # 计算目标舵机位置
        # 摇杆范围 [-1, 1] 映射到舵机范围 [min, max]
        # 
        # Pan（水平）：
        #   右摇杆向左(-1) → pan_min
        #   右摇杆向右(+1) → pan_max
        target_pan = self.map_value(
            right_stick_x, 
            -1.0, 1.0, 
            self.pan_min, self.pan_max
        )
        
        # Tilt（垂直）：
        #   右摇杆向上(-1) → tilt_max（摄像头向上）
        #   右摇杆向下(+1) → tilt_min（摄像头向下）
        #   注意：Y轴反转以符合直觉（向上推摇杆 = 摄像头向上）
        target_tilt = self.map_value(
            -right_stick_y,  # 反转Y轴
            -1.0, 1.0, 
            self.tilt_min, self.tilt_max
        )
        
        # 边界限制（确保在安全范围内）
        target_pan = max(self.pan_min, min(self.pan_max, int(target_pan)))
        target_tilt = max(self.tilt_min, min(self.tilt_max, int(target_tilt)))
        
        # 检查是否有位置变化（避免重复发送相同指令）
        position_changed = (target_pan != self.pan) or (target_tilt != self.tilt)
        
        if position_changed:
            # 更新内部状态
            self.pan = target_pan
            self.tilt = target_tilt
            
            # 发布控制指令
            self.publish_servo()
            self.publish_angles()
            
            # 日志输出
            self.get_logger().info(
                f'摇杆: X={right_stick_x:+.2f}, Y={right_stick_y:+.2f} | '
                f'目标位置: Pan={self.pan}, Tilt={self.tilt}'
            )

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """
        线性映射函数
        将输入范围 [in_min, in_max] 的值映射到输出范围 [out_min, out_max]
        """
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def publish_servo(self):
        """发布舵机控制指令"""
        # 创建俯仰舵机状态
        tilt = PWMServoState()
        tilt.id = [self.tilt_id]
        tilt.position = [self.tilt]
        
        # 创建平移舵机状态
        pan = PWMServoState()
        pan.id = [self.pan_id]
        pan.position = [self.pan]
        
        # 打包消息
        msg = SetPWMServoState()
        msg.state = [tilt, pan]
        msg.duration = self.movement_duration  # 恒定速度运动
        
        # 发布
        self.servo_pub.publish(msg)

    def publish_angles(self):
        """发布当前云台角度信息"""
        msg = Int32MultiArray()
        msg.data = [self.pan, self.tilt]
        self.angles_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraServoControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到退出信号')
    finally:
        node.get_logger().info('正在关闭节点...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
