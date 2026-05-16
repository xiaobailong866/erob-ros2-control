import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

# 引入 TF2 库来获取末端真实位姿
from tf2_ros import Buffer, TransformListener

from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout, 
                             QWidget, QHBoxLayout, QGroupBox, QTabWidget,
                             QDoubleSpinBox, QPushButton, QGridLayout)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot


# 辅助函数：欧拉角与四元数互相转换
def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy
    q[1] = cr * sp * cy + sr * cp * sy
    q[2] = cr * cp * sy - sr * sp * cy
    q[3] = cr * cp * cy + sr * sp * sy
    return q

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

# 1. ROS 2 后台线程
class ROS2Thread(QThread):
    update_state_signal = pyqtSignal(list)
    update_cartesian_signal = pyqtSignal(list) 

    def __init__(self):
        super().__init__()
        self.node = None

    def run(self):
        rclpy.init()
        self.node = rclpy.create_node('qt_ros2_control_node')
        
        self.sub = self.node.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)
        
        self.joint_pub = self.node.create_publisher(
            JointTrajectory, '/left_arm_controller/joint_trajectory', 10)

        self.pose_pub = self.node.create_publisher(
            PoseStamped, '/left_arm_node/teleop_pose', 10)
            
        # TF2 监听器，用于获取末端真实位姿 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.timer = self.node.create_timer(0.1, self.tf_callback) # 10Hz 刷新频率
        
        self.node.get_logger().info("UI 后台已启动，正在监听 TF 树...")

        try:
            rclpy.spin(self.node) 
        except Exception:
            pass
        finally:
            if self.node:
                self.node.destroy_node()
    def tf_callback(self):
        """定时获取末端相对于基座的绝对坐标"""
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'L_wrist_3', rclpy.time.Time())
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            
            roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)
            
            # 将弧度转为度数发给UI
            self.update_cartesian_signal.emit([x, y, z, math.degrees(roll), math.degrees(pitch), math.degrees(yaw)])
        except Exception as e:
            # TF 还没准备好时忽略报错
            pass

    def state_callback(self, msg):
        target_names = [
            'L_Joint_1', 'L_Joint_2', 'L_Joint_3', 
            'L_Joint_4', 'L_Joint_5', 'L_Joint_6', 'L_Joint_7'
        ]
        if all(name in msg.name for name in target_names):
            positions = []
            for name in target_names:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            self.update_state_signal.emit(positions)

    def send_joint_command(self, target_positions):
        if self.node:
            msg = JointTrajectory()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.joint_names = [
                'L_Joint_1', 'L_Joint_2', 'L_Joint_3', 
                'L_Joint_4', 'L_Joint_5', 'L_Joint_6', 'L_Joint_7'
            ]
            point = JointTrajectoryPoint()
            point.positions = target_positions
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 500000000 
            msg.points.append(point)
            self.joint_pub.publish(msg)

    def send_relative_pose_command(self, dx, dy, dz, d_roll, d_pitch, d_yaw):
        if self.node:
            msg = PoseStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            
            msg.pose.position.x = float(dx)
            msg.pose.position.y = float(dy)
            msg.pose.position.z = float(dz)
            
            q = quaternion_from_euler(math.radians(float(d_roll)), math.radians(float(d_pitch)), math.radians(float(d_yaw)))
            msg.pose.orientation.x = float(q[0])
            msg.pose.orientation.y = float(q[1])
            msg.pose.orientation.z = float(q[2])
            msg.pose.orientation.w = float(q[3])

            self.pose_pub.publish(msg)

    def stop(self):
        if rclpy.ok():
            rclpy.shutdown()
        self.quit()

# 2. PyQt 主窗口类
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("单臂工业示教器 (全状态监控版)")
        # --- 修改：扩大 UI 界面尺寸 ---
        self.resize(900, 700)

        self.current_joints = [0.0] * 7

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.tab_joint = QWidget()
        self.tab_cartesian = QWidget()
        
        self.tabs.addTab(self.tab_joint, "关节示教 (Joint Jogging)")
        self.tabs.addTab(self.tab_cartesian, "末端示教 (Cartesian Jogging)")

        self.init_joint_tab()
        self.init_cartesian_tab()

        self.ros_thread = ROS2Thread()
        self.ros_thread.update_state_signal.connect(self.update_feedback_ui)
        # 绑定末端反馈信号
        self.ros_thread.update_cartesian_signal.connect(self.update_cartesian_feedback_ui)
        self.ros_thread.start()

    # 1：关节按键控制
    def init_joint_tab(self):
        layout = QVBoxLayout(self.tab_joint)
        
        step_layout = QHBoxLayout()
        step_layout.addWidget(QLabel("关节转动步长 (度/Deg):"))
        self.joint_step_box = QDoubleSpinBox()
        self.joint_step_box.setDecimals(1)
        self.joint_step_box.setSingleStep(1.0)
        self.joint_step_box.setValue(5.0) 
        step_layout.addWidget(self.joint_step_box)
        step_layout.addStretch()
        layout.addLayout(step_layout)

        group = QGroupBox("7轴点动控制")
        grid = QGridLayout()
        self.joint_fb_labels = []

        for i in range(7):
            grid.addWidget(QLabel(f"Joint {i+1}:"), i, 0)
            
            btn_minus = QPushButton(f"J{i+1} -")
            btn_minus.setMinimumHeight(45) 
            btn_minus.clicked.connect(lambda checked, idx=i, d=-1: self.on_joint_jog(idx, d))
            grid.addWidget(btn_minus, i, 1)
            
            btn_plus = QPushButton(f"J{i+1} +")
            btn_plus.setMinimumHeight(45)
            btn_plus.clicked.connect(lambda checked, idx=i, d=1: self.on_joint_jog(idx, d))
            grid.addWidget(btn_plus, i, 2)
            
            # 单位改为度 (°)
            fb_label = QLabel("Fb: 0.00°")
            fb_label.setStyleSheet("color: #0078D7; font-weight: bold; font-size: 14px;")
            self.joint_fb_labels.append(fb_label)
            grid.addWidget(fb_label, i, 3)

        group.setLayout(grid)
        layout.addWidget(group)
        layout.addStretch()

    # 2：末端按键控制
    def init_cartesian_tab(self):
        layout = QVBoxLayout(self.tab_cartesian)
        
        # 末端实时位姿显示区
        fb_group = QGroupBox("实时末端绝对位姿反馈 (TF2)")
        fb_layout = QHBoxLayout()
        self.cart_fb_labels = {}
        axes_labels = ['X (m)', 'Y (m)', 'Z (m)', 'Roll (°)', 'Pitch (°)', 'Yaw (°)']
        
        for axis in axes_labels:
            label = QLabel(f"{axis}:\n0.000")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("color: #D83B01; font-weight: bold; font-size: 16px; background-color: #F3F2F1; border-radius: 5px; padding: 10px;")
            self.cart_fb_labels[axis] = label
            fb_layout.addWidget(label)
            
        fb_group.setLayout(fb_layout)
        layout.addWidget(fb_group)
        
        # 步长设置
        step_group = QGroupBox("步长设置")
        step_layout = QHBoxLayout()
        step_layout.addWidget(QLabel("直线位移 (m):"))
        self.trans_step_box = QDoubleSpinBox()
        self.trans_step_box.setDecimals(3)
        self.trans_step_box.setSingleStep(0.001)
        self.trans_step_box.setValue(0.010)
        step_layout.addWidget(self.trans_step_box)
        
        step_layout.addWidget(QLabel(" 旋转角度 (deg):"))
        self.rot_step_box = QDoubleSpinBox()
        self.rot_step_box.setDecimals(1)
        self.rot_step_box.setSingleStep(1.0)
        self.rot_step_box.setValue(5.0)
        step_layout.addWidget(self.rot_step_box)
        step_group.setLayout(step_layout)
        layout.addWidget(step_group)

        # 按键点动区
        btn_group = QGroupBox("空间6自由度点动")
        grid = QGridLayout()
        
        axes = [('X 轴 (前后)', 'X'), ('Y 轴 (左右)', 'Y'), ('Z 轴 (上下)', 'Z'),
                ('Roll (滚转)', 'Roll'), ('Pitch (俯仰)', 'Pitch'), ('Yaw (偏航)', 'Yaw')]
        
        for i, (label, axis) in enumerate(axes):
            grid.addWidget(QLabel(label), i, 0)
            
            btn_minus = QPushButton(f"- {axis}")
            btn_minus.setMinimumHeight(45)
            btn_minus.clicked.connect(lambda checked, a=axis, d=-1: self.on_cart_jog(a, d))
            grid.addWidget(btn_minus, i, 1)
            
            btn_plus = QPushButton(f"+ {axis}")
            btn_plus.setMinimumHeight(45)
            btn_plus.clicked.connect(lambda checked, a=axis, d=1: self.on_cart_jog(a, d))
            grid.addWidget(btn_plus, i, 2)

        btn_group.setLayout(grid)
        layout.addWidget(btn_group)
        
        tip = QLabel("注意: 末端坐标由 ROS 2 TF树实时计算反馈。请等待动作执行完毕后再进行下一次点击。")
        tip.setStyleSheet("color: #666666;")
        layout.addWidget(tip)

    # --- 槽函数 ---
    def on_joint_jog(self, joint_idx, direction):
        step_deg = self.joint_step_box.value()
        step_rad = math.radians(step_deg)
        target_positions = list(self.current_joints)
        target_positions[joint_idx] += direction * step_rad
        self.ros_thread.send_joint_command(target_positions)

    def on_cart_jog(self, axis, direction):
        t_step = self.trans_step_box.value()
        r_step = self.rot_step_box.value()
        dx, dy, dz, dr, dp, dyaw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        
        if axis == 'X': dx = direction * t_step
        elif axis == 'Y': dy = direction * t_step
        elif axis == 'Z': dz = direction * t_step
        elif axis == 'Roll': dr = direction * r_step
        elif axis == 'Pitch': dp = direction * r_step
        elif axis == 'Yaw': dyaw = direction * r_step
            
        self.ros_thread.send_relative_pose_command(dx, dy, dz, dr, dp, dyaw)

    @pyqtSlot(list)
    def update_feedback_ui(self, real_positions):
        """缓存真实位置并刷新 UI，将弧度转换为度数"""
        self.current_joints = real_positions
        for i, pos in enumerate(real_positions):
            if i < len(self.joint_fb_labels):
                # --- 修改：弧度转度数 ---
                deg = math.degrees(pos)
                self.joint_fb_labels[i].setText(f"Fb: {deg:.2f}°")

    @pyqtSlot(list)
    def update_cartesian_feedback_ui(self, cart_data):
        """实时刷新末端绝对位姿显示"""
        x, y, z, roll, pitch, yaw = cart_data
        self.cart_fb_labels['X (m)'].setText(f"X (m):\n{x:.3f}")
        self.cart_fb_labels['Y (m)'].setText(f"Y (m):\n{y:.3f}")
        self.cart_fb_labels['Z (m)'].setText(f"Z (m):\n{z:.3f}")
        self.cart_fb_labels['Roll (°)'].setText(f"Roll (°):\n{roll:.2f}")
        self.cart_fb_labels['Pitch (°)'].setText(f"Pitch (°):\n{pitch:.2f}")
        self.cart_fb_labels['Yaw (°)'].setText(f"Yaw (°):\n{yaw:.2f}")

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros_thread.wait()
        print("[INFO] UI后台已关闭！所有资源已释放") 
        super().closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())