import sys
import math
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout, 
                             QWidget, QHBoxLayout, QGroupBox, QTabWidget,
                             QDoubleSpinBox, QPushButton, QGridLayout, QMessageBox)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot

import matplotlib
matplotlib.use('Qt5Agg')

matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=12, height=7, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.fig.subplots_adjust(bottom=0.1, left=0.06, right=0.96, top=0.92, wspace=0.2, hspace=0.35)
        self.ax3d = self.fig.add_subplot(2, 2, 1, projection='3d')
        self.ax_xyz = self.fig.add_subplot(2, 2, 2)
        self.ax_err = self.fig.add_subplot(2, 1, 2)
        super(MplCanvas, self).__init__(self.fig)
        self.setParent(parent)

#ROS2后台线程
class ROS2Thread(QThread):
    analysis_ready_signal = pyqtSignal(list, list, list, list)
    update_cartesian_signal = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.node = None
        
        #采集状态控制
        self.is_recording = False
        self.actual_path = []
        self.time_stamps = []
        self.start_pose = None
        self.target_pose = None

    def run(self):
        rclpy.init()
        self.node = rclpy.create_node('qt_ros2_control_node')
        self.pose_pub = self.node.create_publisher(PoseStamped, '/left_arm_node/teleop_pose', 10)
            
        #TF2监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        #定时器：以30Hz频率刷新状态并采集数据
        self.timer = self.node.create_timer(1.0 / 30.0, self.tf_callback) 
        
        self.node.get_logger().info("UI后台已启动，TF2监听中")

        try:
            rclpy.spin(self.node)
        except Exception:
            pass
        finally:
            if self.node:
                self.node.destroy_node()

    def tf_callback(self):
        try:
            #获取末端相对于基座的绝对坐标
            trans = self.tf_buffer.lookup_transform('base_link', 'L_wrist_3', rclpy.time.Time())
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            
            #更新实时UI显示
            self.update_cartesian_signal.emit([x, y, z])
            
            #如果正在采集，记录真实坐标
            if self.is_recording:
                current_time = self.node.get_clock().now().nanoseconds / 1e9
                if len(self.time_stamps) == 0:
                    self.start_time = current_time
                self.time_stamps.append(current_time - self.start_time)
                self.actual_path.append([x, y, z])
                
        except Exception as e:
            pass

    def start_execution_and_recording(self, dx, dy, dz):
        """发送指令，并同步开启真实数据记录"""
        if not self.node: return
        
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'L_wrist_3', rclpy.time.Time())
            self.start_pose = [trans.transform.translation.x, 
                               trans.transform.translation.y, 
                               trans.transform.translation.z]
                               
            self.target_pose = [self.start_pose[0] + dx, 
                                self.start_pose[1] + dy, 
                                self.start_pose[2] + dz]
                                
            self.actual_path.clear()
            self.time_stamps.clear()
            self.is_recording = True
            self.node.get_logger().info("开始下发指令，TF2记录已启动！")
            
            msg = PoseStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.pose.position.x = float(dx)
            msg.pose.position.y = float(dy)
            msg.pose.position.z = float(dz)
            
            msg.pose.orientation.w = 1.0 
            self.pose_pub.publish(msg)
            
        except Exception as e:
            print("无法获取起点位姿，请检查TF树！", e)

    def stop_recording(self):
        """停止采集并发送数据给 UI 绘图"""
        if self.is_recording:
            self.is_recording = False
            self.node.get_logger().info("记录已停止，生成图表...")
            #把采集到的数据发给主线程绘图
            self.analysis_ready_signal.emit(self.time_stamps, self.actual_path, self.start_pose, self.target_pose)

    def stop(self):
        if rclpy.ok():
            rclpy.shutdown() 
        self.quit()

#PyQt主窗口类
class TomatoArmAnalysisSystem(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("机械臂笛卡尔空间直线插补与轨迹误差分析系统V1.0")
        self.resize(1200, 850)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        top_layout = QHBoxLayout()
        
        #实时坐标反馈区
        fb_group = QGroupBox("实时末端绝对坐标 (TF2)")
        fb_layout = QHBoxLayout()
        self.lbl_x = QLabel("X: 0.000 m")
        self.lbl_y = QLabel("Y: 0.000 m")
        self.lbl_z = QLabel("Z: 0.000 m")
        for lbl in [self.lbl_x, self.lbl_y, self.lbl_z]:
            lbl.setStyleSheet("color: #0078D7; font-weight: bold; font-size: 16px;")
            fb_layout.addWidget(lbl)
        fb_group.setLayout(fb_layout)
        top_layout.addWidget(fb_group, 1)
        
        #直线插补指令区
        cmd_group = QGroupBox("笛卡尔空间直线插补指令区")
        cmd_layout = QGridLayout()
        
        cmd_layout.addWidget(QLabel("目标位移增量 (m):"), 0, 0)
        self.box_dx = QDoubleSpinBox()
        self.box_dx.setRange(-1.0, 1.0)
        self.box_dx.setSingleStep(0.01)
        self.box_dx.setValue(0.05) # 默认 5cm
        cmd_layout.addWidget(QLabel("dX:"), 0, 1)
        cmd_layout.addWidget(self.box_dx, 0, 2)
        
        self.box_dy = QDoubleSpinBox()
        self.box_dy.setRange(-1.0, 1.0)
        self.box_dy.setSingleStep(0.01)
        cmd_layout.addWidget(QLabel("dY:"), 0, 3)
        cmd_layout.addWidget(self.box_dy, 0, 4)
        
        self.box_dz = QDoubleSpinBox()
        self.box_dz.setRange(-1.0, 1.0)
        self.box_dz.setSingleStep(0.01)
        cmd_layout.addWidget(QLabel("dZ:"), 0, 5)
        cmd_layout.addWidget(self.box_dz, 0, 6)
        
        cmd_group.setLayout(cmd_layout)
        top_layout.addWidget(cmd_group, 2)
        
        #采集控制按钮区
        btn_layout = QVBoxLayout()
        self.btn_execute = QPushButton("1. 规划执行直线轨迹并录制")
        self.btn_execute.setStyleSheet("background-color: #28A745; color: white; font-weight: bold; height: 35px;")
        self.btn_execute.clicked.connect(self.on_execute_clicked)
        
        self.btn_stop = QPushButton("2. 运动完毕：停止录制并生成图表")
        self.btn_stop.setStyleSheet("background-color: #D83B01; color: white; font-weight: bold; height: 35px;")
        self.btn_stop.clicked.connect(self.on_stop_clicked)
        
        btn_layout.addWidget(self.btn_execute)
        btn_layout.addWidget(self.btn_stop)
        top_layout.addLayout(btn_layout, 1)
        
        main_layout.addLayout(top_layout)

        # 底部指标与绘图区
        self.lbl_metrics = QLabel("等待执行采集...")
        self.lbl_metrics.setStyleSheet("font-size: 14px; font-weight: bold; color: #333333;")
        self.lbl_metrics.setAlignment(Qt.AlignCenter)
        
        self.lbl_metrics.setMinimumHeight(40)
        self.lbl_metrics.setMaximumHeight(40)
        main_layout.addWidget(self.lbl_metrics)

        self.canvas = MplCanvas(self, width=12, height=7, dpi=100)
        main_layout.addWidget(self.canvas, stretch=1)

        # 启动后台
        self.ros_thread = ROS2Thread()
        self.ros_thread.update_cartesian_signal.connect(self.update_fb_ui)
        self.ros_thread.analysis_ready_signal.connect(self.plot_and_save_data)
        self.ros_thread.start()

    @pyqtSlot(list)
    def update_fb_ui(self, coords):
        self.lbl_x.setText(f"X: {coords[0]:.4f} m")
        self.lbl_y.setText(f"Y: {coords[1]:.4f} m")
        self.lbl_z.setText(f"Z: {coords[2]:.4f} m")

    def on_execute_clicked(self):
        dx = self.box_dx.value()
        dy = self.box_dy.value()
        dz = self.box_dz.value()
        self.ros_thread.start_execution_and_recording(dx, dy, dz)
        self.lbl_metrics.setText("正在执行插补并采集TF2数据，请等待机械臂停稳后点击“停止”")

    def on_stop_clicked(self):
        self.ros_thread.stop_recording()

    @pyqtSlot(list, list, list, list)
    def plot_and_save_data(self, times, actual_path, start_pose, target_pose):
        if len(actual_path) < 2:
            self.lbl_metrics.setText("错误：采集到的数据点太少！")
            return
            
        actual_np = np.array(actual_path)
        p_start = np.array(start_pose)
        p_end = np.array(target_pose)
        
        line_vec = p_end - p_start
        line_length = np.linalg.norm(line_vec)
        
        # 计算综合误差与 XYZ 独立误差
        total_errors_mm = []
        err_x, err_y, err_z = [], [], []
        
        for p in actual_np:
            #综合空间误差(法向距离)
            cross_prod = np.cross(p - p_start, p - p_end)
            dist = np.linalg.norm(cross_prod) / line_length if line_length > 1e-6 else 0
            total_errors_mm.append(dist * 1000.0)
            
            #XYZ独立偏差(点到直线的投影偏差)
            t_proj = np.dot(p - p_start, line_vec) / (line_length**2) if line_length > 1e-6 else 0
            t_proj = max(0, min(1, t_proj)) 
            ideal_p = p_start + t_proj * line_vec
            
            diff = (p - ideal_p) * 1000.0 
            err_x.append(diff[0])
            err_y.append(diff[1])
            err_z.append(diff[2])
            
        total_errors_mm = np.array(total_errors_mm)
        max_err = np.max(total_errors_mm)
        mean_err = np.mean(total_errors_mm)
        
        self.lbl_metrics.setText(f"轨迹分析完成！采集点数: {len(actual_np)} | 综合最大空间偏差: {max_err:.3f} mm | 平均偏差: {mean_err:.3f} mm")

        try:
            with open('trajectory_error_analysis.csv', 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Time_s', 'Actual_X', 'Actual_Y', 'Actual_Z', 'Err_X_mm', 'Err_Y_mm', 'Err_Z_mm', 'Total_Error_mm'])
                for i in range(len(times)):
                    writer.writerow([times[i], actual_np[i,0], actual_np[i,1], actual_np[i,2], 
                                     err_x[i], err_y[i], err_z[i], total_errors_mm[i]])
        except Exception as e:
            print("CSV轨迹文件写入失败", e)

        # 渲染三维与二维图表
        self.canvas.ax3d.clear()
        self.canvas.ax_xyz.clear()
        self.canvas.ax_err.clear()
        
        #绘制 3D 轨迹
        self.canvas.ax3d.plot([p_start[0], p_end[0]], [p_start[1], p_end[1]], [p_start[2], p_end[2]], 
                              'b--', linewidth=2, label='理想目标线')
        self.canvas.ax3d.plot(actual_np[:,0], actual_np[:,1], actual_np[:,2], 
                              'r-', linewidth=2, label='实际 TF2 轨迹')
        self.canvas.ax3d.set_title("3D 笛卡尔空间轨迹跟踪")
        self.canvas.ax3d.set_xlabel("X (m)")
        self.canvas.ax3d.set_ylabel("Y (m)")
        self.canvas.ax3d.set_zlabel("Z (m)")
        self.canvas.ax3d.legend(fontsize=9)
        
        #绘制 XYZ 三轴独立偏差
        self.canvas.ax_xyz.plot(times, err_x, 'r-', linewidth=1.5, label='X轴 偏差')
        self.canvas.ax_xyz.plot(times, err_y, 'g-', linewidth=1.5, label='Y轴 偏差')
        self.canvas.ax_xyz.plot(times, err_z, 'b-', linewidth=1.5, label='Z轴 偏差')
        self.canvas.ax_xyz.set_title("笛卡尔空间各轴独立跟随偏差")
        self.canvas.ax_xyz.set_ylabel("偏差 (mm)")
        self.canvas.ax_xyz.grid(True, linestyle=':')
        self.canvas.ax_xyz.legend(loc='upper right', fontsize=9)
        
        #绘制综合空间误差
        self.canvas.ax_err.plot(times, total_errors_mm, 'k-', linewidth=2, label='Total Spatial Error (综合空间误差)')
        self.canvas.ax_err.fill_between(times, total_errors_mm, alpha=0.2, color='gray')
        
        max_idx = np.argmax(total_errors_mm)
        self.canvas.ax_err.plot(times[max_idx], max_err, 'ro')
        self.canvas.ax_err.set_ylim(0, max_err * 1.3)
        
        self.canvas.ax_err.annotate(f'最大误差峰值\n{max_err:.2f} mm', 
                                    xy=(times[max_idx], max_err), 
                                    xytext=(times[max_idx], max_err * 1.1),
                                    arrowprops=dict(facecolor='red', shrink=0.05, width=1),
                                    fontsize=10, color='red', weight='bold', ha='center')
                                    
        self.canvas.ax_err.set_title("轨迹综合法向误差分析 (Total Spatial Error)")
        self.canvas.ax_err.set_xlabel("执行时间 (s)")
        self.canvas.ax_err.set_ylabel("空间绝对偏差距离 (mm)")
        self.canvas.ax_err.grid(True, linestyle='--')
        self.canvas.ax_err.legend(loc='upper right')
        self.canvas.draw()

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros_thread.wait(1000) 
        print("后台线程已安全关闭，程序退出。")
        super().closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = TomatoArmAnalysisSystem()
    window.show()
    sys.exit(app.exec_())