#!/usr/bin/env python
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Point
from moveit_msgs.msg import RobotTrajectory
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
import copy

class MoveItTrajDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_traj_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('Manipulator')
                
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.3)
        self.arm.set_max_velocity_scaling_factor(0.3)
        
        # 创建一个发布器，用于显示轨迹标记
        self.marker_publisher = rospy.Publisher(
            '/visualization_marker',
            Marker,
            queue_size=100)
            
        # 创建一个计时器，用于持续发布轨迹标记
        self.marker_id = 0
        self.trajectory_points = []
        rospy.Timer(rospy.Duration(0.2), self.publish_trajectory_marker)

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)

        # 获取初始姿态作为默认四元数
        self.current_pose = self.arm.get_current_pose(self.end_effector_link).pose
        self.default_orientation = self.current_pose.orientation
        
        rospy.loginfo("机械臂初始化完成")

    def print_pose(self, pose):
        """打印位姿信息"""
        position = pose.position
        orientation = pose.orientation
        rospy.loginfo("位置: x=%.3f, y=%.3f, z=%.3f", position.x, position.y, position.z)
        rospy.loginfo("姿态: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
                     orientation.x, orientation.y, orientation.z, orientation.w)

    def create_figure8_trajectory(self, center_x, center_y, center_z, width, height, points=50):
        """生成8字形轨迹点列表"""
        waypoints = []
        t = np.linspace(0, 2*np.pi, points)
        
        for i in range(points):
            pose = Pose()
            # 使用参数方程生成8字形轨迹
            pose.position.x = center_x + width * np.sin(t[i])
            pose.position.y = center_y + height * np.sin(t[i]) * np.cos(t[i])
            pose.position.z = center_z
            
            # 使用默认姿态
            pose.orientation = self.default_orientation
            waypoints.append(pose)
        
        return waypoints

    def create_ellipse_trajectory(self, center_x, center_y, center_z, a, b, points=50):
        """生成椭圆轨迹点列表"""
        waypoints = []
        t = np.linspace(0, 2*np.pi, points)
        
        for i in range(points):
            pose = Pose()
            # 使用参数方程生成椭圆轨迹
            pose.position.x = center_x + a * np.cos(t[i])
            pose.position.y = center_y + b * np.sin(t[i])
            pose.position.z = center_z
            
            # 使用默认姿态
            pose.orientation = self.default_orientation
            waypoints.append(pose)
        
        return waypoints

    def create_spiral_trajectory(self, center_x, center_y, start_z, radius, height, turns=3, points=100):
        """生成螺旋轨迹点列表"""
        waypoints = []
        t = np.linspace(0, turns*2*np.pi, points)
        
        for i in range(points):
            pose = Pose()
            # 使用参数方程生成螺旋轨迹
            pose.position.x = center_x + radius * np.cos(t[i])
            pose.position.y = center_y + radius * np.sin(t[i])
            pose.position.z = start_z + height * (t[i] / (turns*2*np.pi))
            
            # 使用默认姿态
            pose.orientation = self.default_orientation
            waypoints.append(pose)
        
        return waypoints

    def publish_trajectory_marker(self, event=None):
        """发布轨迹标记，显示机械臂末端执行器的运动轨迹"""
        if not self.trajectory_points:
            return
            
        # 创建一个线条标记，用于显示轨迹
        marker = Marker()
        marker.header.frame_id = self.reference_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "end_effector_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # 线宽
        
        # 设置颜色 (红色)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # 轨迹点
        for point in self.trajectory_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            marker.points.append(p)
            
        # 设置标记生存时间 (0表示永久)
        marker.lifetime = rospy.Duration(0)
        
        # 发布标记
        self.marker_publisher.publish(marker)

    def record_trajectory_point(self):
        """记录当前末端执行器位置用于轨迹显示"""
        current_pose = self.arm.get_current_pose(self.end_effector_link).pose
        point = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
        self.trajectory_points.append(point)
        
    def clear_trajectory(self):
        """清除轨迹记录"""
        self.trajectory_points = []
        # 发布一个空标记以清除显示
        marker = Marker()
        marker.header.frame_id = self.reference_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "end_effector_trajectory"
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_publisher.publish(marker)
        rospy.loginfo("轨迹已清除")

    def execute_cartesian_trajectory(self, waypoints):
        """执行笛卡尔轨迹"""
        # 清除之前的轨迹
        self.clear_trajectory()
        
        # 设置机械臂当前的状态为运动初始状态
        self.arm.set_start_state_to_current_state()
        
        # 规划笛卡尔路径
        (plan, fraction) = self.arm.compute_cartesian_path(
                                waypoints,   # 路径点
                                0.01,        # eef_step - 终端执行器的步长(米)
                                True)        # avoid_collisions - 避免碰撞检查
        
        if fraction < 0.5:
            rospy.logwarn("只能规划 %.2f%% 的轨迹路径", fraction * 100.0)
            return False
        
        rospy.loginfo("规划了 %.2f%% 的轨迹路径", fraction * 100.0)
        
        # 创建一个监听器，用于实时记录轨迹点
        trajectory_recorder = rospy.Timer(rospy.Duration(0.05), 
                                          lambda e: self.record_trajectory_point())
        
        # 执行轨迹
        self.arm.execute(plan, wait=True)
        
        # 停止记录
        trajectory_recorder.shutdown()
        
        return True

    def move_to_start_position(self):
        """移动到适合的起始位置"""
        start_pose = Pose()
        start_pose.position.x = 1.0
        start_pose.position.y = 0.0
        start_pose.position.z = 1.2
        start_pose.orientation = self.default_orientation
        
        # 设置机械臂当前的状态为运动初始状态
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(start_pose)
        self.arm.go()
        rospy.sleep(1)
        
        # 获取当前位置作为轨迹中心
        current_pose = self.arm.get_current_pose(self.end_effector_link).pose
        center_x = current_pose.position.x
        center_y = current_pose.position.y
        center_z = current_pose.position.z
        
        rospy.loginfo("轨迹中心点: x=%.3f, y=%.3f, z=%.3f", 
                     center_x, center_y, center_z)
        
        return center_x, center_y, center_z

    def execute_figure8(self, center_x, center_y, center_z):
        """执行8字形轨迹"""
        rospy.loginfo("开始执行8字形轨迹...")
        waypoints = self.create_figure8_trajectory(
            center_x, center_y, center_z, 
            width=0.2, height=0.1, points=50
        )
        success = self.execute_cartesian_trajectory(waypoints)
        if not success:
            rospy.logwarn("8字形轨迹执行失败")
        return success

    def execute_ellipse(self, center_x, center_y, center_z):
        """执行椭圆轨迹"""
        rospy.loginfo("开始执行椭圆轨迹...")
        waypoints = self.create_ellipse_trajectory(
            center_x, center_y, center_z, 
            a=0.2, b=0.15, points=50
        )
        success = self.execute_cartesian_trajectory(waypoints)
        if not success:
            rospy.logwarn("椭圆轨迹执行失败")
        return success

    def execute_spiral(self, center_x, center_y, center_z):
        """执行螺旋轨迹"""
        rospy.loginfo("开始执行螺旋轨迹...")
        waypoints = self.create_spiral_trajectory(
            center_x, center_y, center_z, 
            radius=0.1, height=0.2, turns=2, points=80
        )
        success = self.execute_cartesian_trajectory(waypoints)
        if not success:
            rospy.logwarn("螺旋轨迹执行失败")
        return success

    def print_menu(self):
        """打印操作菜单"""
        print("\n===== 机械臂轨迹演示菜单 =====")
        print("1. 执行8字形轨迹")
        print("2. 执行椭圆轨迹")
        print("3. 执行螺旋轨迹")
        print("4. 返回起始位置")
        print("5. 清除轨迹显示")
        print("0. 退出程序")
        print("================================")
        print("请输入选项 (0-5): ")

    def interactive_demo(self):
        """交互式轨迹演示"""
        try:
            # 移动到合适的起始位置
            center_x, center_y, center_z = self.move_to_start_position()
            
            while not rospy.is_shutdown():
                self.print_menu()
                
                # 获取用户输入
                try:
                    choice = int(input())
                except ValueError:
                    rospy.logwarn("无效输入，请输入0-5之间的数字")
                    continue
                
                if choice == 0:
                    rospy.loginfo("退出程序...")
                    break
                elif choice == 1:
                    self.execute_figure8(center_x, center_y, center_z)
                elif choice == 2:
                    self.execute_ellipse(center_x, center_y, center_z)
                elif choice == 3:
                    self.execute_spiral(center_x, center_y, center_z)
                elif choice == 4:
                    center_x, center_y, center_z = self.move_to_start_position()
                elif choice == 5:
                    self.clear_trajectory()
                else:
                    rospy.logwarn("无效选项，请输入0-5之间的数字")
            
            # 控制机械臂回到初始化位置
            self.arm.set_named_target('home')
            self.arm.go()
            
            rospy.loginfo("演示完成")
            
        except Exception as e:
            rospy.logerr("轨迹执行出错: %s", str(e))
        finally:
            # 关闭并退出moveit
            moveit_commander.roscpp_shutdown()
            moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        demo = MoveItTrajDemo()
        demo.interactive_demo()
    except rospy.ROSInterruptException:
        rospy.loginfo("轨迹演示被中断")