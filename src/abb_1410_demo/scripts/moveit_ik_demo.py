#!/usr/bin/env python
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Point
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import copy
class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
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
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.01)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)

        # 获取并打印关节名称，以便于输出关节角度
        self.joint_names = self.arm.get_active_joints()
        rospy.loginfo("机械臂关节名称: %s", self.joint_names)

        # 创建一个发布器，用于显示轨迹标记
        self.marker_publisher = rospy.Publisher(
            '/visualization_marker',
            Marker,
            queue_size=100)
            
        # 创建一个计时器，用于持续发布轨迹标记
        self.marker_id = 0
        self.trajectory_points = []
        rospy.Timer(rospy.Duration(0.2), self.publish_trajectory_marker)
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
        # 确保标记的姿态被正确初始化
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
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
    def print_joint_angles(self):
        # 获取当前关节角度（单位：弧度）
        current_joints = self.arm.get_current_joint_values()
        
        # 打印关节角度（同时转换为角度单位便于阅读）
        rospy.loginfo("当前关节角度（弧度）: %s", current_joints)
        rospy.loginfo("当前关节角度（角度）: %s", np.degrees(current_joints).tolist())
        
        # 为了更清晰的输出，打印每个关节的名称和对应角度
        for i, name in enumerate(self.joint_names):
            rospy.loginfo("关节 %s: %.2f 弧度 (%.2f 度)", 
                         name, current_joints[i], np.degrees(current_joints[i]))
    def poses(self):
        # 运动到第一个目标位置
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose1 = PoseStamped()
        target_pose1.header.frame_id = self.reference_frame
        target_pose1.header.stamp = rospy.Time.now()     
        target_pose1.pose.position.x = 0.986
        target_pose1.pose.position.y = -0.349216
        target_pose1.pose.position.z = 1.335
        target_pose1.pose.orientation.x = 0.47962
        target_pose1.pose.orientation.y = 0.519863
        target_pose1.pose.orientation.z = 0.515131
        target_pose1.pose.orientation.w = 0.484094

        # 运动到第2个目标位置
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose2 = PoseStamped()
        target_pose2.header.frame_id = self.reference_frame
        target_pose2.header.stamp = rospy.Time.now()     
        target_pose2.pose.position.x = 1.24799
        target_pose2.pose.position.y = -0.349216
        target_pose2.pose.position.z = 1.335
        target_pose2.pose.orientation.x = 0.479586
        target_pose2.pose.orientation.y = 0.519852
        target_pose2.pose.orientation.z = 0.515053
        target_pose2.pose.orientation.w = 0.484223

        # 运动到第3个目标位置
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose3 = PoseStamped()
        target_pose3.header.frame_id = self.reference_frame
        target_pose3.header.stamp = rospy.Time.now()     
        target_pose3.pose.position.x = 1.24799
        target_pose3.pose.position.y = -0.349216
        target_pose3.pose.position.z = 1.0235
        target_pose3.pose.orientation.x = 0.479741
        target_pose3.pose.orientation.y = 0.51988
        target_pose3.pose.orientation.z = 0.514929
        target_pose3.pose.orientation.w = 0.484171


        ## 运动到第4个目标位置
        target_pose4 = PoseStamped()
        target_pose4.header.frame_id = self.reference_frame
        target_pose4.header.stamp = rospy.Time.now()     
        target_pose4.pose.position.x = 0.986
        target_pose4.pose.position.y = -0.349216
        target_pose4.pose.position.z = 1.0235
        target_pose4.pose.orientation.x = 0.479642
        target_pose4.pose.orientation.y = 0.519846
        target_pose4.pose.orientation.z = 0.515091
        target_pose4.pose.orientation.w = 0.484133

        ## 运动到第5个目标位置
        target_pose5 = PoseStamped()
        target_pose5.header.frame_id = self.reference_frame
        target_pose5.header.stamp = rospy.Time.now()     
        target_pose5.pose.position.x = 0.986
        target_pose5.pose.position.y = 0.418872
        target_pose5.pose.position.z = 1.0235
        target_pose5.pose.orientation.x = 0.479648
        target_pose5.pose.orientation.y = 0.519867
        target_pose5.pose.orientation.z = 0.515168
        target_pose5.pose.orientation.w = 0.484023

        ## 运动到第6个目标位置
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose6 = PoseStamped()
        target_pose6.header.frame_id = self.reference_frame
        target_pose6.header.stamp = rospy.Time.now()     
        target_pose6.pose.position.x = 0.986
        target_pose6.pose.position.y = 0.418872
        target_pose6.pose.position.z = 1.335
        target_pose6.pose.orientation.x = 0.479557
        target_pose6.pose.orientation.y = 0.519961
        target_pose6.pose.orientation.z = 0.5152
        target_pose6.pose.orientation.w = 0.483978

        ## 运动到第7个目标位置
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose7 = PoseStamped()
        target_pose7.header.frame_id = self.reference_frame
        target_pose7.header.stamp = rospy.Time.now()     
        target_pose7.pose.position.x = 0.986
        target_pose7.pose.position.y = 0
        target_pose7.pose.position.z = 1.335
        target_pose7.pose.orientation.x = 0.479588
        target_pose7.pose.orientation.y = 0.519915
        target_pose7.pose.orientation.z = 0.515073
        target_pose7.pose.orientation.w = 0.484131



        return target_pose1, target_pose2, target_pose3, target_pose4, target_pose5, target_pose6, target_pose7

    def move_to_target_pose(self):
        # 清除之前的轨迹
        self.clear_trajectory()

        target_pose1, target_pose2, target_pose3, target_pose4, target_pose5, target_pose6, target_pose7 = self.poses()
        end_effector_link = self.end_effector_link
        arm = self.arm
        # 创建一个监听器，用于实时记录轨迹点
        trajectory_recorder = rospy.Timer(rospy.Duration(0.05), 
                                     lambda e: self.record_trajectory_point())
        for target_pose in [target_pose1, target_pose2, target_pose3, target_pose4, target_pose5, target_pose6, target_pose7]:
            # 设置机器臂当前的状态作为运动初始状态
            arm.set_start_state_to_current_state()
            # 设置机械臂终端运动的目标位姿
            arm.set_pose_target(target_pose, end_effector_link)
            # 规划运动路径
            plan = arm.plan()
            # 检查规划是否成功
            if plan and isinstance(plan, tuple) and plan[0]:
                traj = plan[1]  # 提取轨迹部分
                rospy.loginfo("规划成功")
                self.print_joint_angles()
            else:
                rospy.logerr("规划失败")
                return
            # 按照规划的运动路径控制机械臂运动
            arm.execute(traj)
            rospy.sleep(1)
        # 停止记录
        trajectory_recorder.shutdown()
        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

        
        
if __name__ == "__main__":
    test = MoveItIkDemo()
    test.move_to_target_pose()


