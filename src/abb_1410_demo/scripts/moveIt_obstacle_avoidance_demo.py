#!/usr/bin/env python
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from moveit_msgs.msg import DisplayTrajectory
from visualization_msgs.msg import Marker, MarkerArray
import copy
class MoveItObstacleAvoidanceDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_obstacle_avoidance_demo')
                
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
        rospy.Timer(rospy.Duration(0.1), self.publish_trajectory_marker)


        # 创建场景对象
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)  # 等待场景接口初始化
        
        # 添加障碍物
        self.add_obstacles()
        rospy.sleep(2)  # 等待障碍物添加完成
        
        # 开始避障演示
        self.obstacle_avoidance_demo()
        
    def print_joint_angles(self):
        """打印当前关节角度"""
        # 获取当前关节角度（单位：弧度）
        current_joints = self.arm.get_current_joint_values()
        
        # 打印关节角度（同时转换为角度单位便于阅读）
        rospy.loginfo("当前关节角度（弧度）: %s", current_joints)
        rospy.loginfo("当前关节角度（角度）: %s", np.degrees(current_joints).tolist())
        
        # 为了更清晰的输出，打印每个关节的名称和对应角度
        for i, name in enumerate(self.joint_names):
            rospy.loginfo("关节 %s: %.2f 弧度 (%.2f 度)", 
                         name, current_joints[i], np.degrees(current_joints[i]))
    
    def add_obstacles(self):
        """向场景中添加障碍物"""
        # 清除之前的所有物体
        self.scene.remove_world_object()
        
        # 添加一个盒子作为障碍物
        box_pose = PoseStamped()
        box_pose.header.frame_id = self.reference_frame
        box_pose.pose.position.x = 0.8
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 1.0
        box_pose.pose.orientation.w = 1.0
        
        # 添加盒子
        self.scene.add_box("obstacle_box", box_pose, size=(0.2, 0.2, 0.2))
        rospy.loginfo("已添加盒子障碍物")
        
        # 添加一个圆柱体作为障碍物
        cylinder_pose = PoseStamped()
        cylinder_pose.header.frame_id = self.reference_frame
        cylinder_pose.pose.position.x = 0.7
        cylinder_pose.pose.position.y = 0.4
        cylinder_pose.pose.position.z = 1.0
        cylinder_pose.pose.orientation.w = 1.0
        
        # 添加圆柱体（半径0.1，高度0.3）
        self.scene.add_cylinder("obstacle_cylinder", cylinder_pose, height=0.3, radius=0.1)
        rospy.loginfo("已添加圆柱体障碍物")
        
        # 添加一个平面作为桌面
        plane_pose = PoseStamped()
        plane_pose.header.frame_id = self.reference_frame
        plane_pose.pose.position.x = 0
        plane_pose.pose.position.y = 0
        plane_pose.pose.position.z = 0
        plane_pose.pose.orientation.w = 1.0
        
        # 添加平面
        self.scene.add_plane("table", plane_pose)
        rospy.loginfo("已添加平面作为桌面")
    
    def clear_obstacles(self):
        """清除场景中的障碍物"""
        self.scene.remove_world_object("obstacle_box")
        self.scene.remove_world_object("obstacle_cylinder")
        self.scene.remove_world_object("table")
        rospy.loginfo("已清除所有障碍物")
    
    
    def poses(self):
        """定义目标位姿列表"""
        # 定义一系列目标位姿，这些位姿会让机械臂在障碍物周围移动
        
        # 运动到第一个目标位置
        target_pose1 = PoseStamped()
        target_pose1.header.frame_id = self.reference_frame
        target_pose1.header.stamp = rospy.Time.now()     
        target_pose1.pose.position.x = 0.6
        target_pose1.pose.position.y = 0.2
        target_pose1.pose.position.z = 1.0
        target_pose1.pose.orientation.x = 0.48
        target_pose1.pose.orientation.y = 0.52
        target_pose1.pose.orientation.z = 0.52
        target_pose1.pose.orientation.w = 0.48

        # 运动到第二个目标位置
        target_pose2 = PoseStamped()
        target_pose2.header.frame_id = self.reference_frame
        target_pose2.header.stamp = rospy.Time.now()     
        target_pose2.pose.position.x = 0.7
        target_pose2.pose.position.y = -0.2
        target_pose2.pose.position.z = 1.3
        target_pose2.pose.orientation.x = 0.48
        target_pose2.pose.orientation.y = 0.52
        target_pose2.pose.orientation.z = 0.52
        target_pose2.pose.orientation.w = 0.48

        # 运动到第三个目标位置 - 需要绕开盒子障碍物
        target_pose3 = PoseStamped()
        target_pose3.header.frame_id = self.reference_frame
        target_pose3.header.stamp = rospy.Time.now()     
        target_pose3.pose.position.x = 0.9
        target_pose3.pose.position.y = 0.2
        target_pose3.pose.position.z = 1.2
        target_pose3.pose.orientation.x = 0.48
        target_pose3.pose.orientation.y = 0.52
        target_pose3.pose.orientation.z = 0.52
        target_pose3.pose.orientation.w = 0.48

        # 运动到第四个目标位置 - 需要绕开圆柱体障碍物
        target_pose4 = PoseStamped()
        target_pose4.header.frame_id = self.reference_frame
        target_pose4.header.stamp = rospy.Time.now()     
        target_pose4.pose.position.x = 0.6
        target_pose4.pose.position.y = 0.4
        target_pose4.pose.position.z = 0.8
        target_pose4.pose.orientation.x = 0.48
        target_pose4.pose.orientation.y = 0.52
        target_pose4.pose.orientation.z = 0.52
        target_pose4.pose.orientation.w = 0.48

        # 运动到第五个目标位置
        target_pose5 = PoseStamped()
        target_pose5.header.frame_id = self.reference_frame
        target_pose5.header.stamp = rospy.Time.now()     
        target_pose5.pose.position.x = 0.5
        target_pose5.pose.position.y = -0.4
        target_pose5.pose.position.z = 1.1
        target_pose5.pose.orientation.x = 0.48
        target_pose5.pose.orientation.y = 0.52
        target_pose5.pose.orientation.z = 0.52
        target_pose5.pose.orientation.w = 0.48

        return [target_pose1, target_pose2, target_pose3, target_pose4, target_pose5]
    
    def obstacle_avoidance_demo(self):
        """执行避障演示"""
        arm = self.arm
        target_poses = self.poses()
        
        # 设置更长的规划时间以处理复杂的避障场景
        arm.set_planning_time(10.0)
        
        # 尝试规划并执行每个目标位姿
        for i, target_pose in enumerate(target_poses):
            rospy.loginfo(f"\n\n尝试规划到目标位姿 {i+1}/{len(target_poses)}")
            
            # 设置机器臂当前的状态作为运动初始状态
            arm.set_start_state_to_current_state()
            
            # 设置机械臂终端运动的目标位姿
            arm.set_pose_target(target_pose, self.end_effector_link)
            
            # 规划运动路径
            rospy.loginfo(f"规划到目标位置: x={target_pose.pose.position.x}, y={target_pose.pose.position.y}, z={target_pose.pose.position.z}")
            plan = arm.plan()
            
            # 检查规划是否成功
            if plan and isinstance(plan, tuple) and plan[0]:
                traj = plan[1]  # 提取轨迹部分
                rospy.loginfo("规划成功，发送轨迹到RViz并执行...")
                
                
                # 执行规划的轨迹
                arm.execute(traj)
                rospy.sleep(1)
                
                # 打印当前关节角度
                self.print_joint_angles()
            else:
                rospy.logwarn(f"到目标位姿 {i+1} 的规划失败，跳过该点")
                continue
        
        # 清除障碍物
        self.clear_obstacles()
        rospy.sleep(1)
        
        # 控制机械臂回到初始化位置
        rospy.loginfo("演示完成，返回初始位置")
        arm.set_named_target('home')
        arm.go()
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("演示完成")

if __name__ == "__main__":
    try:
        MoveItObstacleAvoidanceDemo()
    except rospy.ROSInterruptException:
        pass