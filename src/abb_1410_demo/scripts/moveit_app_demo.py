#!/usr/bin/env python
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
import copy
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('Manipulator')
        # 初始化场景对象
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)  # 等待场景接口建立
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

        # 创建一个发布器，用于显示轨迹标记
        self.marker_publisher = rospy.Publisher(
            '/visualization_marker',
            Marker,
            queue_size=100)
            
        # 创建一个计时器，用于持续发布轨迹标记
        self.marker_id = 0
        self.trajectory_points = []
        rospy.Timer(rospy.Duration(0.1), self.publish_trajectory_marker)
        
        # 圆柱体参数
        self.cylinder_count = 5  # 圆柱体数量
        self.cylinder_spacing = 0.15  # 圆柱体之间的间隔距离
        self.cylinder_radius = 0.06  # 圆柱体半径
        self.cylinder_height = 0.5  # 圆柱体高度
        self.cylinder_base_x = 0.986 + 0.067  # 圆柱体X坐标
        self.cylinder_base_z = 1.5  # 圆柱体Z坐标
        self.current_y_offset = 0.0  # 当前Y轴偏移量
        self.cylinder_names = []  # 存储圆柱体名称
        
        self.clear_scene()
        
    def clear_scene(self):
        """清除场景中的所有物体"""
        for obj_name in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj_name)
        self.cylinder_names = []
        rospy.loginfo("已清除场景中的所有物体")
    
    def add_cylinder(self, name, position, height, radius, frame_id=None):
        """添加圆柱体障碍物"""
        if frame_id is None:
            frame_id = self.reference_frame
            
        # 创建碰撞对象
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = name
        
        # 定义圆柱体
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]  # [height, radius]
        
        # 设置圆柱体位置
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0
        
        # 将圆柱体添加到碰撞对象
        collision_object.primitives = [cylinder]
        collision_object.primitive_poses = [pose]
        collision_object.operation = CollisionObject.ADD
        
        # 将碰撞对象添加到场景
        self.scene.add_object(collision_object)
        self.cylinder_names.append(name)
        rospy.loginfo(f"已添加障碍物: {name}")
        rospy.sleep(0.5)  # 等待场景更新

    def create_obstacle_environment(self):
        """创建带有多个圆柱体障碍物的环境"""
        # 清除现有障碍物
        self.clear_scene()
        
        # 创建多个圆柱体，沿Y轴排列
        for i in range(self.cylinder_count):
            y_pos = -((self.cylinder_count - 1) * self.cylinder_spacing) / 2 + i * self.cylinder_spacing + self.current_y_offset
            self.add_cylinder(f"cylinder_{i}", 
                            [self.cylinder_base_x, y_pos, self.cylinder_base_z], 
                            self.cylinder_height, self.cylinder_radius)
        
        rospy.loginfo("已创建圆柱体障碍物环境")
        rospy.sleep(1)  # 等待场景完全更新
        
    def move_cylinders(self):
        """移动所有圆柱体沿Y轴方向"""
        self.current_y_offset += self.cylinder_spacing
        self.create_obstacle_environment()
        rospy.loginfo(f"圆柱体已沿Y轴移动 {self.cylinder_spacing} 米")

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
        
        rospy.loginfo("轨迹和立方体已清除")
        
    def poses(self):
        # 运动到第一个目标位置
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose1 = PoseStamped()
        target_pose1.header.frame_id = self.reference_frame
        target_pose1.header.stamp = rospy.Time.now()     
        target_pose1.pose.position.x = 0.986
        target_pose1.pose.position.y = 0
        target_pose1.pose.position.z = 1.73981
        target_pose1.pose.orientation.x = 0.47974
        target_pose1.pose.orientation.y = 0.519732
        target_pose1.pose.orientation.z = 0.515311
        target_pose1.pose.orientation.w = 0.483924

        target_pose2 = PoseStamped()
        target_pose2.header.frame_id = self.reference_frame
        target_pose2.header.stamp = rospy.Time.now()     
        target_pose2.pose.position.x = 0.986
        target_pose2.pose.position.y = 0
        target_pose2.pose.position.z = 1.335
        target_pose2.pose.orientation.x = 0.479588
        target_pose2.pose.orientation.y = 0.519915
        target_pose2.pose.orientation.z = 0.515073
        target_pose2.pose.orientation.w = 0.484131
        return target_pose1, target_pose2
    
    def move_to_target_pose_MoveL(self):
        # 清除之前的轨迹
        self.clear_trajectory()
        # 创建障碍物环境
        self.create_obstacle_environment()
        target_pose1, target_pose2 = self.poses()
        end_effector_link = self.end_effector_link
        arm = self.arm

        # 创建一个监听器，用于实时记录轨迹点
        trajectory_recorder = rospy.Timer(rospy.Duration(0.05), 
                                     lambda e: self.record_trajectory_point())
        
        # 计算需要移动的次数
        move_count = 3
        
        for move_step in range(move_count):
            rospy.loginfo(f"开始第 {move_step + 1}/{move_count} 次移动")
            
            for target_pose in [target_pose1, target_pose2]:
                # 设置机器臂当前的状态作为运动初始状态
                arm.set_start_state_to_current_state()
                # 设置机械臂终端运动的目标位姿
                arm.set_pose_target(target_pose, end_effector_link)
                # 创建笛卡尔路径点列表
                waypoints = []
                current_pose = arm.get_current_pose(end_effector_link)
                waypoints.append(current_pose.pose)  # 当前位置作为起点
                waypoints.append(target_pose.pose)   # 目标位置作为终点

                # 笛卡尔路径规划
                (plan, fraction) = arm.compute_cartesian_path(
                    waypoints,
                    0.01,
                    avoid_collisions=True
                )

                rospy.loginfo(f"规划完成，{fraction * 100:.2f}% 路径被覆盖")
                if fraction == 1.0 and plan and plan.joint_trajectory.points:
                    rospy.loginfo("笛卡尔路径规划成功")
                    arm.execute(plan)
                else:
                    rospy.logerr("笛卡尔路径规划失败")
            
            # 每次来回运动后移动圆柱体
            if move_step < move_count - 1:  # 最后一次不需要移动
                self.move_cylinders()
                rospy.sleep(1)  # 等待圆柱体移动完成
            
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
    test.move_to_target_pose_MoveL()