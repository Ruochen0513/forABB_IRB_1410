#!/usr/bin/env python
import rospy, sys
import moveit_commander
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point
import copy
class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)

        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('Manipulator')
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
                        
        # 设置机械臂运动的允许误差值
        self.arm.set_goal_joint_tolerance(0.001)

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


    def joints(self):
        joint_positions = []
        joint_position1 = [-0.34682357326061064, 0.10601685751130724, -0.1335686168324609, 1.4642779976772364, 0.33996894798537963, -1.4654412015853875]
        joint_position2 = [-0.27591327100069596, 0.5485614312670358, -0.6829236504200271, 1.1246680560371627, 0.3134741682728087, -1.1036019885441088]
        joint_position3 = [-0.27773192275732156, 0.5003083055555797, -0.23760886754439436, 2.300518777933009, 0.3707055748245943, -2.3348209029529112]
        joint_position4 = [-0.3535944612967467, 0.11586984331869694, 0.2405759522187829, 2.3074221213603354, 0.4941283929764087, -2.369925622442398]
        joint_position5 = [0.41767661101171544, 0.1561203275691909, 0.19823299591547575, -2.2314905478406533, 0.5428495909726367, -3.9773703808115233]
        joint_position6 = [0.4122898327521148, 0.149153717820839, -0.1803077838399229, -1.4831230121858061, 0.4074968533483106, -4.804328971504141]
        joint_position7 = [0,0,0,0,0,0]
        joint_positions = [joint_position1, joint_position2, joint_position3, joint_position4, joint_position5, joint_position6, joint_position7]
        return joint_positions


    def move_fk_demo(self):
        rospy.loginfo("正在进行正运动学规划")
        # 清除之前的轨迹
        self.clear_trajectory()
        joint_positions = self.joints()
        # 创建一个监听器，用于实时记录轨迹点
        trajectory_recorder = rospy.Timer(rospy.Duration(0.05), 
                                     lambda e: self.record_trajectory_point())
        for joint_position in joint_positions:
            self.arm.set_joint_value_target(joint_position)
            self.arm.go()
            rospy.sleep(1)
        # 停止记录
        trajectory_recorder.shutdown()
        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    test = MoveItFkDemo()
    test.move_fk_demo()
