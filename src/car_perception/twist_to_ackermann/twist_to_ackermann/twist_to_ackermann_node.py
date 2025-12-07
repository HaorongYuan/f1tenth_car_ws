import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped  # 只保留需要的类型

class TwistToAckermannNode(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('drive_topic', 'drive')
        self.declare_parameter('frame_id', 'base_link')

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # 订阅Twist消息
        self.subscription = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.twist_callback,
            10)
        
        # 只创建AckermannDriveStamped类型的发布者（与订阅者ackermann_mux匹配）
        self.publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.get_logger().info('Twist to Ackermann node initialized (only Stamped type)')

    def twist_callback(self, msg):
        # 只使用AckermannDriveStamped类型（带header，更规范）
        ackermann_stamped_msg = AckermannDriveStamped()
        # 填充header（时间戳和坐标系）
        ackermann_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_stamped_msg.header.frame_id = self.frame_id
        # 填充驱动数据（速度和转向角）
        ackermann_stamped_msg.drive.speed = msg.linear.x
        ackermann_stamped_msg.drive.steering_angle = msg.angular.z  # 假设输入是角度（如TEB输出）

        # 限制速度和转向角（只针对Stamped消息的drive属性）
        ackermann_stamped_msg.drive.speed = max(min(ackermann_stamped_msg.drive.speed, 2.0), -2.0)
        ackermann_stamped_msg.drive.steering_angle = max(min(ackermann_stamped_msg.drive.steering_angle, 0.7), -0.7)

        # 发布Stamped类型消息
        self.publisher.publish(ackermann_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermannNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
