import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from mono_optical_flow_calc import MonoOpticalFlowCalculations

from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class MonoOpticalFlowOdomNode(Node):
    def __init__(self):
        super().__init__("odometry_node")
        print("INIT")
        self.image_count = 0
        self.old_image = None
        self.new_image = None
        self.bridge = CvBridge()
        # self.cam_body_tf = None
        self.prev_time = None
        self.mono_calculator = MonoOpticalFlowCalculations()

        # self.declare_parameters(namespace = "", parameters=[
        #     ("body_id", "base_link"),
        #     ("cam_id", "camera")
        # ])
        
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            depth = 10
        )

        # Subscriptions
        self.camera_sub = self.create_subscription(
            Image,
            "/pontus/camera_1", # this will get remapped in launch file
            self.camera_sub_callback,
            qos_profile = qos_profile
        )

        # self.tf_sub = self.create_subscription(
        #     TFMessage,
        #     "/tf_static",
        #     self.tf_sub_callback,
        #     10
        # )

        

        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            "/odometry", # remap to whatever later
            10
        )

    def camera_sub_callback(self, image: Image):
        print("RUNNNING")
        curr_time = self.get_clock().now()
        prev_time = self.prev_time
        self.prev_time = curr_time
        cv_image = self.bridge.imgmsg_to_cv2(image)
        if self.image_count == 0:
            self.old_image = cv_image
        else:
            if self.image_count == 1:
                self.new_image = cv_image
            else:
                self.old_image = self.new_image
                self.new_image = cv_image
            

            
            # CALL ACTUAL ODOM FUNCTIONALITY HERE
            lin_vel, ang_vel = self.mono_calculator.calculate(self.image_count, self.old_image, self.new_image, prev_time, curr_time)
            message = Odometry()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "camera_1"
            message.child_frame_id = "camera_1"

            message.twist.twist.linear.x = lin_vel[0]
            message.twist.twist.linear.y = lin_vel[1]
            message.twist.twist.linear.z = lin_vel[2]

            message.twist.twist.angular.x = ang_vel[0]
            message.twist.twist.angular.y = ang_vel[1]
            message.twist.twist.angular.z = ang_vel[2]

            self.odom_pub.publish(message)
        self.image_count += 1

    # def tf_sub_callback(self, tf: TFMessage):
    #     for tf_stamped in tf.transforms:
    #         header = tf_stamped.header
    #         frame_id = header.frame_id
    #         child_frame_id = tf_stamped.child_frame_id
    #         if (frame_id == self.get_parameter("body_id") and child_frame_id == self.get_parameter("cam_id")):
    #             self.cam_body_tf = tf_stamped.transform
    #             break
    #     if (self.cam_body_tf is not None):
    #         self.destroy_subscription(self.tf_sub)

def main(args = None):
    rclpy.init(args=args)
    node = MonoOpticalFlowOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()