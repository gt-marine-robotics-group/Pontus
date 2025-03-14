import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from pontus_msgs.msg import YOLOResultArray, YOLOResult
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import torch
from typing import Optional, List


class YOLONode(Node):
    def __init__(self):
        super().__init__('perception_YOLO')

        self.declare_parameters(namespace='', parameters=[
            ('auv', ''),
            ('threshold', '0.5')
        ])

        pkg_share = get_package_share_directory('pontus_perception')

        model_path = pkg_share + '/yolo/' + self.get_parameter('auv').value + '/model.pt'

        self.cv_bridge = CvBridge()

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using {self.device}")
        self.model = YOLO(model_path).to(self.device)

        self.threshold = 0.6

        self.image_sub = self.create_subscription(
            Image,
            'input',
            self.image_callback,
            10
        )
        self.image_pub = self.create_publisher(
            Image,
            'yolo_debug',
            10
        )
        self.results_pub = self.create_publisher(
            YOLOResultArray,
            'results',
            10
        )

    def image_callback(self, msg: Image) -> None:
        """
        Take in an image and run YOLO on the image.

        Args:
        ----
            msg (Image): the image we want to run YOLO on

        Return:
        ------
            None

        """
        bgr = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(bgr)[0]

        result_array = YOLOResultArray()

        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, conf, class_id = result

            if conf < self.threshold:
                continue

            conf_score = round(conf, 2)
            label = f"{results.names[int(class_id)]}: {conf_score}"
            cv2.rectangle(bgr, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
            cv2.putText(bgr, label, (int(x1), int(y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)

            r = YOLOResult()
            r.x1 = x1
            r.y1 = y1
            r.x2 = x2
            r.y2 = y2
            r.class_id = int(class_id)
            r.label = results.names[int(class_id)]
            r.confidence = conf
            result_array.results.append(r)

        self.results_pub.publish(result_array)
        ros_image = self.cv_bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        self.image_pub.publish(ros_image)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)

    node = YOLONode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
