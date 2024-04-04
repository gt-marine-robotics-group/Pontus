import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class PreprocessingNode(Node):
    def __init__(self):
        super().__init__('perception_preprocessing')
        
        self.declare_parameters(namespace='', parameters=[
            ('auv', '')
        ])

        self.cv_bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, 'input', 
            self.image_preprocessing, 10)
        
        self.image_pub = self.create_publisher(Image, 'output', 10)


    def image_preprocessing(self, msg: Image):
        # Read the image
        img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # img = np.float32(img)/255.0

        # Apply Gaussian blur to reduce noise
        img_blur = cv2.GaussianBlur(img, (29, 29), 0)

        # Deblurring using deconvolution
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        img_deblur = cv2.filter2D(img_blur, -1, kernel)

        # Sharpening the image
        img_sharpen = cv2.addWeighted(img, 1.5, img_deblur, -0.5, 0)

        # Contrast enhancement
        img_contrast = cv2.convertScaleAbs(img_sharpen, alpha=1.2, beta=0)

        processed_image = self.cv_bridge.cv2_to_imgmsg(img_contrast, encoding='bgr8')
        
        self.image_pub.publish(processed_image)

def main(args=None):
    rclpy.init(args=args)

    node = PreprocessingNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

