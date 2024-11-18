import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import numpy as np

class YoloInferenceNode(Node):
    
    def __init__(self):
        super().__init__('yolo_node')

        self.model = YOLO('/root/yolo_segmentation/best.pt')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image,'/camera/image_raw',self.listener_callback,10)
        
        self.subscription_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.listener_callback_depth, 10)

        self.depth_image = None

    def listener_callback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(cv_image)

        results.render()
        
        processed_image = results.imgs[0]
        
        bounding_boxes_image = cv_image.copy()
        
        for box, conf, cls in zip(results.xywh[0], results.conf[0], results.cls[0]):
            
            x1, y1, x2, y2 = map(int, box[:4])
            
            label = int(cls)
            
            if label == 0:
                color = (255, 0, 0)
                text = "Pallet"
            elif label == 1:
                color = (0, 255, 0)
                text = "Ground"
            else:
                continue

            cv2.rectangle(bounding_boxes_image, (x1, y1), (x2, y2), color, 2)

            cv2.putText(bounding_boxes_image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow('Seg', processed_image)

        cv2.imshow('Bounding Boxes', bounding_boxes_image)
        
        cv2.waitKey(1)
        
    def listener_callback_depth(self, msg):
        
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        
        
def main(args=None):
    rclpy.init(args=args)

    yolo_inference_node = YoloInferenceNode()

    rclpy.spin(yolo_inference_node)
    
    yolo_inference_node.destroy_node()
    
    cv2.destroyAllWindows()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
