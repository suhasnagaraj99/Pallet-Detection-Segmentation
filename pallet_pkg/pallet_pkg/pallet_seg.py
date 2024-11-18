import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class YoloInferenceNode(Node):
    
    def __init__(self):
        super().__init__('yolo_node')

        self.model = YOLO('/root/yolo_segmentation/best.pt')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image,'/robot1/zed2i/left/image_rect_color',self.listener_callback,qos_profile_sensor_data)

        self.seg_publisher = self.create_publisher(Image, '/segmented_feed', 10)
        
        self.det_publisher = self.create_publisher(Image, '/detected_objects', 10)
        
        self.subscription_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.listener_callback_depth, qos_profile_sensor_data)

        self.depth_image = None

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(cv_image)

        bounding_boxes_image = cv_image.copy()
        segmented_image = cv_image.copy()

        for result in results:
            if hasattr(result.masks, 'data'):
                for mask, cls in zip(result.masks.data, result.boxes.cls):
                    binary_mask = mask.cpu().numpy().astype(np.uint8) * 255
                    resized_mask = cv2.resize(binary_mask, (cv_image.shape[1], cv_image.shape[0]))

                    colored_mask = np.zeros_like(cv_image)
                    if int(cls) == 0:
                        colored_mask[:, :] = [255, 0, 0]
                    elif int(cls) == 1:
                        colored_mask[:, :] = [0, 255, 0]
                    else:
                        continue

                    colored_mask = cv2.bitwise_and(colored_mask, colored_mask, mask=resized_mask)

                    alpha = 0.4
                    segmented_image = cv2.addWeighted(segmented_image, 1 - alpha, colored_mask, alpha, 0)

            for box, conf, cls in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
                x1, y1, x2, y2 = map(int, box)
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
                cv2.putText(bounding_boxes_image, f"{text} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        segmented_image_msg = self.bridge.cv2_to_imgmsg(segmented_image, 'bgr8')
        bounding_boxes_msg = self.bridge.cv2_to_imgmsg(bounding_boxes_image, 'bgr8')

        self.seg_publisher.publish(segmented_image_msg)
        self.det_publisher.publish(bounding_boxes_msg)
        
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
