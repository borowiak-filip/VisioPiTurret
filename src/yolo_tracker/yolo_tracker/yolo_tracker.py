import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

YOLO_CLASS_IDS = {
    'person': 0,
    'cell phone': 67,  # YOLOv8 uses COCO class IDs
}

class YOLOTracker(Node):
    def __init__(self):
        super().__init__('yolo_tracker')
        
        self.declare_parameter("target_class", "person")
        self.target_class = self.get_parameter("target_class").get_parameter_value().string_value
        self.target_id = YOLO_CLASS_IDS.get(self.target_class, 0)

        self.get_logger().info(f"Tracking class: {self.target_class} (ID {self.target_id})")
        

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )

        self.annotated_image_pub = self.create_publisher(Image, 'yolo/image_annoated', 10)
        self.pan_pub = self.create_publisher(Float32, 'pan_angle', 10)
        self.tilt_pub = self.create_publisher(Float32, 'tilt_angle', 10)

        self.model = YOLO('yolov8n.pt')  # Nano model (fastest)

        self.frame_width = 640
        self.frame_height = 480

        self.current_pan = 0.0
        self.current_tilt = 0.0

        self.get_logger().info("YOLO Tracker Node Started")
        
        self.timer = self.create_timer(0.02, self.update_servo)
        
        self.last_dx = 0
        self.last_dy = 0
        
    def update_servo(self):
        dx = self.last_dx
        dy = self.last_dy
        
        if dx != 0 or dy !=0:
            self.adjust_servo(dx, dy)
            

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, verbose=False)[0]

        targets = [det for det in results.boxes.data.tolist() if int(det[5]) == self.target_id]  # class 0 = person

        if targets:
            x1, y1, x2, y2, conf, cls = targets[0]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # Draw bounding box
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # Calculate offset from center
            dx = cx - self.frame_width // 2
            dy = cy - self.frame_height // 2

            self.last_dx = dx
            self.last_dy = dy
            
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.annotated_image_pub.publish(img_msg)

        #cv2.imshow("YOLO View", frame)
        #cv2.waitKey(1)

    def adjust_servo(self, dx, dy):
        kp = 0.05  # tuning parameter
        self.current_pan -= dx * kp
        self.current_tilt += dy * kp

        self.current_pan = max(-90.0, min(90.0, self.current_pan))
        self.current_tilt = max(-90.0, min(90.0, self.current_tilt))

        self.pan_pub.publish(Float32(data=self.current_pan))
        self.tilt_pub.publish(Float32(data=self.current_tilt))


def main(args=None):
    rclpy.init(args=args)
    node = YOLOTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
