import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        #init GPIO
        self.factory = PiGPIOFactory()
        PAN_SERVO_PIN = 23
        TILT_SERVO_PIN = 18

        self.tilt_servo = AngularServo(TILT_SERVO_PIN, min_pulse_width=0.0006, max_pulse_width=0.0023, pin_factory=self.factory)
        self.pan_servo = AngularServo(PAN_SERVO_PIN, min_pulse_width=0.0006, max_pulse_width=0.0023, pin_factory=self.factory)
        
        # Set servos to neutral position initially
        self.pan_servo.angle = 0  # Set to neutral position
        self.tilt_servo.angle = 0  # Set to neutral position
        
        self.create_subscription(Float32, 'pan_angle', self.pan_angle_callback, 10)
        self.create_subscription(Float32, 'tilt_angle', self.tilt_angle_callback, 10)
        
        self.get_logger().info("Servo Controller Node Initialized")
      
      #ros2 topic pub /pan_angle std_msgs/msg/Float32 "{data: 45.0}"  # Move pan servo to 45 #degrees
      #ros2 topic pub /tilt_angle std_msgs/msg/Float32 "{data: 30.0}"  # Move tilt servo to 30 #degrees  
    def pan_angle_callback(self, msg):
        self.move_servo(self.pan_servo, msg.data)
         
    def tilt_angle_callback(self, msg):
        self.move_servo(self.tilt_servo, msg.data)
        
    def move_servo(self, servo, target_angle):
        try:
            self.get_logger().info(f"Moving servo to {target_angle}")
            servo.angle = target_angle
            sleep(0.05)
            
        except Exception as e:
            self.get_logger().error(f"Error moving servo: {e}")
        
    
    def close_servos(self):
        # Close the servos when shutting down
        self.pan_servo.close()
        self.tilt_servo.close()
        self.get_logger().info("Servos closed")     

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    
    try:
        rclpy.spin(servo_controller)
    except KeyboardInterrupt:
        pass
    finally:
        servo_controller.close_servos()
        servo_controller.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()