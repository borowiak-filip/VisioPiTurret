#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <std_msgs/msg/float32.hpp>
#include <algorithm>

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher() : Node("camera_publisher"), cap(0) {
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
            rclcpp::shutdown();
        }

        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap.set(cv::CAP_PROP_FPS, 30);

        publisher_ = image_transport::create_publisher(this, "camera/image");

        //! Comment this out for manual control
        //pan_pub_ = this->create_publisher<std_msgs::msg::Float32>("pan_angle", 10);
        //tilt_pub_ = this->create_publisher<std_msgs::msg::Float32>("tilt_angle", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CameraPublisher::capture_frame, this)
        );

    }

private:
    void capture_frame() {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) return;

       
        std_msgs::msg::Header header;
        header.stamp = this->now();

        auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        publisher_.publish(msg);

    }
    cv::VideoCapture cap;
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}

