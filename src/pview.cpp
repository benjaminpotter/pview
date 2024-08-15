/// pview.cpp
///
/// Ben Potter
/// 14 Aug 2024 

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


const std::string NAME = "pview";
const std::string TOPIC = "/arena_camera_node/image_raw";
const std::string WINDOW_NAME = "pview";
const double IMAGE_SCALE_FACTOR = 0.25;


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv::Mat image;
    image = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat image_scaled;
    cv::resize(image, image_scaled, image_scaled.size(), IMAGE_SCALE_FACTOR, IMAGE_SCALE_FACTOR, cv::InterpolationFlags::INTER_AREA);
    cv::imshow(WINDOW_NAME, image_scaled);
    cv::waitKey(30);
}


int main(int argc, char* argv[]) {

    // Makes the ROS ecosystem aware of this node.
    ros::init(argc, argv, NAME);
    ROS_INFO("pview init");

    // Allows communication with the ROS ecosystem.
    // Also defines the lifecycle of the node.
    ros::NodeHandle handle;

    // Creates a CV2 window with name.
    cv::namedWindow(WINDOW_NAME, cv::WindowFlags::WINDOW_NORMAL);

    image_transport::ImageTransport transport(handle);
    image_transport::Subscriber subscriber;
    subscriber = transport.subscribe(TOPIC, 1, imageCallback);
    
    // Blocks until node is closed via CTRL-C or shutdown is requested by master.
    // Allows callbacks to be invoked.
    ros::spin();

    // Clean up.
    cv::destroyAllWindows();
    ROS_INFO("pview cleaned")

    return 0;
}
