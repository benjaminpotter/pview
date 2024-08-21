/// @file pview.cpp
///
/// @author Ben Potter
/// @date 14 Aug 2024 

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define CCOMPASS_IMPLEMENTATION
#include "ccompass.h"


const std::string NAME = "pview";

// TODO: Use parameter server to set these values.
const std::string IMG_TOPIC = "/arena_camera_node/image_raw";
const std::string IMU_TOPIC = "/novatel/imu";

namespace pview {

const std::string RAW_WINDOW_NAME = "raw";
const std::string AZI_WINDOW_NAME = "azimuth";
const double RAW_SCL = 0.25;
const double AZI_SCL = 0.50;

class Node {
public:
    Node(const std::string imgTopic, const std::string imuTopic): 
        handle(),
        imgTransport(handle)
    {
        ROS_INFO("pview init");

        cv::namedWindow(RAW_WINDOW_NAME, cv::WindowFlags::WINDOW_NORMAL);
        cv::namedWindow(AZI_WINDOW_NAME, cv::WindowFlags::WINDOW_NORMAL);

        // TODO: Check that the topic refers to an image topic.

        imgSubscriber = imgTransport.subscribe(imgTopic, 1, &Node::handleImage, this);
        ROS_INFO("pview subscribed %s", imgTopic.c_str());

        imuSubscriber = handle.subscribe(imuTopic, 1, &Node::handleIMU, this); 
        ROS_INFO("pview subscribed %s", imuTopic.c_str());
    }
    
    ~Node() {
        ROS_INFO("pview clean up");

        cv::destroyAllWindows();
    }

    ///
    /// @brief Start the node event loop, blocks execution until ROS requests a shutdown.
    ///
    void spin() {

        while(ros::ok()) {

            // We should only draw the images if there is valid image data in the buffer.
            if(rawImgPtr != nullptr) {
                cv::Mat rawImgScl;
                cv::resize(rawImgPtr->image, rawImgScl, cv::Size(), RAW_SCL, RAW_SCL, cv::InterpolationFlags::INTER_AREA);
                cv::imshow(RAW_WINDOW_NAME, rawImgScl);
            }

            // We should only draw the images if there is valid image data in the buffer.
            if(aziImgPtr != nullptr) {
                cv::Mat aziImgScl;
                cv::resize(*aziImgPtr, aziImgScl, cv::Size(), AZI_SCL, AZI_SCL, cv::InterpolationFlags::INTER_AREA);
                cv::imshow(AZI_WINDOW_NAME, aziImgScl);
            }

            // Allow OpenCV to draw the image to the screen.
            cv::waitKey(30);

            // Process any pending events.
            ros::spinOnce();
        }

        // Program will exit. 
    }

    ///
    /// @brief Pop an image from the ROS topic buffer, process it, and cache the result locally.
    ///
    void handleImage(const sensor_msgs::ImageConstPtr& msg) {

        rawImgPtr = cv_bridge::toCvShare(msg, "mono8");

        cv::Size s = rawImgPtr->image.size();
        int w = s.width/2, h = s.height/2;

        struct cc_stokes *stokes_vectors;
        stokes_vectors = (struct cc_stokes*) malloc(sizeof(struct cc_stokes) * w * h);

        cc_compute_stokes(rawImgPtr->image.data, stokes_vectors, w, h);
        cc_transform_stokes(stokes_vectors, w, h);

        double *aolps;
        aolps = (double*) malloc(sizeof(double) * w * h);
        cc_compute_aolp(stokes_vectors, aolps, w, h);

        free(stokes_vectors);

        // Reference to the OpenCV documentation for cv::Mat.
        // https://docs.opencv.org/3.4/d3/d63/classcv_1_1Mat.html#a51615ebf17a64c968df0bf49b4de6a3a
        //
        // Specifically, regarding construction from existing data:
        //
        // Matrix constructors that take data and step parameters do not 
        // allocate matrix data. Instead, they just initialize the matrix 
        // header that points to the specified data, which means that no data is
        // copied. This operation is very efficient and can be used to process 
        // external data using OpenCV functions. 
        //
        // The external data is not automatically deallocated, so you should 
        // take care of it. 
        //
        // This implies we must manually free the pixel data. It will not be 
        // deallocated when the cv::Mat is freed by the unique pointer.

        if(aziImgPtr != nullptr)
            free(aziImgPtr->ptr());

        unsigned char *pix;
        pix = (unsigned char*) malloc(sizeof(unsigned char) * w * h * 4);
        cc_compute_cmap(aolps, w * h, -M_PI_2, M_PI_2, (struct cc_color*) pix);
        free(aolps);

        // Save a local cache of the AoLP colour plot.
        aziImgPtr = std::make_unique<cv::Mat>(cv::Mat(h, w, CV_8UC4, pix));
    }

    ///
    /// @brief Pop an IMU frame from the ROS topic buffer, process it, and cache the result locally.
    ///
    void handleIMU(const sensor_msgs::Imu::ConstPtr& data) {

        tf2::Quaternion q;
        tf2::convert(data->orientation, q);

        tf2::Matrix3x3 m(q);
        m.getRPY(orientation.x, orientation.y, orientation.z);

        ROS_INFO("orientation (roll, pitch, yaw): %f, %f, %f", orientation.x, 0, 0);
    }

private:

    // Allows communication with the ROS ecosystem.
    // Also defines the lifecycle of the node.
    ros::NodeHandle handle;

    // Handle images.
    image_transport::ImageTransport imgTransport;
    image_transport::Subscriber imgSubscriber;

    // Handle IMU.
    ros::Subscriber imuSubscriber;

    // Drawing state.
    cv_bridge::CvImageConstPtr rawImgPtr;
    std::unique_ptr<cv::Mat> aziImgPtr; 
    tf2::Vector3 rpy;
};
}

int main(int argc, char* argv[]) {

    // Makes the ROS ecosystem aware of this node.
    ros::init(argc, argv, NAME);

    // Create an instance of the Node class to hold node state.
    // We pass the node the topic names it should listen on.
    pview::Node node(IMG_TOPIC, IMU_TOPIC);
    node.spin();

    return 0;
}
