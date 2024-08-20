/// pview.cpp
///
/// Ben Potter
/// 14 Aug 2024 

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

        // Allows communication with the ROS ecosystem.
        // Also defines the lifecycle of the node.
        // ros::NodeHandle handle;

        // Creates a CV2 window with name.
        cv::namedWindow(RAW_WINDOW_NAME, cv::WindowFlags::WINDOW_NORMAL);
        cv::namedWindow(AZI_WINDOW_NAME, cv::WindowFlags::WINDOW_NORMAL);

        // TODO: Check that the topic refers to an image topic.

        // Subscribe to the image topic. 
        // The callback belongs to the node class as a member function.
        // We pass a reference to the function and a reference to the instance of Node which
        // should handle a function call.
        imgSubscriber = imgTransport.subscribe(imgTopic, 1, &Node::handleImage, this);
        ROS_INFO("pview subscribed %s", imgTopic.c_str());

        // Subscribe to the IMU topic to recieve attitude data.
        imuSubscriber = handle.subscribe(imuTopic, 1, &Node::handleIMU, this); 
        ROS_INFO("pview subscribed %s", imuTopic.c_str());
    }
    
    ~Node() {
        ROS_INFO("pview clean up");

        // Clean up.
        cv::destroyAllWindows();
    }

    void spin() {

        while(ros::ok()) {

            // 1 frame was drawn in the time between this current frame and the last frame.
            //double fps = 1 / (e.current_real.toSec() - e.last_real.toSec());

            // We should only draw the images if there is valid image data in the buffer.
            if(rawImgPtr != nullptr) {
                cv::Mat rawImgScl;
                cv::resize(rawImgPtr->image, rawImgScl, cv::Size(), RAW_SCL, RAW_SCL, cv::InterpolationFlags::INTER_AREA);
                cv::imshow(RAW_WINDOW_NAME, rawImgScl);
            }

            if(aziImgPtr != nullptr) {
                cv::Mat aziImgScl;
                cv::resize(*aziImgPtr, aziImgScl, cv::Size(), AZI_SCL, AZI_SCL, cv::InterpolationFlags::INTER_AREA);
                cv::imshow(AZI_WINDOW_NAME, aziImgScl);
            }

            cv::waitKey(30);

            // Process any pending events.
            ros::spinOnce();
        }

        // Program will exit. 
    }

    void handleImage(const sensor_msgs::ImageConstPtr& msg) {
        // Called by ROS when a new image is published.
        // We collect the image from the buffer, parse it, and save the result
        // to our local buffer.

        // Save shared pointer to image message.
        // By setting this pointer, we are letting the pointer to the last image message
        // go out of scope. The shared pointer will handle destroying the underlying
        // object, hopefully.
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

        // https://docs.opencv.org/3.4/d3/d63/classcv_1_1Mat.html#a51615ebf17a64c968df0bf49b4de6a3a
        // According to the documentation, passing a pointer to external pixel data requires the caller
        // to manually free the data after use. This process is normally done when the cv::Mat is 
        // deconstructed.
        //
        // Specifically, regarding the data pointer (pix):
        // Pointer to the user data. Matrix constructors that take data and step parameters do not 
        // allocate matrix data. Instead, they just initialize the matrix header that points to the 
        // specified data, which means that no data is copied. This operation is very efficient and 
        // can be used to process external data using OpenCV functions. The external data is not automatically 
        // deallocated, so you should take care of it. 
        
        // There might be a better way to do this.
        if(aziImgPtr != nullptr)
            // Release pix from the last image.
            free(aziImgPtr->ptr());

        unsigned char *pix;
        pix = (unsigned char*) malloc(sizeof(unsigned char) * w * h * 4);
        cc_compute_cmap(aolps, w * h, -M_PI_2, M_PI_2, (struct cc_color*) pix);

        // Create a unique pointer to this cv::Mat.
        aziImgPtr = std::make_unique<cv::Mat>(cv::Mat(h, w, CV_8UC4, pix));

        free(aolps);
        free(stokes_vectors);
    }

    void handleIMU(const sensor_msgs::Imu::ConstPtr& data) {

        // The input argument is a *pointer* to the structure. Use accordingly.

        tf2::Quaternion q;
        tf2::convert(data->orientation, q);

        tf2::Matrix3x3 m(q);

        double r,p,y;
        m.getRPY(r, p, y);

        r *= 180 / M_PI;
        p *= 180 / M_PI;
        y *= 180 / M_PI;

        ROS_INFO("orientation (roll, pitch, yaw): %f, %f, %f", r, p, y);
    }

private:
    ros::NodeHandle handle;
    image_transport::ImageTransport imgTransport;
    image_transport::Subscriber imgSubscriber;
    ros::Subscriber imuSubscriber;

    cv_bridge::CvImageConstPtr rawImgPtr;
    std::unique_ptr<cv::Mat> aziImgPtr; 
    tf2::Vector3 orientation;
};

}

int main(int argc, char* argv[]) {

    // Makes the ROS ecosystem aware of this node.
    ros::init(argc, argv, NAME);

    // Create an instance of the Node class to hold node state.
    // We pass the node the topic names it should listen on.
    pview::Node node(IMG_TOPIC, IMU_TOPIC);
    
    // Blocks until node is closed via CTRL-C or shutdown is requested by master.
    // Allows callbacks to be invoked.
    node.spin();

    return 0;
}
