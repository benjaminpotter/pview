/// pview.cpp
///
/// Ben Potter
/// 14 Aug 2024 

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#define CCOMPASS_IMPLEMENTATION
#include "ccompass.h"

// #define M_PI_2 1.57079632679


const std::string NAME = "pview";
const std::string TOPIC = "/arena_camera_node/image_raw";
const std::string RAW_WINDOW_NAME = "raw";
const std::string AZI_WINDOW_NAME = "azimuth";
const double RAW_SCL = 0.25;
const double AZI_SCL = 0.50;


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv::Mat raw_image;
    raw_image = cv_bridge::toCvShare(msg, "mono8")->image;

    cv::Size s = raw_image.size();
    int w = s.width/2, h = s.height/2;

    // TODO Like, don't allocate this on the fly..
    struct cc_stokes *stokes_vectors;
    stokes_vectors = (struct cc_stokes*) malloc(sizeof(struct cc_stokes) * w * h);

    cc_compute_stokes(raw_image.data, stokes_vectors, w, h);
    cc_transform_stokes(stokes_vectors, w, h);

    double *aolps;
    aolps = (double*) malloc(sizeof(double) * w * h);
    cc_compute_aolp(stokes_vectors, aolps, w, h);

    unsigned char *pix;
    pix = (unsigned char*) malloc(sizeof(unsigned char) * w * h * 4);
    cc_compute_cmap(aolps, w * h, -M_PI_2, M_PI_2, (struct cc_color*) pix);

    cv::Mat raw_image_scaled;
    cv::resize(raw_image, raw_image_scaled, cv::Size(), RAW_SCL, RAW_SCL, cv::InterpolationFlags::INTER_AREA);
    cv::imshow(RAW_WINDOW_NAME, raw_image_scaled);

    cv::Mat azimuth_image(h, w, CV_8UC4, pix);
    cv::Mat azimuth_image_scaled;
    cv::resize(azimuth_image, azimuth_image_scaled, cv::Size(), AZI_SCL, AZI_SCL, cv::InterpolationFlags::INTER_AREA);
    cv::imshow(AZI_WINDOW_NAME, azimuth_image_scaled);
    cv::waitKey(30);

    free(pix);
    free(aolps);
    free(stokes_vectors);
}


int main(int argc, char* argv[]) {

    // Makes the ROS ecosystem aware of this node.
    ros::init(argc, argv, NAME);
    ROS_INFO("pview init");

    // Allows communication with the ROS ecosystem.
    // Also defines the lifecycle of the node.
    ros::NodeHandle handle;

    // Creates a CV2 window with name.
    cv::namedWindow(RAW_WINDOW_NAME, cv::WindowFlags::WINDOW_NORMAL);
    cv::namedWindow(AZI_WINDOW_NAME, cv::WindowFlags::WINDOW_NORMAL);

    image_transport::ImageTransport transport(handle);
    image_transport::Subscriber subscriber;
    subscriber = transport.subscribe(TOPIC, 1, imageCallback);
    
    // Blocks until node is closed via CTRL-C or shutdown is requested by master.
    // Allows callbacks to be invoked.
    ros::spin();

    // Clean up.
    cv::destroyAllWindows();
    ROS_INFO("pview cleaned");

    return 0;
}
