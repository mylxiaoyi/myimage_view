#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>
#include <opencv2/opencv.hpp>

using namespace sensor_msgs;
using namespace message_filters;

void vidCb(const sensor_msgs::ImagePtr img) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

    cv::imshow("Image", cv_ptr->image);

    if (cv::waitKey(5) == 27)
        ros::shutdown();
}

void callback(const ImageConstPtr &color_ptr, const ImageConstPtr &depth_ptr, const CameraInfoConstPtr &cinfo_ptr) {
    std::cout << "hello in callback" << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "myimage_view");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> color_sub(nh, "color_image", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "depth_image", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub(nh, "camera_info", 1);

    typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color_sub, depth_sub, cinfo_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    ros::spin();

    return 0;
}
