#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

void vidCb(const sensor_msgs::ImageConstPtr & img) {
    cv::Mat cv_image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8)->image;

    cv::imshow("Image", cv_image);

    if (cv::waitKey(5) == 27)
        ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "myimage_view");

    ros::NodeHandle nh;

    string vid_channel = nh.resolveName("image");

    ros::Subscriber vid_sub = nh.subscribe(vid_channel, 1, &vidCb);

    ros::spin();

    std::cout << "Done" << std::endl;

    return 0;
}
