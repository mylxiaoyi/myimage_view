#ifndef SEGMENTOR_H
#define SEGMENTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <caffe/caffe.hpp>

class Segmentor
{
public:
    Segmentor (ros::NodeHandle nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub;

    void imageCb (const sensor_msgs::ImageConstPtr& img_msg);

    bool bFirstImage;
    cv::Mat mean_mat;
    int nrows, ncols;

    std::string model_file;
    std::string trained_file;

    float mean_b = 116.190;
    float mean_g = 97.203;
    float mean_r = 92.318;

    std::shared_ptr<caffe::Net<float>> net;
    caffe::Blob<float>* input_layer;
    int height, width;

    unsigned char palette_b[40];
    unsigned char palette_g[40];
    unsigned char palette_r[40];
};

#endif // SEGMENTOR_H
