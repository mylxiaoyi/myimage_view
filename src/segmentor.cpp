#include "segmentor.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

Segmentor::Segmentor (ros::NodeHandle nh)
: nh_ (nh), bFirstImage (true),
  model_file ("/home/mylxiaoyi/source/deep_learning/fcn.berkeleyvision.org/"
              "nyud-fcn32s-color/deploy.prototxt"),
  trained_file ("/home/mylxiaoyi/source/deep_learning/fcn.berkeleyvision.org/"
                "nyud-fcn32s-color/nyud-fcn32s-color-heavy.caffemodel"),
  palette_b{ 0,  128, 0, 128, 0,  128, 0,  128, 64,  192, 64, 192, 64, 192,
             64, 192, 0, 128, 0,  128, 0,  0,   128, 128, 0,  128, 0,  128,
             0,  128, 0, 128, 96, 224, 96, 224, 96,  224, 96, 224 },
  palette_g{ 0,   0,   128, 128, 0,   0,   128, 128, 0,   0,
             128, 128, 0,   0,   128, 128, 64,  64,  192, 192,
             64,  192, 64,  192, 0,   0,   128, 128, 0,   0,
             128, 128, 0,   0,   0,   0,   128, 128, 128, 128 },
  palette_r{ 0,   0,   0, 0, 128, 128, 128, 128, 0,   0,  0,  0,  128, 128, 128,
             128, 0,   0, 0, 0,   128, 128, 128, 128, 64, 64, 64, 64,  192, 192,
             192, 192, 0, 0, 128, 128, 0,   0,   128, 128 }
{
    std::string image_channel = nh.resolveName ("image");
    image_sub = nh.subscribe (image_channel, 1, &Segmentor::imageCb, this);

    caffe::Caffe::set_mode (caffe::Caffe::GPU);

    net.reset (new caffe::Net<float> (model_file, caffe::TEST));
    net->CopyTrainedLayersFrom (trained_file);

    input_layer = net->input_blobs ()[0];
}


void Segmentor::imageCb (const sensor_msgs::ImageConstPtr& img_msg)
{
    cv::Mat image =
    cv_bridge::toCvCopy (img_msg, sensor_msgs::image_encodings::BGR8)->image;

    if (bFirstImage)
    {
        nrows = image.rows;
        ncols = image.cols;

        cv::Mat channel_mean_b (nrows, ncols, CV_32FC1, mean_b);
        cv::Mat channel_mean_g (nrows, ncols, CV_32FC1, mean_g);
        cv::Mat channel_mean_r (nrows, ncols, CV_32FC1, mean_r);

        std::vector<cv::Mat> channels;
        channels.push_back (channel_mean_b);
        channels.push_back (channel_mean_g);
        channels.push_back (channel_mean_r);

        cv::Mat mean;
        cv::merge (channels, mean);

        cv::Scalar channel_mean = cv::mean (mean);
        std::cout << "channel_mean = " << channel_mean << std::endl;

        mean_mat = cv::Mat (nrows, ncols, mean.type (), channel_mean);

        input_layer->Reshape (1, 3, nrows, ncols);
        net->Reshape ();

        height = input_layer->height();
        width = input_layer->width();

        bFirstImage = false;
    }

    cv::Mat img_normalized;
    cv::Mat img_float;
    image.convertTo (img_float, CV_32FC3);
    cv::subtract (img_float, mean_mat, img_normalized);

    std::vector<cv::Mat> input_channels;
    float* input_data = input_layer->mutable_cpu_data ();
    for (int i = 0; i < 3; i++)
    {
        cv::Mat channel (height, width, CV_32FC1, input_data);
        input_channels.push_back (channel);
        input_data += width * height;
    }

    cv::split (img_normalized, input_channels);

    net->Forward ();

    caffe::Blob<float>* output_layer = net->output_blobs ()[0];

    const float* begin = output_layer->cpu_data ();
    const float* end =
    begin + output_layer->channels () * output_layer->width () * output_layer->height ();

    std::vector<float> results (begin, end);

    std::vector<unsigned char> data;
    for (size_t i = 0; i < results.size (); i++)
    {
        int idx = results[i];
        data.push_back (palette_b[idx]);
        data.push_back (palette_g[idx]);
        data.push_back (palette_r[idx]);
    }

    cv::Mat seg_img (nrows, ncols, CV_8UC3, data.data ());

    cv::imshow ("image", image);
    cv::imshow ("seg_image", seg_img);
    cv::waitKey (1);
}
