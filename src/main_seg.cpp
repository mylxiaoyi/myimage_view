#include <ros/ros.h>
#include "segmentor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_seg_node");
    ros::NodeHandle nh;
    Segmentor seg(nh);
    ros::spin();
}
