#include <ros/ros.h>
#include <mcr_line_contour_detector/line_contour_finder_ros.h>
#include "tf/transform_listener.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_contour_finder_node");
    ros::NodeHandle nh("~");

    int frame_rate = 30;    // in Hz
    LineContourFinderROS contour_finder_ros_;

    ROS_INFO("[contour_finder] node started");
    ros::Rate loop_rate(frame_rate);

    while (ros::ok())
    {
        contour_finder_ros_.update();
        //contour_finder_->update();

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
