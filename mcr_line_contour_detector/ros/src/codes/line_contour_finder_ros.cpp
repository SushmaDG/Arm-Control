#include <mcr_line_contour_detector/line_contour_finder_ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <mcr_perception_msgs/PointCloud2List.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tf/transform_listener.h"
#include "pcl_ros/transforms.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <std_msgs/Int32MultiArray.h>
#include <list>
#include <cmath>

LineContourFinderROS::LineContourFinderROS()
: nh_("~")
, pointcloud_msg_received_(false)
, publish_debug_image_(true)
{
    //dynamic_reconfigure_server_.setCallback(boost::bind(&LineContourFinderROS::dynamicReconfigCallback, this, _1, _2));
    image_transport::ImageTransport it(nh_);
    //Publishers
    pub_contour_pointclouds_ = nh_.advertise<mcr_perception_msgs::PointCloud2List>("output/pointclouds", 1);
    pub_contour_pointclouds_combined_ = nh_.advertise<sensor_msgs::PointCloud2>("output/pointclouds_combined", 1);
    pub_contour_pointclouds_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>("output/pointclouds_filtered", 1);
    pub_trajectory_original_ = nh_.advertise<geometry_msgs::PoseArray>("output/trajectory/original", 1);
    pub_debug_image_ = it.advertise("output/debug_image", 1);
    sub_event_in_ = nh_.subscribe("input/event_in", 1, &LineContourFinderROS::eventInCallback, this);
    pub_event_out_ = nh_.advertise<std_msgs::String>("output/event_out", 1);

    nh_.param<std::string> ("desired_frame", desired_frame_, "/base_link");
    nh_.param<bool> ("filter_contour", filter_contour_, true);
    nh_.param<float>("pcl_leaf_size", pcl_leaf_size_, 0.008);
    nh_.param<int>("min_x", min_x_, 230);
    nh_.param<int>("max_x", max_x_, 580);
    nh_.param<int>("min_y", min_y_, 200);
    nh_.param<int>("max_y", max_y_, 350);
    nh_.param<bool>("new_th", new_th_, true);
}

LineContourFinderROS::~LineContourFinderROS()
{
}

void LineContourFinderROS::update()
{
    if (pointcloud_msg_received_)
    {
        std::cout<<filter_contour_<<std::endl;
        std::cout<<min_x_<<", "<<max_x_<<", "<<min_y_<<", "<<max_y_<<std::endl;
        std::cout<<pcl_leaf_size_<<std::endl;
        findContours();
        pointcloud_msg_received_ = false;
    }
}

void LineContourFinderROS::pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    ROS_INFO("[line_contour_finder] Received pointcloud message");
    pointcloud_msg_ = msg;
    pointcloud_msg_received_ = true;
    sub_pointcloud_.shutdown();
}

void LineContourFinderROS::eventInCallback(const std_msgs::String &msg)
{
    std_msgs::String event_out;
    if (msg.data == "e_start")
    {
        sub_pointcloud_ = nh_.subscribe("input/pointcloud", 1, &LineContourFinderROS::pointcloudCallback, this);
        ROS_INFO("[line_contour_finder] Subscribed to pointcloud");
        //event_out.data = "e_started";
        //pub_event_out_.publish(event_out);
    }
    else if (msg.data == "e_stop")
    {
        sub_pointcloud_.shutdown();
        event_out.data = "e_stopped";

        pub_event_out_.publish(event_out);

    }
    else
    {
        return;
    }
}

/* void LineContourFinderROS::dynamicReconfigCallback(mcr_contour_matching::ContourFinderConfig &config, uint32_t level)
{
    contour_finder_.setCannyThreshold(config.canny_threshold);
    contour_finder_.setCannyMultiplier(config.canny_multiplier);
} */

void LineContourFinderROS::findContours()
{
    pcl::PCLPointCloud2::Ptr pcl_input_cloud(new pcl::PCLPointCloud2);

    // Convert to PCL data type
    pcl_conversions::toPCL(*pointcloud_msg_, *pcl_input_cloud);

    pcl_input_cloud->header.frame_id = pointcloud_msg_->header.frame_id;

    pcl::PCLImage pcl_image;
    pcl::toPCLPointCloud2(*pcl_input_cloud, pcl_image);

    sensor_msgs::ImagePtr image_msg = boost::make_shared<sensor_msgs::Image>();
    pcl_conversions::moveFromPCL(pcl_image, *image_msg);
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

    //event_out for publishing success or failure
    std_msgs::String event_out;

    cv::Mat debug_image;
    std::vector<std::vector<cv::Point> > contours;

    if(!new_th_)
    {
        contours = line_contour_finder_.find2DContours(cv_image->image, debug_image);
    }else
    {
        contours = line_contour_finder_.find2DContoursTest(cv_image->image, debug_image);
    }

    ROS_INFO("[line_contour_finder] Found %i 2D contours", (int)contours.size());
    if (contours.size() == 0)
    {
        event_out.data = "e_failure";
        pub_event_out_.publish(event_out);
        return;
    }

    //Put the contour size to array for state machine
    std_msgs::Int32MultiArray contour_size_msg;
    contour_size_msg.data.resize(2);
    //Set initial contour size to 0
    contour_size_msg.data[0] = 0;
    contour_size_msg.data[1] = 0;
    contour_size_msg.data[0] = (int) contours.size();


    //publish 2d img debug
    if (publish_debug_image_)
    {
        cv_bridge::CvImage debug_image_msg;
        debug_image_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_image_msg.image = debug_image;
        pub_debug_image_.publish(debug_image_msg.toImageMsg());
    }


    //Generate 3d contours and filter (take only those inside ROI)
    std::vector<pcl::PCLPointCloud2::Ptr> pcl_contours;
    if (filter_contour_)
    {
        std::vector<cv::Point> contour_;
        cv::Point point_;

        contour_ = contours[0];
        std::vector<cv::Point> filtered_contour;
        for(int i=0; i<contour_.size(); i++)
        {
            point_ = contour_[i];
            if (point_.x >= min_x_ && point_.y >= min_y_ && point_.x <= max_x_ && point_.y <= max_y_)
            {
                filtered_contour.push_back(point_);
            }
        }
        std::vector<std::vector<cv::Point> > filtered_contours;
        filtered_contours.push_back(filtered_contour);
        pcl_contours = line_contour_finder_.get3DContours(filtered_contours, pcl_input_cloud);
    }
    else
    {
      pcl_contours = line_contour_finder_.get3DContours(contours, pcl_input_cloud);
    }

    ROS_INFO("[line_contour_finder] Found %i 3D contours", (int)pcl_contours.size());
    if (pcl_contours.size() == 0)
    {
        event_out.data = "e_failure";
        pub_event_out_.publish(event_out);
        return;
    }

    //Set number of 3d contour to data and publish
    contour_size_msg.data[1] = pcl_contours.size();
    //pub_contour_size_.publish(contour_size_msg);
    if (contour_size_msg.data[0] != contour_size_msg.data[1])
    {
        event_out.data = "e_failure";
        pub_event_out_.publish(event_out);
        ROS_INFO("[line_contour_finder] Failed to match 2d and 3d contour");
        return;
    }

    int pcl_size = pcl_contours[0]->height * pcl_contours[0]->width;
    ROS_INFO("[line_contour_finder] Original pcl size: %i", pcl_size);

    //Downsample or subsample pointcloud
    pcl::PCLPointCloud2::Ptr pcl_filtered(new pcl::PCLPointCloud2 ());
    //create filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (pcl_contours[0]);
    sor.setLeafSize (pcl_leaf_size_, pcl_leaf_size_, pcl_leaf_size_);
    sor.filter (*pcl_filtered);

    pcl_size = pcl_filtered->height * pcl_filtered->width;
    ROS_INFO("[line_contour_finder] Downsampled pcl size: %i", pcl_size);

    //Transform using tf
    sensor_msgs::PointCloud2 ros_pcl_transformed;
    ros_pcl_transformed.header.frame_id = desired_frame_;

    sensor_msgs::PointCloud2 ros_pcl;
    pcl_conversions::fromPCL(*pcl_filtered, ros_pcl);
    ros_pcl.header.frame_id = pointcloud_msg_->header.frame_id;
    try
    {
        ros::Time common_time;
        transform_listener_.getLatestCommonTime(desired_frame_, ros_pcl.header.frame_id, common_time, NULL);
        ros_pcl.header.stamp = common_time;
        transform_listener_.waitForTransform(ros_pcl_transformed.header.frame_id, ros_pcl.header.frame_id, ros::Time::now(), ros::Duration(1.0));
        pcl_ros::transformPointCloud(ros_pcl_transformed.header.frame_id, ros_pcl, ros_pcl_transformed, transform_listener_);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("Tf error: %s", ex.what());
        event_out.data = "e_failure";
        pub_event_out_.publish(event_out);
        ros::Duration(1.0).sleep();
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_pcl (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(ros_pcl_transformed, *xyz_pcl);

    geometry_msgs::PoseArray trajectory_original_msg_;

    trajectory_original_msg_.header.frame_id = desired_frame_;
    trajectory_original_msg_.poses.resize(pcl_size);


    double sum_z_axis = 0;
    for (int i=0; i<pcl_size; i++)
    {
        trajectory_original_msg_.poses[i].position.x = xyz_pcl->points[i].x;
        trajectory_original_msg_.poses[i].position.y = xyz_pcl->points[i].y;
        trajectory_original_msg_.poses[i].position.z = xyz_pcl->points[i].z;
        sum_z_axis = sum_z_axis + xyz_pcl->points[i].z;
    }

    double z_axiz_mean = sum_z_axis / pcl_size;

    //Publish original trajectory
    pub_trajectory_original_.publish(trajectory_original_msg_);
    std::cout << "Published the poses" << '\n';
    //std::cout << trajectory_original_msg_ << '\n';

    geometry_msgs::PoseArray poses_;
    // line parameters
    //std::cout << "Slope and intercept between first two poses" << '\n';
    double threshold = 0.5;
    poses_.poses.push_back(trajectory_original_msg_.poses[0]);
    for(int i=1; i<pcl_size; i++)
    {
      double m1, c1, m2, c2;
      compute_line_parameters(m1, c1, trajectory_original_msg_.poses[0], trajectory_original_msg_.poses[i]);
      compute_line_parameters(m2, c2, trajectory_original_msg_.poses[0], trajectory_original_msg_.poses[i+1]);
      if((m2 <= (m1+threshold) && !(m2 < (m1 - threshold))) && (c2 <= (c1+threshold) && !(c2 < (c1 - threshold)))){
        poses_.poses.push_back(trajectory_original_msg_.poses[i+1]);
      }
    }

    // //Splitting the pose array into 5 line segments
    // geometry_msgs::PoseArray splitted_poses_1, splitted_poses_2, splitted_poses_3, splitted_poses_4, splitted_poses_5;
    // int split_len = 10;
    //
    // //Splitting the PoseArray into 5 parts: 10 X 3 + 14 x 2
    // for(int i=0; i<pcl_size; i++)
    // {
    //   splitted_poses_1.poses.push_back(trajectory_original_msg_.poses[i]);
    //   if(i%10==9){
    //   while(split_len>0){
    //     splitted_poses_2.poses.push_back(trajectory_original_msg_.poses[i]);
    //     split_len = split_len - 1;
    //   }
    //   //std::cout << i << '\n';
    //   break;
    //  }
    // }

    std::vector<geometry_msgs::Pose> poses_vec;
    std::vector<geometry_msgs::Pose> sorted_poses;

    for (int k = 0; k<trajectory_original_msg_.poses.size(); k++){
      poses_vec.push_back(trajectory_original_msg_.poses[k]);
    }
  //   std::cout << "Vector of poses - x, y, z" << '\n';
  //   for (std::vector<geometry_msgs::Pose>::const_iterator i = poses_vec.begin(); i != poses_vec.end(); ++i){
  //   std::cout << *i << '\t';
  // }
  std::cout << "Length of vector" << '\n';
  std::cout << poses_vec.size() << '\n';
  std::cout << poses_vec[0].position.x << '\n';

  //   int index;
  //   std::vector<float> distances;
  //   for(int j=0; j<pcl_size; j++){
  //     float smallesDistance = 1000.0;
  //     sorted_poses.push_back(poses_vec[0]);
  //
  //   for(int i=0; i<poses_vec.size(); i++){
  //     float dist_vals = sqrt(pow((sorted_poses.poses[j].position.x - poses.poses[i].position.x),2) +\
  //                           pow((sorted_poses.poses[j].position.y - poses.poses[i].position.y),2) + \
  //                           pow((sorted_poses.poses[j].position.z - poses.poses[i].position.z),2));
  //     if(dist_vals!=0){
  //       distances.push_back(dist_vals);
  //       if(distances.size()>1){
  //         //compare and sort
  //         if(distances[i]< smallesDistance){
  //           index = i;
  //           smallesDistance = distances[i];
  //         }
  //       }
  //     }
  //     sorted_poses.poses.push_back(trajectory_original_msg_.poses[index]);
  //     //delete that pose from list of poses
  //   }
  //   poses.poses.clear();
  // }


    // for(int i=0; i<distances.size(); i++){
    //   std::cout << distances[i] << '\n';
    // }

    // int counter = 0;
    // for(std::list<float>::iterator i=distances.begin(); i!=distances.end(); ++i)
    // {
    //   //Print the distace values from first pose to remaining
    //   std::cout << "Distance from 0 to" << '\t';
    //   std::cout << counter << '\t';
    //   std::cout << *i << '\n';
    //   counter = counter+1;
    // }

    event_out.data = "e_success";
    pub_event_out_.publish(event_out);

    publish_debug_image_ = true;
    if (publish_debug_image_)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_contour(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*(pcl_contours[0]), *xyz_contour);

        sensor_msgs::PointCloud2 ros_pointcloud;
        pcl::PCLPointCloud2::Ptr pcl_contour(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*xyz_contour, *pcl_contour);
        pcl_conversions::fromPCL(*pcl_contour, ros_pointcloud);

        ros_pointcloud.header = pointcloud_msg_->header;

        pub_contour_pointclouds_combined_.publish(pcl_contour);


        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_contour_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*pcl_filtered, *xyz_contour_filtered);

        sensor_msgs::PointCloud2 ros_pointcloud_filtered;
        pcl::PCLPointCloud2::Ptr pcl_contour_filtered(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*xyz_contour_filtered, *pcl_contour_filtered);
        pcl_conversions::fromPCL(*pcl_contour_filtered, ros_pointcloud_filtered);

        ros_pointcloud_filtered.header = pointcloud_msg_->header;

        pub_contour_pointclouds_filtered_.publish(pcl_contour_filtered);
    }
}

void LineContourFinderROS::compute_line_parameters(double & slope, double & intercept, geometry_msgs::Pose pose_1, geometry_msgs::Pose pose_2)
{
  //function to calculate the line parameters: slope (m) and intercept (c)
  double x1, x2, y1, y2;

  x1 = pose_1.position.x;
  y1 = pose_1.position.y;

  x2 = pose_2.position.x;
  y2 = pose_2.position.y;

  slope = (y2 - y1)/(x2 - x1);

  //calculating the intercept
  intercept = y1 - slope * x1;
}

double LineContourFinderROS::compute_distance(geometry_msgs::Pose current, geometry_msgs::Pose next)
{
  double x = current.position.x - next.position.x;
	double y = current.position.y - next.position.y;
	double distance;

	distance = pow(x, 2) + pow(y, 2);
	distance = sqrt(distance);
  return distance;
}
