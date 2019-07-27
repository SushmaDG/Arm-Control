#ifndef LINE_CONTOUR_FINDER_ROS_H_
#define LINE_CONTOUR_FINDER_ROS_H_

#include <ros/ros.h>
#include <mcr_line_contour_detector/line_contour_finder.h>

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include "tf/transform_listener.h"
#include <memory>

/**
 * ROS interface for contour finder
 * Subscribes to:
 *  -pointcloud: pointcloud in which to find contours
 *
 * Publishes:
 *  -contour pointclouds: pose array of the contour
 *  -debug image: showing the line contour
 */
class LineContourFinderROS
{
public:
    /**
     * Constructor
     */
    LineContourFinderROS();
    /**
     * Destructor
     */
    virtual ~LineContourFinderROS();
    /**
     * If pointcloud message has been received, the findContours function is called.
     * This function can be called once or periodically.
     */
    void update();

private:
    /**
     * Copy constructor.
     */
    //LineContourFinderROS(const LineContourFinderROS &other);

    /**
     * Copy assignment operator.
     */
    LineContourFinderROS &operator=(LineContourFinderROS other);

    /**
     * Callback for pointcloud. Saves the pointcloud message.
     *
     * @param msg
     *          sensor_msgs::PointCloud2 message
     */
    void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg);

    /**
     * Callback for event_in topic. Starts subscription to pointcloud if event_in is "e_trigger"
     */
    void eventInCallback(const std_msgs::String &msg);

    /**
     * Callback for dynamic reconfigure server to set canny threshold and multiplier
     */
    //void dynamicReconfigCallback(mcr_contour_matching::ContourFinderConfig &config, uint32_t level);

    /**
     * Finds 2D contours and the corresponding 3D contours and publishes the pose array of 3D contours as pointclouds
     */

    void findContours();

    /**
    * Given current pose and the next pose and compute euclidean distance
    */

    void compute_line_parameters(double & slope, double & intercept, geometry_msgs::Pose pose_1, geometry_msgs::Pose pose_2);

    /**
    * A function that returns the line parameters: slope and intercept given two poses.
    */

    double compute_distance(geometry_msgs::Pose current, geometry_msgs::Pose next);


private:
    /**
     * Object of ContourFinder
     */
    LineContourFinder line_contour_finder_;

    /**
     * Node handle
     */
    ros::NodeHandle nh_;

    /**
     * Subscriber for input pointcloud
     */
    ros::Subscriber sub_pointcloud_;

    /**
     * Subscriber for event_in topic
     */
    ros::Subscriber sub_event_in_;

    /**
    * Publisher for event_out topic
    */
    ros::Publisher pub_event_out_;

    /**
     * Publisher for 3D contours list
     */
    ros::Publisher pub_contour_pointclouds_;

    /**
     * Publisher for 3D contours as a single pointcloud
     */
    ros::Publisher pub_contour_pointclouds_combined_;

    /**
    * Publisher for filtered 3d contours
    */
    ros::Publisher pub_contour_pointclouds_filtered_;
    /**
     * Publisher for JointTrajectoryGoal msg
     */
    ros::Publisher pub_trajectory_original_;


    /**
     * Publisher for full trajectory
     */
    ros::Publisher pub_trajectory_full_;

    /**
     * Publisher for debug image showing 2d contour
     */
    image_transport::Publisher pub_debug_image_;

    /**
     * Used to store pointcloud message received in callback
     */
    sensor_msgs::PointCloud2::Ptr pointcloud_msg_;

    /**
     * Dynamic reconfigure server
     */
    //dynamic_reconfigure::Server<mcr_contour_matching::ContourFinderConfig> dynamic_reconfigure_server_;

    /**
     * Flag indicating whether pointcloud has been received
     */
    bool pointcloud_msg_received_;
    /**
     * Flag indicating whether debug image should be published
     */
    bool publish_debug_image_;

    /**
    * Create object for transform_listener_
    */
    tf::TransformListener transform_listener_;

    std::string desired_frame_;

    /**
    * Parameter for filtering contour in the ROI
    */
    bool filter_contour_;
    int min_x_;
    int max_x_;
    int min_y_;
    int max_y_;

    /**
    * Parameter for pcl leaf_size to downsample pcl
    */
    float pcl_leaf_size_;

    bool new_th_;
};

#endif
