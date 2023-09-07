#ifndef ROBOT_ODOM_H__
#define ROBOT_ODOM_H__

#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>  
#include <robot_msgs/lowlevel.h>  
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define X       0
#define Y       1
#define Z       2
#define THETA   2
#define ROLL    0
#define PITCH   1
#define YAW     2

class ROBOTODOM{
    public:
        ROBOTODOM();
        ~ROBOTODOM();
        bool init();
        bool update();
    
    private:
        // ROS Nodes
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;

        // ROS Time
        ros::Time prev_update_time_;

        // ROS Publishers
        ros::Publisher imu_pub_;
        ros::Publisher odom_pub_;
        
        // ROS Subscriber
        ros::Subscriber lowlevel_data_sub_;

        // ROS Variables
        sensor_msgs::Imu imu_;
        nav_msgs::Odometry odom_;
        tf::TransformBroadcaster tf_broadcaster_;

        double odom_pose_[3];
        double imu_angle_[3];

        double acc_[3];
        double gyr_[3];
        double vel_[3];

        // ROS Functions
        void commandDataCallback(const robot_msgs::lowlevelConstPtr data_msg);
        bool updateOdometry(ros::Duration diff_time);
        bool updateImu(ros::Duration diff_time);
        void updateTF(geometry_msgs::TransformStamped& odom_tf);
};

#endif