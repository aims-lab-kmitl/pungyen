#include <robot_odom/robot_odom.h>

ROBOTODOM::ROBOTODOM()
: nh_priv_("~"){
    // Init robot odom node
    bool init_result = init();
    ROS_ASSERT(init_result);
}

ROBOTODOM::~ROBOTODOM(){

}

bool ROBOTODOM::init(){
    nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
    nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));
    nh_.param("imu_frame", imu_.header.frame_id, std::string("imu_link"));

    odom_pose_[X]       = 0.0;
    odom_pose_[Y]       = 0.0;
    odom_pose_[THETA]   = 0.0;

    imu_angle_[ROLL]    = 0.0;
    imu_angle_[PITCH]   = 0.0;
    imu_angle_[YAW]     = 0.0;

    acc_[X] = 0.0;
    acc_[Y] = 0.0;
    acc_[Z] = 0.0;

    gyr_[X] = 0.0;
    gyr_[Y] = 0.0;
    gyr_[Z] = 0.0;

    vel_[X] = 0.0;
    vel_[Y] = 0.0;
    vel_[THETA] = 0.0;

    double pcov_pose[36] = { 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 1e-3, 1e-9, 0.0, 0.0, 0.0,
                        0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 1e3};
    memcpy(&(odom_.pose.covariance), pcov_pose, sizeof(double)*36);

    double pcov_twist[36] = { 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 1e-3, 1e-9, 0.0, 0.0, 0.0,
                        0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1};
    memcpy(&(odom_.twist.covariance), pcov_twist, sizeof(double)*36);

    double pcov_linear[9] = {  1e-2, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0};
    memcpy(&(imu_.linear_acceleration_covariance), pcov_linear, sizeof(double)*9);

    double pcov_angular[9] = {  1e6, 0.0, 0.0,
                        0.0, 1e6, 0.0,
                        0.0, 0.0, 1e6};
    memcpy(&(imu_.angular_velocity_covariance), pcov_angular, sizeof(double)*9);

    double pcov_orientation[9] = {  1e6, 0.0, 0.0,
                        0.0, 1e6, 0.0,
                        0.0, 0.0, 0.05};
    memcpy(&(imu_.orientation_covariance), pcov_orientation, sizeof(double)*9);
    ROS_INFO("Initial data");

    // initialize publishers
    imu_pub_    = nh_.advertise<sensor_msgs::Imu>("/imu_data", 100);
    odom_pub_   = nh_.advertise<nav_msgs::Odometry>("/odom", 100);
    ROS_INFO("ROS Publisher /imu_data");
    ROS_INFO("ROS Publisher /odom");

    // initialize subscribers
    lowlevel_data_sub_ = nh_.subscribe("/robot/lowlevel/data", 100, &ROBOTODOM::commandDataCallback, this);
    ROS_INFO("ROS Subscriber /robot/lowlevel/data");

    prev_update_time_ = ros::Time::now();

    return true;
}

void ROBOTODOM::commandDataCallback(const robot_msgs::lowlevelConstPtr data_msg){
    acc_[X] = (double)data_msg->accelerometer.x;
    acc_[Y] = (double)data_msg->accelerometer.y;
    acc_[Z] = (double)data_msg->accelerometer.z;

    gyr_[X] = (double)data_msg->gyroscope.x;
    gyr_[Y] = (double)data_msg->gyroscope.y;
    gyr_[Z] = (double)data_msg->gyroscope.z;

    vel_[X] = (double)data_msg->velocity.x;
    vel_[Y] = (double)data_msg->velocity.y;
    vel_[THETA] = (double)data_msg->velocity.theta;
}

bool ROBOTODOM::updateOdometry(ros::Duration diff_time){
    double diffYawPose = vel_[THETA] * diff_time.toSec();
    double diffXPose = (vel_[X] * cos(odom_pose_[THETA]) - vel_[Y] * sin(odom_pose_[THETA])) * diff_time.toSec();
    double diffYPose = (vel_[X] * sin(odom_pose_[THETA]) + vel_[Y] * cos(odom_pose_[THETA])) * diff_time.toSec();

    odom_pose_[X] += diffXPose;
    odom_pose_[Y] += diffYPose;
    odom_pose_[THETA] += diffYawPose;

    odom_.pose.pose.position.x = odom_pose_[X];
    odom_.pose.pose.position.y = odom_pose_[Y];
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[THETA]);
    odom_.twist.twist.linear.x = vel_[X];
    odom_.twist.twist.linear.y = vel_[Y];
    odom_.twist.twist.linear.z = 0.0;
    odom_.twist.twist.angular.x = 0.0;
    odom_.twist.twist.angular.y = 0.0;
    odom_.twist.twist.angular.z = vel_[THETA];

    return true;
}

bool ROBOTODOM::updateImu(ros::Duration diff_time){
    double diffImuRoll  = gyr_[X] * diff_time.toSec() * (M_PI/180);
    double diffImuPitch = gyr_[Y] * diff_time.toSec() * (M_PI/180);
    double diffImuYaw   = gyr_[Z] * diff_time.toSec() * (M_PI/180);

    imu_angle_[ROLL]    += diffImuRoll;
    imu_angle_[PITCH]   += diffImuPitch;
    imu_angle_[YAW]     += diffImuYaw;

    imu_.linear_acceleration.x = acc_[X] * 9.80665;
    imu_.linear_acceleration.y = acc_[Y] * 9.80665;
    imu_.linear_acceleration.z = acc_[Z] * 9.80665;
    
    imu_.angular_velocity.x = gyr_[X] * (M_PI/180);
    imu_.angular_velocity.y = gyr_[Y] * (M_PI/180);
    imu_.angular_velocity.z = gyr_[Z] * (M_PI/180);

    imu_.orientation = tf::createQuaternionMsgFromRollPitchYaw(imu_angle_[ROLL], imu_angle_[PITCH], imu_angle_[YAW]);

    return true;
}

void ROBOTODOM::updateTF(geometry_msgs::TransformStamped& odom_tf){
    odom_tf.header = odom_.header;
    odom_tf.child_frame_id = odom_.child_frame_id;
    odom_tf.transform.translation.x = odom_.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_.pose.pose.position.z;
    odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

bool ROBOTODOM::update(){
    ros::Time time_now = ros::Time::now();
    ros::Duration step_time = time_now - prev_update_time_;
    prev_update_time_ = time_now;

    // odom
    updateOdometry(step_time);
    odom_.header.stamp = time_now;
    odom_pub_.publish(odom_);

    // imu
    updateImu(step_time);
    imu_.header.stamp = time_now;
    imu_pub_.publish(imu_);

    geometry_msgs::TransformStamped odom_tf;
    updateTF(odom_tf);
    // tf_broadcaster_.sendTransform(odom_tf);

    return true;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "robot_odom_node");
    ROBOTODOM robot_odom_;
    
    ros::Rate loop_rate(30);

    while(ros::ok()){
        robot_odom_.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}