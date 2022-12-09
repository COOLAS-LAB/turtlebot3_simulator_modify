#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <random>

// #define DEBUG

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> slamSyncPolicy;

class UWB
{
  private:
    void odom2rotation_and_translation_3d ( geometry_msgs::Pose pose, Eigen::Matrix3d& rotation, Eigen::Vector3d& translation ) {
      Eigen::Quaterniond t_Q;
      t_Q.x() = pose.orientation.x;
      t_Q.y() = pose.orientation.y;
      t_Q.z() = pose.orientation.z;
      t_Q.w() = pose.orientation.w;

      t_Q.normalize();
      rotation = t_Q.matrix();

      translation(0) = pose.position.x;
      translation(1) = pose.position.y;
      translation(2) = pose.position.z;
    }

    void odom2rotation_and_translation_2d ( geometry_msgs::Pose pose, Eigen::Matrix3d& rotation, Eigen::Vector3d& translation ) {
      tf::Quaternion quat;
      tf::quaternionMsgToTF(pose.orientation, quat);
      double roll, pitch, yaw;
      tf::Matrix3x3( quat ).getRPY(roll, pitch, yaw);
      geometry_msgs::Quaternion q_msg = tf::createQuaternionMsgFromYaw( yaw );
      
      Eigen::Quaterniond t_Q;
      t_Q.x() = q_msg.x;
      t_Q.y() = q_msg.y;
      t_Q.z() = q_msg.z;
      t_Q.w() = q_msg.w;

      t_Q.normalize();
      rotation = t_Q.matrix();

      translation(0) = pose.position.x;
      translation(1) = pose.position.y;
      translation(2) = 0;
    }

    double gaussian_noise( double mean, double stddev ){
      unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
      std::default_random_engine generator(seed);
      std::normal_distribution<double> dist(mean, stddev);
      return dist(generator);
    }
  public:
    UWB ( ros::NodeHandle& node ) : node_ ( node ) {
      disRA_pub_.reset(new ros::Publisher(node_.advertise<std_msgs::Float64MultiArray>("/uwb/dis_ra", 100)));
      disRB_pub_.reset(new ros::Publisher(node_.advertise<std_msgs::Float64MultiArray>("/uwb/dis_rb", 100)));

      tb3_r_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/tb3_0/odom", 100);
      tb3_a_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/tb3_1/odom", 100);
      tb3_b_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/tb3_2/odom", 100);

      sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(300), *tb3_r_odom_sub_, *tb3_a_odom_sub_, *tb3_b_odom_sub_);
      
      node_.param<double>("noise_mean", mean_, 0.0);
      node_.param<double>("noise_stddev", stddev_, 0.1);
      node_.param<double>("uwb_r1_x", uwb_r1_x_, 1.0);
      node_.param<double>("uwb_r1_y", uwb_r1_y_, 2.0);
      node_.param<double>("uwb_r2_x", uwb_r2_x_, 3.0);
      node_.param<double>("uwb_r2_y", uwb_r2_y_, 4.0);
      node_.param<double>("uwb_r3_x", uwb_r3_x_, 0.0);
      node_.param<double>("uwb_r3_y", uwb_r3_y_, 5.0);
      node_.param<double>("uwb_a0_x", uwb_a0_x_, 0.0);
      node_.param<double>("uwb_a0_y", uwb_a0_y_, 0.0);
      node_.param<double>("uwb_a1_x", uwb_a1_x_, 1.0);
      node_.param<double>("uwb_a1_y", uwb_a1_y_, 1.0);
      node_.param<double>("uwb_b0_x", uwb_b0_x_, 0.0);
      node_.param<double>("uwb_b0_y", uwb_b0_y_, 0.0);
      node_.param<double>("uwb_b1_x", uwb_b1_x_, 1.0);
      node_.param<double>("uwb_b1_y", uwb_b1_y_, 1.0);

      uwb_r1_(0) = uwb_r1_x_; uwb_r1_(1) = uwb_r1_y_; uwb_r1_(2) = 0;
      uwb_r2_(0) = uwb_r2_x_; uwb_r2_(1) = uwb_r2_y_; uwb_r2_(2) = 0;
      uwb_r3_(0) = uwb_r3_x_; uwb_r3_(1) = uwb_r3_y_; uwb_r3_(2) = 0;
      uwb_a0_(0) = uwb_a0_x_; uwb_a0_(1) = uwb_a0_y_; uwb_a0_(2) = 0;
      uwb_a1_(0) = uwb_a1_x_; uwb_a1_(1) = uwb_a1_y_; uwb_a1_(2) = 0;
      uwb_b0_(0) = uwb_b0_x_; uwb_b0_(1) = uwb_b0_y_; uwb_b0_(2) = 0;
      uwb_b1_(0) = uwb_b1_x_; uwb_b1_(1) = uwb_b1_y_; uwb_b1_(2) = 0;

      sync_->registerCallback(boost::bind(&UWB::odomCallback, this, _1, _2,  _3));
    }
    
    void odomCallback ( const nav_msgs::Odometry::ConstPtr& tb3_r_odom, const nav_msgs::Odometry::ConstPtr& tb3_a_odom, const nav_msgs::Odometry::ConstPtr& tb3_b_odom ) {
      Eigen::Matrix3d rotation_r, rotation_a, rotation_b;
      Eigen::Vector3d translation_r, translation_a, translation_b;

      odom2rotation_and_translation_2d ( tb3_r_odom->pose.pose, rotation_r, translation_r );
      odom2rotation_and_translation_2d ( tb3_a_odom->pose.pose, rotation_a, translation_a );
      odom2rotation_and_translation_2d ( tb3_b_odom->pose.pose, rotation_b, translation_b );

      Eigen::Vector3d uwb_r1_w = rotation_r * uwb_r1_ + translation_r;
      Eigen::Vector3d uwb_r2_w = rotation_r * uwb_r2_ + translation_r;
      Eigen::Vector3d uwb_r3_w = rotation_r * uwb_r3_ + translation_r;

      Eigen::Vector3d uwb_a0_w = rotation_a * uwb_a0_ + translation_a;
      Eigen::Vector3d uwb_a1_w = rotation_a * uwb_a1_ + translation_a;

      Eigen::Vector3d uwb_b0_w = rotation_b * uwb_b0_ + translation_b;
      Eigen::Vector3d uwb_b1_w = rotation_b * uwb_b1_ + translation_b;

      double disr1a0 = ( uwb_r1_w - uwb_a0_w ).norm() + gaussian_noise ( mean_, stddev_ );
      double disr2a0 = ( uwb_r1_w - uwb_a0_w ).norm() + gaussian_noise ( mean_, stddev_ );
      double disr3a0 = ( uwb_r1_w - uwb_a0_w ).norm() + gaussian_noise ( mean_, stddev_ );
      double disr1a1 = ( uwb_r1_w - uwb_a1_w ).norm() + gaussian_noise ( mean_, stddev_ );
      double disr2a1 = ( uwb_r1_w - uwb_a1_w ).norm() + gaussian_noise ( mean_, stddev_ );
      double disr3a1 = ( uwb_r1_w - uwb_a1_w ).norm() + gaussian_noise ( mean_, stddev_ );

      double disr1b0 = ( uwb_r1_w - uwb_a0_w ).norm() + gaussian_noise ( mean_, stddev_ );
      double disr2b0 = ( uwb_r1_w - uwb_a0_w ).norm() + gaussian_noise ( mean_, stddev_ );
      double disr3b0 = ( uwb_r1_w - uwb_a0_w ).norm() + gaussian_noise ( mean_, stddev_ );
      double disr1b1 = ( uwb_r1_w - uwb_a1_w ).norm() + gaussian_noise ( mean_, stddev_ );
      double disr2b1 = ( uwb_r1_w - uwb_a1_w ).norm() + gaussian_noise ( mean_, stddev_ );
      double disr3b1 = ( uwb_r1_w - uwb_a1_w ).norm() + gaussian_noise ( mean_, stddev_ );

      #ifdef DEBUG
      std::cout << "disr*a0: " << "( " << disr1a0 << ", " << disr2a0 << ", " << disr3a0 << " )" << std::endl;
      std::cout << "disr*a1: " << "( " << disr1a1 << ", " << disr2a1 << ", " << disr3a1 << " )" << std::endl;
      std::cout << "disr*b0: " << "( " << disr1b0 << ", " << disr2b0 << ", " << disr3b0 << " )" << std::endl;
      std::cout << "disr*b1: " << "( " << disr1b1 << ", " << disr2b1 << ", " << disr3b1 << " )" << std::endl;
      #endif

      disRA_.data.push_back ( disr1a0 );
      disRA_.data.push_back ( disr2a0 );
      disRA_.data.push_back ( disr3a0 );
      disRA_.data.push_back ( disr1a1 );
      disRA_.data.push_back ( disr2a1 );
      disRA_.data.push_back ( disr3a1 );

      disRB_.data.push_back ( disr1b0 );
      disRB_.data.push_back ( disr2b0 );
      disRB_.data.push_back ( disr3b0 );
      disRB_.data.push_back ( disr1b1 );
      disRB_.data.push_back ( disr2b1 );
      disRB_.data.push_back ( disr3b1 );

      disRA_pub_->publish ( disRA_ );
      disRB_pub_->publish ( disRB_ );
    }

    std::shared_ptr<ros::Publisher> disRA_pub_, disRB_pub_;
    message_filters::Subscriber<nav_msgs::Odometry> *tb3_r_odom_sub_, *tb3_a_odom_sub_, *tb3_b_odom_sub_;
    message_filters::Synchronizer<slamSyncPolicy> *sync_;
    Eigen::Vector3d uwb_r1_, uwb_r2_, uwb_r3_, uwb_a0_, uwb_a1_, uwb_b0_, uwb_b1_;
    ros::NodeHandle node_;
    std_msgs::Float64MultiArray disRA_, disRB_;


    double uwb_r1_x_, uwb_r1_y_, uwb_r2_x_, uwb_r2_y_, uwb_r3_x_, uwb_r3_y_, uwb_a0_x_, uwb_a0_y_, uwb_a1_x_, uwb_a1_y_, uwb_b0_x_, uwb_b0_y_, uwb_b1_x_, uwb_b1_y_;
    double mean_, stddev_;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "uwb_simulator");
  ros::NodeHandle node("~");
  UWB uwb( node );
  ros::spin();
  return 0;
}