#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class OdomToPoseWithCS {
public:
  OdomToPoseWithCS() {
    // Initialize subscriber and publisher
    odom_sub_ =
        nh_.subscribe("odom", 10, &OdomToPoseWithCS::odomCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "pose_with_covariance", 10);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher pose_pub_;

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    // Copy header
    pose_msg.header = msg->header;

    // Copy pose and covariance
    pose_msg.pose = msg->pose;

    // Publish the converted message
    pose_pub_.publish(pose_msg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_to_posewithcs_node");
  OdomToPoseWithCS converter;
  ros::spin();
  return 0;
}
