#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class OdomTfBroadcaster {
public:
  OdomTfBroadcaster(const std::string &odom_topic) {
    ros::NodeHandle nh;
    sub_odom_ =
        nh.subscribe(odom_topic, 50, &OdomTfBroadcaster::odomCallback, this);
    ROS_INFO_STREAM("Odom TF Broadcaster subscribed to topic: " << odom_topic);
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z));

    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    transform.setRotation(q);

    br.sendTransform(
        tf::StampedTransform(transform, msg->header.stamp,
                             msg->header.frame_id, // 通常是 "odom"
                             msg->child_frame_id   // 通常是 "base_link"
                             ));
  }

  ros::Subscriber sub_odom_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_tf_broadcaster");

  ros::NodeHandle private_nh("~");

  std::string odom_topic;
  private_nh.param<std::string>("odom_topic", odom_topic, std::string("/odom"));

  OdomTfBroadcaster broadcaster(odom_topic);

  ros::spin();
  return 0;
}