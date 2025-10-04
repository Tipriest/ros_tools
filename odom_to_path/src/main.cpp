#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

class OdomToPathConverter {
public:
  OdomToPathConverter(ros::NodeHandle &nh) {
    // 获取主题名称参数
    std::string odom_topic, path_topic, frame_id;
    nh.param<std::string>("odom_topic", odom_topic, "/odom");
    nh.param<std::string>("path_topic", path_topic, "/path");
    nh.param<std::string>("frame_id", frame_id, "odom");
    nh.param<int>("max_path_length", max_path_length_, 10000);

    // 初始化订阅者和发布者
    odom_sub_ =
        nh.subscribe(odom_topic, 10, &OdomToPathConverter::odomCallback, this);
    path_pub_ = nh.advertise<nav_msgs::Path>(path_topic, 10);

    path_.header.frame_id = frame_id; // 使用参数指定frame_id

    ROS_INFO_STREAM("[" << ros::this_node::getName()
                        << "] Subscribing to: " << odom_topic);
    ROS_INFO_STREAM("[" << ros::this_node::getName()
                        << "] Publishing to: " << path_topic);
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;

    path_.header.stamp = ros::Time::now();
    path_.poses.push_back(pose_stamped);

    // 限制path_的最大长度
    if (max_path_length_ > 0 && path_.poses.size() > max_path_length_) {
      path_.poses.erase(path_.poses.begin());
    }

    path_pub_.publish(path_);
  }

  ros::Subscriber odom_sub_;
  ros::Publisher path_pub_;
  nav_msgs::Path path_;
  int max_path_length_; // 最大路径长度
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_to_path");
  ros::NodeHandle nh("~");

  OdomToPathConverter converter(nh);

  ros::spin();
  return 0;
}
