#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class TfToPathConverter {
public:
  TfToPathConverter(ros::NodeHandle &nh) : tf_listener_(tf_buffer_) {
    // 获取参数
    std::string path_topic, frame_id, target_frame, source_frame;
    nh.param<std::string>("path_topic", path_topic, "/path");
    nh.param<std::string>("frame_id", frame_id, "odom");
    nh.param<std::string>("source_frame", source_frame, "world");
    nh.param<std::string>("target_frame", target_frame, "base_link");
    nh.param<int>("max_path_length", max_path_length_, 100000);

    // 初始化发布者
    path_pub_ = nh.advertise<nav_msgs::Path>(path_topic, 10);

    path_.header.frame_id = frame_id; // 使用参数指定frame_id
    source_frame_ = source_frame;
    target_frame_ = target_frame;

    ROS_INFO_STREAM("[" << ros::this_node::getName()
                        << "] Publishing to: " << path_topic);
    ROS_INFO_STREAM("[" << ros::this_node::getName()
                        << "] Listening for TF from " << source_frame << " to "
                        << target_frame);
  }

  void updatePath() {
    geometry_msgs::TransformStamped transform_stamped;
    try {
      // 获取TF变换
      transform_stamped = tf_buffer_.lookupTransform(
          source_frame_, target_frame_, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(1.0, "Failed to get transform: %s", ex.what());
      return;
    }

    // 转换为PoseStamped
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = transform_stamped.header;
    pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
    pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
    pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
    pose_stamped.pose.orientation = transform_stamped.transform.rotation;

    // 更新Path
    path_.header.stamp = ros::Time::now();
    path_.poses.push_back(pose_stamped);

    // 限制path_的最大长度
    if (max_path_length_ > 0 && path_.poses.size() > max_path_length_) {
      path_.poses.erase(path_.poses.begin());
    }

    // 发布Path
    path_pub_.publish(path_);
  }

private:
  ros::Publisher path_pub_;
  nav_msgs::Path path_;
  int max_path_length_; // 最大路径长度

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string target_frame_;
  std::string source_frame_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_to_path");
  ros::NodeHandle nh("~");

  TfToPathConverter converter(nh);

  ros::Rate rate(10); // 10 Hz
  while (ros::ok()) {
    converter.updatePath();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
