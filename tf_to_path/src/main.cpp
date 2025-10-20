#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// 该类负责从TF变换中获取位姿信息，并将其转换为Path消息发布
class TfToPathConverter {
public:
  // 构造函数，初始化TF监听器和ROS发布者
  TfToPathConverter(ros::NodeHandle &nh) : tf_listener_(tf_buffer_) {
    // 从参数服务器获取配置参数
    std::string path_topic, frame_id, target_frame, source_frame;
    nh.param<std::string>("path_topic", path_topic, "/path"); // Path发布话题
    nh.param<std::string>("frame_id", frame_id, "odom");      // Path的frame_id
    nh.param<std::string>("source_frame", source_frame, "world"); // TF源坐标系
    nh.param<std::string>("target_frame", target_frame,
                          "base_link"); // TF目标坐标系
    nh.param<int>("max_path_length", max_path_length_,
                  100000); // Path的最大长度

    // 初始化Path发布者
    path_pub_ = nh.advertise<nav_msgs::Path>(path_topic, 10);

    path_.header.frame_id = frame_id; // 使用参数指定frame_id
    source_frame_ = source_frame;
    target_frame_ = target_frame;

    // 打印配置信息
    ROS_INFO_STREAM("[" << ros::this_node::getName()
                        << "] Publishing to: " << path_topic);
    ROS_INFO_STREAM("[" << ros::this_node::getName()
                        << "] Listening for TF from " << source_frame << " to "
                        << target_frame);
  }

  // 更新Path的方法，从TF中获取位姿并添加到Path中
  void updatePath() {
    geometry_msgs::TransformStamped transform_stamped;
    try {
      // 尝试从TF缓冲区中获取变换
      transform_stamped = tf_buffer_.lookupTransform(
          source_frame_, target_frame_, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      // 如果获取失败，打印警告信息
      ROS_WARN_THROTTLE(1.0, "Failed to get transform: %s", ex.what());
      return;
    }

    // 将TF变换转换为PoseStamped
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = transform_stamped.header;
    pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
    pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
    pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
    pose_stamped.pose.orientation = transform_stamped.transform.rotation;

    // 更新Path的时间戳并添加新的位姿
    path_.header.stamp = ros::Time::now();
    path_.poses.push_back(pose_stamped);

    // 如果Path长度超过最大值，移除最早的位姿
    if (max_path_length_ > 0 && path_.poses.size() > max_path_length_) {
      path_.poses.erase(path_.poses.begin());
    }

    // 发布更新后的Path
    path_pub_.publish(path_);
  }

private:
  ros::Publisher path_pub_; // Path消息发布者
  nav_msgs::Path path_;     // 存储Path的对象
  int max_path_length_;     // Path的最大长度

  tf2_ros::Buffer tf_buffer_;              // TF缓冲区
  tf2_ros::TransformListener tf_listener_; // TF监听器
  std::string target_frame_;               // TF目标坐标系
  std::string source_frame_;               // TF源坐标系
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_to_path"); // 初始化ROS节点
  ros::NodeHandle nh("~");             // 创建私有节点句柄

  TfToPathConverter converter(nh); // 创建TfToPathConverter对象

  ros::Rate rate(10); // 设置循环频率为10Hz
  while (ros::ok()) {
    converter.updatePath(); // 定期更新Path
    ros::spinOnce();        // 处理回调
    rate.sleep();           // 按照频率休眠
  }

  return 0;
}
