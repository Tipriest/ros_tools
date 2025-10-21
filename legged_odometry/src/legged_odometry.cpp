// 引入Pinocchio库中用于解析URDF文件的头文件。
#include "pinocchio/parsers/urdf.hpp"
// 引入Pinocchio库中用于处理关节配置的头文件，例如生成随机位姿。
#include "pinocchio/algorithm/joint-configuration.hpp"
// 引入Pinocchio库中用于运动学计算的头文件，例如正向运动学。
#include <pinocchio/algorithm/frames.hxx>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
// other pkgs
#include "backward.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unordered_map>

namespace backward {

// 用于处理程序崩溃时的信号捕获
backward::SignalHandling sh;

} // namespace backward

// 定义一个类，用于实现腿式机器人的里程计计算
class LeggedOdometry {
public:
  // 构造函数，初始化ROS节点句柄、Pinocchio模型和数据
  LeggedOdometry(ros::NodeHandle &nh) : nh_(nh), model_(), data_(model_) {
    // 加载URDF模型
    std::string urdf_path =
        "/home/tipriest/Documents/legged_localization_benchmark/src/"
        "unitree_ros/robots/hexapod_description/elspider_mini_description/urdf/"
        "elsipder_mini.urdf";
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    // 初始化关节状态的订阅器
    joint_state_sub_ = nh_.subscribe("/elspider_mini/joint_states", 10,
                                     &LeggedOdometry::jointStateCallback, this);
    // 初始化每条腿的足部力传感器订阅器
    for (const auto &leg : {"LB", "LF", "LM", "RB", "RF", "RM"}) {
      foot_force_subs_.emplace_back(nh_.subscribe<geometry_msgs::WrenchStamped>(
          "/visual/" + std::string(leg) + "_foot_contact/the_force", 10,
          boost::bind(&LeggedOdometry::footForceCallback, this, _1, leg)));
    }

    // 初始化TF广播器
    odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    // 初始化机器人状态
    odom_pose_.setZero(); // 里程计位姿初始化为零
    // 初始化关节名称到索引的映射
    joint_name_to_index_["LB_HAA"] = 0;
    joint_name_to_index_["LB_HFE"] = 1;
    joint_name_to_index_["LB_KFE"] = 2;
    joint_name_to_index_["LF_HAA"] = 3;
    joint_name_to_index_["LF_HFE"] = 4;
    joint_name_to_index_["LF_KFE"] = 5;
    joint_name_to_index_["LM_HAA"] = 6;
    joint_name_to_index_["LM_HFE"] = 7;
    joint_name_to_index_["LM_KFE"] = 8;
    joint_name_to_index_["RB_HAA"] = 9;
    joint_name_to_index_["RB_HFE"] = 10;
    joint_name_to_index_["RB_KFE"] = 11;
    joint_name_to_index_["RF_HAA"] = 12;
    joint_name_to_index_["RF_HFE"] = 13;
    joint_name_to_index_["RF_KFE"] = 14;
    joint_name_to_index_["RM_HAA"] = 15;
    joint_name_to_index_["RM_HFE"] = 16;
    joint_name_to_index_["RM_KFE"] = 17;
  }

  // 关节状态回调函数
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    // 确保关节位置和速度向量大小正确
    joint_positions_ = Eigen::VectorXd::Zero(model_.nq);  // nq: 关节数量
    joint_velocities_ = Eigen::VectorXd::Zero(model_.nv); // nv: 速度数量

    // 将关节名称映射到位置和速度
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = joint_name_to_index_.find(msg->name[i]);
      if (it != joint_name_to_index_.end()) {
        joint_positions_[it->second] = msg->position[i];
        joint_velocities_[it->second] = msg->velocity[i];
      }
    }

    // 根据接触点更新里程计
    updateOdometry();
  }

  // 足部力传感器回调函数
  void footForceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg,
                         const std::string &leg) {
    // 判断足部是否接触地面
    foot_contacts_[leg] = msg->wrench.force.z < 0.05; // 接触条件
  }

  // 更新里程计
  void updateOdometry() {
    // 确保关节位置向量大小正确
    if (joint_positions_.size() != model_.nq) {
      ROS_ERROR("Joint positions size mismatch: expected %ld, got %ld",
                model_.nq, joint_positions_.size());
      return;
    }

    // 使用Pinocchio计算正向运动学
    pinocchio::forwardKinematics(model_, data_, joint_positions_);
    pinocchio::updateFramePlacements(model_, data_);

    Eigen::Vector3d delta_position = Eigen::Vector3d::Zero(); // 位移增量
    int contact_count = 0;                                    // 接触点计数

    // 遍历每条腿，计算位移增量
    for (const auto &leg : {"LB", "LF", "LM", "RB", "RF", "RM"}) {
      if (foot_contacts_[leg]) {
        auto foot_frame_id = model_.getFrameId(std::string(leg) + "_FOOT_LINK");
        Eigen::Vector3d current_foot_position =
            data_.oMf[foot_frame_id].translation();

        // 计算相对于之前位置的位移
        if (previous_foot_positions_.find(leg) !=
            previous_foot_positions_.end()) {
          delta_position +=
              previous_foot_positions_[leg] - current_foot_position;
        }

        // 更新该足部的之前位置
        previous_foot_positions_[leg] = current_foot_position;
        ++contact_count;
      }
    }

    // 如果有接触点，更新里程计位姿
    if (contact_count > 0) {
      delta_position /= contact_count;        // 平均位移增量
      odom_pose_.head<3>() += delta_position; // 更新里程计位置

      // 计算旋转增量（假设从关节状态推导机器人姿态）
      Eigen::Quaterniond delta_orientation(
          data_.oMf[0].rotation()); // 示例：从根关节获取旋转
      Eigen::Quaterniond current_orientation(odom_pose_.tail<4>()); // 当前姿态
      Eigen::Quaterniond updated_orientation =
          delta_orientation * current_orientation;
      updated_orientation.normalize(); // 确保四元数归一化
      odom_pose_.tail<4>() << updated_orientation.w(), updated_orientation.x(),
          updated_orientation.y(), updated_orientation.z(); // 更新姿态
    }

    // 发布里程计信息
    publishOdometry();
  }

  // 发布里程计信息
  void publishOdometry() {
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";

    // 设置平移部分
    odom_tf.transform.translation.x = odom_pose_(0);
    odom_tf.transform.translation.y = odom_pose_(1);
    odom_tf.transform.translation.z = odom_pose_(2);

    // 设置旋转部分
    odom_tf.transform.rotation.w = odom_pose_(3);
    odom_tf.transform.rotation.x = odom_pose_(4);
    odom_tf.transform.rotation.y = odom_pose_(5);
    odom_tf.transform.rotation.z = odom_pose_(6);

    // 发送TF变换
    odom_broadcaster_->sendTransform(odom_tf);
  }

private:
  ros::NodeHandle nh_;                           // ROS节点句柄
  ros::Subscriber joint_state_sub_;              // 关节状态订阅器
  std::vector<ros::Subscriber> foot_force_subs_; // 足部力传感器订阅器
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_; // TF广播器

  pinocchio::Model model_; // Pinocchio模型
  pinocchio::Data data_;   // Pinocchio数据

  Eigen::VectorXd joint_positions_;  // 关节位置向量
  Eigen::VectorXd joint_velocities_; // 关节速度向量
  Eigen::VectorXd odom_pose_;        // 里程计位姿

  std::unordered_map<std::string, int>
      joint_name_to_index_; // 关节名称到索引的映射
  std::unordered_map<std::string, bool> foot_contacts_; // 足部接触状态
  std::unordered_map<std::string, Eigen::Vector3d>
      previous_foot_positions_; // 足部之前的位置
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "legged_odometry");
  ros::NodeHandle nh("~");

  // 创建LeggedOdometry对象并运行ROS循环
  LeggedOdometry odometry(nh);

  ros::spin();
  return 0;
}
