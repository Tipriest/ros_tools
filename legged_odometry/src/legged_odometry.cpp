// Pinocchio
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/algorithm/frames.hxx>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

// ROS
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>

// Other
#include "backward.hpp"
#include <Eigen/Dense>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace backward {

// 用于处理程序崩溃时的信号捕获
backward::SignalHandling sh;

} // namespace backward

// 定义一个类，用于实现腿式机器人的里程计计算（按 README 的状态机逻辑）
class LeggedOdometry {
public:
  enum class RobotState {
    BEFORE_INIT,
    FREE_STAND,
    SIX_GAIT,
    THREE_GAIT,
    TWO_GAIT,
    UNKNOWN_GAIT
  };

  // 构造函数，初始化ROS节点句柄、Pinocchio模型和数据
  explicit LeggedOdometry(ros::NodeHandle &nh)
      : nh_(nh), pnh_("~"), model_(), data_(model_) {
    // 参数
    pnh_.param<std::string>("base_frame", base_frame_,
                            std::string("base_link"));
    pnh_.param<std::string>("odom_frame", odom_frame_, std::string("world"));
    pnh_.param<std::string>("joint_state_topic", joint_state_topic_,
                            std::string("/elspider_mini/joint_states"));
    pnh_.param<bool>("use_imu", use_imu_, false);
    pnh_.param<std::string>("imu_topic", imu_topic_, std::string("/imu/data"));
    pnh_.param<double>("contact_force_threshold", contact_force_threshold_,
                       0.05);
    pnh_.param<std::string>(
        "urdf_path", urdf_path_,
        std::string("/home/tipriest/Documents/legged_localization_benchmark/"
                    "src/unitree_ros/robots/hexapod_description/"
                    "elspider_mini_description/urdf/elsipder_mini.urdf"));

    // 默认腿顺序和足端frame名称（可通过参数 foot_frames 覆盖）
    leg_names_ = {"LB", "LF", "LM", "RB", "RF", "RM"};
    std::vector<std::string> default_foot_frames = {
        "LB_FOOT_LINK", "LF_FOOT_LINK", "LM_FOOT_LINK",
        "RB_FOOT_LINK", "RF_FOOT_LINK", "RM_FOOT_LINK"};
    if (!pnh_.getParam("foot_frames", foot_frames_)) {
      foot_frames_ = default_foot_frames;
    }
    if (foot_frames_.size() != 6) {
      ROS_WARN("foot_frames param size (%zu) != 6, using defaults.",
               foot_frames_.size());
      foot_frames_ = default_foot_frames;
    }

    // 加载URDF模型
    try {
      pinocchio::urdf::buildModel(urdf_path_, model_);
    } catch (const std::exception &e) {
      ROS_FATAL("Failed to load URDF model from '%s': %s", urdf_path_.c_str(),
                e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    // 解析 frame id
    try {
      base_frame_id_ = model_.getFrameId(base_frame_);
    } catch (...) {
      ROS_FATAL("Base frame '%s' not found in model.", base_frame_.c_str());
      throw;
    }
    for (size_t i = 0; i < 6; ++i) {
      int fid = -1;
      try {
        fid = model_.getFrameId(foot_frames_[i]);
      } catch (...) {
        ROS_FATAL("Foot frame '%s' not found in model.",
                  foot_frames_[i].c_str());
        throw;
      }
      foot_frame_ids_.push_back(fid);
      leg_id_of_name_[leg_names_[i]] = static_cast<int>(i);
    }

    // 订阅器
    joint_state_sub_ = nh_.subscribe(joint_state_topic_, 50,
                                     &LeggedOdometry::jointStateCallback, this);
    if (use_imu_) {
      imu_sub_ =
          nh_.subscribe(imu_topic_, 50, &LeggedOdometry::imuCallback, this);
    }
    for (const auto &leg : leg_names_) {
      std::string topic =
          "/visual/" + leg + std::string("_foot_contact/the_force");
      foot_force_subs_.emplace_back(nh_.subscribe<geometry_msgs::WrenchStamped>(
          topic, 10,
          boost::bind(&LeggedOdometry::footForceCallback, this, _1, leg)));
      foot_contacts_[leg] = false;
    }

    // 发布器与TF
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    // 初始化状态
    robot_state_ = RobotState::BEFORE_INIT;
    PHASE_START_GLOBAL_T_wb_ = pinocchio::SE3::Identity();
    PHASE_CUR_GLOBAL_T_wb_ = pinocchio::SE3::Identity();
    imu_orientation_.w = 1.0;
    imu_orientation_.x = 0.0;
    imu_orientation_.y = 0.0;
    imu_orientation_.z = 0.0;
  }

  // 关节状态回调函数：更新 q 并推进状态机
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    // 填充 q（按 Pinocchio 的顺序），这里假设模型为固定基座、nq=18；
    // 如为浮动基座，请在 URDF 内固定或扩展此处处理。
    joint_positions_ = Eigen::VectorXd::Zero(model_.nq);
    // 通过 joint name -> joint id -> idx_qs 映射，稳健写入 q
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string &nm = msg->name[i];
      if (!std::isfinite(msg->position[i]))
        continue;
      try {
        pinocchio::JointIndex jid = model_.getJointId(nm);
        if (jid >= model_.joints.size())
          continue;
        const auto &j = model_.joints[jid];
        if (j.nq() == 1) {
          joint_positions_[model_.idx_qs[jid]] = msg->position[i];
        } else if (j.nq() > 1) {
          // 若多自由度关节，需联合处理；这里简化：只在 nq==1 情形下写入
        }
      } catch (...) { /* 忽略不在模型中的关节 */
      }
    }
    processOnce();
  }

  // IMU 回调（可选）
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu_orientation_ = msg->orientation;
  }

  // 足部力传感器回调函数：以竖直力阈值判断是否着地
  void footForceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg,
                         const std::string &leg) {
    const double f = std::fabs(msg->wrench.force.x) +
                     std::fabs(msg->wrench.force.y) +
                     std::fabs(msg->wrench.force.z);
    foot_contacts_[leg] = (f >= contact_force_threshold_);
  }

  // 主处理逻辑：实现 README 描述的状态机
  void processOnce() {
    if (joint_positions_.size() != model_.nq) {
      ROS_ERROR_THROTTLE(1.0,
                         "Joint positions size mismatch: expected %d, got %d",
                         (int)model_.nq, (int)joint_positions_.size());
      return;
    }

    // 前向运动学并更新 frame 位姿
    pinocchio::forwardKinematics(model_, data_, joint_positions_);
    pinocchio::updateFramePlacements(model_, data_);

    // 统计当前着地腿集合
    CUR_STANCE_LEGS_ID_SET_.clear();
    CUR_STANCE_LEGS_ID_NUM_ = 0;
    for (size_t i = 0; i < leg_names_.size(); ++i) {
      const std::string &leg = leg_names_[i];
      if (foot_contacts_[leg]) {
        CUR_STANCE_LEGS_ID_SET_.insert(static_cast<int>(i));
        CUR_STANCE_LEGS_ID_NUM_++;
      }
    }

    // BEFORE_INIT -> 初始化
    if (robot_state_ == RobotState::BEFORE_INIT) {
      if (CUR_STANCE_LEGS_ID_NUM_ < 3) {
        robot_state_ = RobotState::UNKNOWN_GAIT;
        ROS_WARN_THROTTLE(1.0,
                          "Less than 3 stance legs, waiting to initialize...");
        return;
      }
      // 设置状态
      setStateByStanceCount(CUR_STANCE_LEGS_ID_NUM_);
      PHASE_START_STANCE_LEGS_ID_SET_ = CUR_STANCE_LEGS_ID_SET_;
      PHASE_START_STANCE_LEGS_ID_NUM_ = CUR_STANCE_LEGS_ID_NUM_;

      // 记录局部 T_bf0
      PHASE_START_LOCAL_T_bf_.clear();
      for (int leg_id : PHASE_START_STANCE_LEGS_ID_SET_) {
        pinocchio::SE3 T_bf = calcBaseToFoot(static_cast<size_t>(leg_id));
        PHASE_START_LOCAL_T_bf_[leg_id] = T_bf;
      }
      // 世界原点作为初始位姿
      PHASE_START_GLOBAL_T_wb_ = pinocchio::SE3::Identity();
      PHASE_CUR_GLOBAL_T_wb_ = PHASE_START_GLOBAL_T_wb_;
      publishOdometry();
      return;
    }

    // 已经初始化：判断是否处于同一相位
    if (CUR_STANCE_LEGS_ID_SET_ == PHASE_START_STANCE_LEGS_ID_SET_) {
      // 同一相位：根据无滑移约束更新 T_wb
      std::vector<pinocchio::SE3> candidates;
      candidates.reserve(CUR_STANCE_LEGS_ID_SET_.size());
      for (int leg_id : CUR_STANCE_LEGS_ID_SET_) {
        const pinocchio::SE3 &T_bf0 = PHASE_START_LOCAL_T_bf_.at(leg_id);
        pinocchio::SE3 T_bf = calcBaseToFoot(static_cast<size_t>(leg_id));
        // T_wb = (T_wb0 * T_bf0) * inv(T_bf)
        pinocchio::SE3 T = PHASE_START_GLOBAL_T_wb_ * T_bf0 * T_bf.inverse();
        candidates.push_back(T);
      }
      PHASE_CUR_GLOBAL_T_wb_ = averageSE3(candidates);

      // IMU 融合（可选：使用 IMU 的俯仰横滚，保留 yaw）
      if (use_imu_) {
        PHASE_CUR_GLOBAL_T_wb_.rotation() = fuseImuRollPitch(
            PHASE_CUR_GLOBAL_T_wb_.rotation(), imu_orientation_);
      }

      publishOdometry();
    } else {
      // 新相位：
      PHASE_START_GLOBAL_T_wb_ = PHASE_CUR_GLOBAL_T_wb_;
      PHASE_START_STANCE_LEGS_ID_SET_ = CUR_STANCE_LEGS_ID_SET_;
      PHASE_START_STANCE_LEGS_ID_NUM_ = CUR_STANCE_LEGS_ID_NUM_;
      setStateByStanceCount(PHASE_START_STANCE_LEGS_ID_NUM_);

      if (PHASE_START_STANCE_LEGS_ID_NUM_ < 3) {
        robot_state_ = RobotState::BEFORE_INIT;
        ROS_WARN_THROTTLE(
            1.0, "Phase changed to <3 stance legs, reinitialize when stable.");
        return;
      }

      PHASE_START_LOCAL_T_bf_.clear();
      for (int leg_id : PHASE_START_STANCE_LEGS_ID_SET_) {
        pinocchio::SE3 T_bf = calcBaseToFoot(static_cast<size_t>(leg_id));
        PHASE_START_LOCAL_T_bf_[leg_id] = T_bf;
      }
      // 当前位姿从上一相位延续
      PHASE_CUR_GLOBAL_T_wb_ = PHASE_START_GLOBAL_T_wb_;
      publishOdometry();
    }
  }

  // 发布里程计 TF 与 Odometry
  void publishOdometry() {
    const Eigen::Vector3d p = PHASE_CUR_GLOBAL_T_wb_.translation();
    const Eigen::Matrix3d R = PHASE_CUR_GLOBAL_T_wb_.rotation();
    Eigen::Quaterniond q(R);
    q.normalize();

    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = odom_frame_;
    odom_tf.child_frame_id = base_frame_;
    odom_tf.transform.translation.x = p.x();
    odom_tf.transform.translation.y = p.y();
    odom_tf.transform.translation.z = p.z();
    odom_tf.transform.rotation.w = q.w();
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_broadcaster_->sendTransform(odom_tf);

    nav_msgs::Odometry odom;
    odom.header = odom_tf.header;
    odom.child_frame_id = odom_tf.child_frame_id;
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();
    odom.pose.pose.orientation = odom_tf.transform.rotation;
    odom_pub_.publish(odom);
    // std::cout << "cur p = " << p.transpose() << std::endl
    //           << "cur q = " << q.w() << " " << q.x() << " " << q.y() << " "
    //           << q.z() << " " << std::endl;
  }

private:
  // 计算 base->foot 的局部齐次变换
  pinocchio::SE3 calcBaseToFoot(size_t leg_index) const {
    const pinocchio::SE3 &T_wb = data_.oMf[base_frame_id_];
    const pinocchio::SE3 &T_wf = data_.oMf[foot_frame_ids_.at(leg_index)];
    return T_wb.inverse() * T_wf;
  }

  // 根据着地腿数量设置状态
  void setStateByStanceCount(int n) {
    if (n >= 6)
      robot_state_ = RobotState::FREE_STAND;
    else if (n == 5)
      robot_state_ = RobotState::SIX_GAIT;
    else if (n == 4)
      robot_state_ = RobotState::THREE_GAIT;
    else if (n == 3)
      robot_state_ = RobotState::TWO_GAIT;
    else
      robot_state_ = RobotState::UNKNOWN_GAIT;
  }

  // 平均多个 SE3（简化：平移取均值，旋转取四元数平均）
  static pinocchio::SE3 averageSE3(const std::vector<pinocchio::SE3> &Ts) {
    if (Ts.empty())
      return pinocchio::SE3::Identity();
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_avg(0, 0, 0, 0);
    for (const auto &T : Ts) {
      p += T.translation();
      Eigen::Quaterniond q(T.rotation());
      if (q_avg.w() == 0 && q_avg.vec().norm() == 0)
        q_avg = q; // 初始化
      else {
        if (q_avg.dot(q) < 0.0)
          q.coeffs() *= -1.0; // 避免双覆盖
        q_avg.coeffs() += q.coeffs();
      }
    }
    p /= static_cast<double>(Ts.size());
    q_avg.normalize();
    return pinocchio::SE3(q_avg.toRotationMatrix(), p);
  }

  // 使用 IMU 的 roll/pitch 融合当前旋转（保留 yaw）
  static Eigen::Matrix3d
  fuseImuRollPitch(const Eigen::Matrix3d &R_in,
                   const geometry_msgs::Quaternion &imu_q) {
    // 从 R_in 提取 yaw
    double sy = std::sqrt(R_in(0, 0) * R_in(0, 0) + R_in(1, 0) * R_in(1, 0));
    double yaw = std::atan2(R_in(1, 0), R_in(0, 0));

    // IMU 四元数转 roll/pitch
    Eigen::Quaterniond q_imu(imu_q.w, imu_q.x, imu_q.y, imu_q.z);
    Eigen::Matrix3d R_imu = q_imu.toRotationMatrix();
    double roll = std::atan2(R_imu(2, 1), R_imu(2, 2));
    double pitch = std::asin(-R_imu(2, 0));

    // 重建融合旋转 R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
    return (Rz * Ry * Rx).toRotationMatrix();
  }

  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber imu_sub_;
  std::vector<ros::Subscriber> foot_force_subs_;
  ros::Publisher odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  // 参数
  std::string base_frame_;
  std::string odom_frame_;
  std::string joint_state_topic_;
  std::string imu_topic_;
  bool use_imu_{false};
  double contact_force_threshold_{0.05};
  std::string urdf_path_;

  // 模型
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd joint_positions_;

  // 帧与腿信息
  int base_frame_id_{-1};
  std::vector<int> foot_frame_ids_;
  std::vector<std::string> leg_names_;
  std::vector<std::string> foot_frames_;
  std::unordered_map<std::string, int> leg_id_of_name_;

  // 接触与传感器
  std::unordered_map<std::string, bool> foot_contacts_;
  geometry_msgs::Quaternion imu_orientation_;

  // 状态机变量
  RobotState robot_state_{RobotState::BEFORE_INIT};
  std::set<int> PHASE_START_STANCE_LEGS_ID_SET_;
  int PHASE_START_STANCE_LEGS_ID_NUM_{0};
  std::set<int> CUR_STANCE_LEGS_ID_SET_;
  int CUR_STANCE_LEGS_ID_NUM_{0};
  std::unordered_map<int, pinocchio::SE3> PHASE_START_LOCAL_T_bf_;
  pinocchio::SE3 PHASE_START_GLOBAL_T_wb_{pinocchio::SE3::Identity()};
  pinocchio::SE3 PHASE_CUR_GLOBAL_T_wb_{pinocchio::SE3::Identity()};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "legged_odometry");
  ros::NodeHandle nh;

  // 创建LeggedOdometry对象并运行ROS循环
  LeggedOdometry odometry(nh);

  ros::spin();
  return 0;
}
