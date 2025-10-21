// 引入Pinocchio库中用于解析URDF文件的头文件。
#include "pinocchio/parsers/urdf.hpp"

// 引入Pinocchio库中用于处理关节配置的头文件，例如生成随机位姿。
#include "pinocchio/algorithm/joint-configuration.hpp"
// 引入Pinocchio库中用于运动学计算的头文件，例如正向运动学。
#include "pinocchio/algorithm/frames.hxx"
#include "pinocchio/algorithm/kinematics.hpp"

// 引入C++标准库中的输入输出流，用于在控制台打印信息。
#include <iostream>

int main(int argc, char **argv) {
  // 使用pinocchio命名空间，这样就不需要在每次调用Pinocchio函数时都写
  // `pinocchio::` 前缀。
  using namespace pinocchio;

  const std::string urdf_filename =
      "/home/tipriest/Documents/legged_localization_benchmark/src/unitree_ros/"
      "robots/hexapod_description/el_201_description/urdf/el_201.urdf";

  // 1. 从URDF文件加载模型
  // 创建一个空的模型对象。
  Model model;
  // 调用 `buildModel` 函数，从指定的URDF文件解析并构建机器人模型。
  // 通过urdf加载模型
  std::cout << "\033[31m"
            << "start to build urdf through model" << std::endl;
  pinocchio::urdf::buildModel(urdf_filename, model, true);
  std::cout << "\033[31m"
            << "finish build urdf through model" << std::endl;
  // 在加载模型后添加以下代码
  std::cout << "=== 关节顺序信息 ===" << std::endl;
  std::cout << "模型总关节数: " << model.njoints << std::endl;
  std::cout << "配置空间维度 (nq): " << model.nq << std::endl;
  std::cout << "速度空间维度 (nv): " << model.nv << std::endl;

  // 打印所有关节信息
  for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints;
       ++joint_id) {
    std::cout << "关节ID: " << joint_id << ", 名称: " << model.names[joint_id]
              << ", 父关节: " << model.parents[joint_id] << std::endl;
  }

  // 特别关注可配置关节
  std::cout << "\n=== 可配置关节顺序 (对应q向量的顺序) ===" << std::endl;
  int q_index = 0;
  for (JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints;
       ++joint_id) { // 从1开始，跳过根关节
    const auto &joint = model.joints[joint_id];
    if (joint.nq() > 0) { // 如果关节有配置变量
      std::cout << "q[" << q_index << "-" << (q_index + joint.nq() - 1)
                << "] -> 关节 '" << model.names[joint_id]
                << "' (ID: " << joint_id << ")" << std::endl;
      q_index += joint.nq();
    }
  }

  // 2. 创建算法所需的数据结构
  // 基于加载的模型，创建一个Data对象。这个对象将用于存储所有算法计算的中间结果，
  // 如关节的位姿、速度、雅可比矩阵等，以避免重复计算。
  Data data(model);

  // 3. 加载所有关节的角度配置
  Eigen::Matrix<double, 18, 1> q;
  q << 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  std::cout << "q: " << q.transpose() << std::endl;

  // 打印每个关节在配置向量 q 中的起始索引和自由度数量
  std::cout << "打印每个关节在配置向量 q 中的起始索引和自由度数量: "
            << std::endl;
  for (JointIndex joint_id = 1; joint_id < model.njoints; ++joint_id) {
    std::cout << "Joint " << model.names[joint_id] << " 在向量q中的起始索引 "
              << model.idx_qs[joint_id] << " and has "
              << model.joints[joint_id].nq() << " DoF(s)."
              << "当前关节角度: "
              << q.segment(model.idx_qs[joint_id], model.joints[joint_id].nq())
                     .transpose()
              << std::endl;
  }

  // 4. 执行正向运动学计算
  // `forwardKinematics` 函数会根据给定的关节配置
  // `q`，计算机器人模型中每个关节相对于世界坐标系的位姿（位置和姿态）。
  // 计算结果会更新到 `data` 对象中。
  forwardKinematics(model, data, q);

  // 5. 打印每个joint相对于universe的旋转矩阵和平移矩阵
  std::cout << "打印每个joint相对于universe的旋转矩阵和平移矩阵" << std::endl;
  for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints;
       ++joint_id)
    // 打印关节的名称和其在世界坐标系下的位置（平移向量）
    // `data.oMi[joint_id]` 存储了第 `joint_id`
    // 个关节坐标系相对于世界坐标系（origin）的变换矩阵（SE3类型）。
    // `.translation()` 获取该变换的平移部分。
    std::cout << std::setw(24) << std::left << model.names[joint_id] << ": "
              << std::fixed << std::setprecision(2)
              << data.oMi[joint_id].translation().transpose() << std::endl
              << data.oMi[joint_id].rotation() << std::endl;

  // 6. 打印每个连杆的位置
  updateFramePlacements(model, data);
  std::cout << "\n=== Link Positions ===" << std::endl;
  for (FrameIndex frame_id = 0; frame_id < model.nframes; ++frame_id) {
    if (model.frames[frame_id].type == BODY) { // 只打印BODY类型的frame
      std::cout << std::setw(24) << std::left
                << model.frames[frame_id].name << ": " << std::fixed
                << std::setprecision(2)
                << data.oMf[frame_id].translation().transpose()<<std::endl
                << data.oMf[frame_id].rotation() << std::endl;
    }
  }

  // 每个关节的名字
  // 它在 q 向量里对应的索引范围
  // 当前 q 中的数值
  for (JointIndex joint_id = 1; joint_id < model.njoints; ++joint_id) {
    int idx_q = model.joints[joint_id].idx_q();
    int nq_joint = model.joints[joint_id].nq();
    std::cout << std::setw(24) << std::left << model.names[joint_id]
              << "  DOF: " << nq_joint << "  q index: " << idx_q;
    if (nq_joint == 1) {
      std::cout << "  value in q: " << q[idx_q];
    } else {
      std::cout << "  values: " << q.segment(idx_q, nq_joint).transpose();
    }
    std::cout << std::endl;
  }
}