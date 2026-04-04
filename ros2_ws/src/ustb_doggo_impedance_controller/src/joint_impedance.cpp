#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <unordered_map>

class JointImpedance : public rclcpp::Node
{
public:
  JointImpedance() : Node("joint_impedance")
  {
    //发布的力矩
    effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/leg_effort_controller/commands", 10);
    
    //接收的关节数据
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JointImpedance::jointCallback, this, std::placeholders::_1));

    //接收的期望关节位置
    desired_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/desired_joint_positions",
      rclcpp::QoS(10),
      std::bind(&JointImpedance::desiredPosCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Joint-space Impedance started | k=%.1f Nm/rad, d=%.2f Nms/rad",
      stiffness_, damping_);
  }

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.empty()) return;

    std::vector<std::string> leg_joint_names = {
      "Joint_lff", "Joint_lfb",
      "Joint_lbf", "Joint_lbb",
      "Joint_rff", "Joint_rfb",
      "Joint_rbf", "Joint_rbb"
    };

    //创建关节信息名字和下标对应表格
    std::unordered_map<std::string, size_t> name_to_index;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      name_to_index[msg->name[i]] = i;
    }

    //力矩控制命令数据初始化
    std_msgs::msg::Float64MultiArray effort_cmd;
    effort_cmd.data.resize(8, 0.0);

    //查找关节，计算力矩
    for (size_t i = 0; i < leg_joint_names.size(); ++i) {
      const auto& joint_name = leg_joint_names[i];

      if (name_to_index.find(joint_name) == name_to_index.end()) {
        RCLCPP_WARN(this->get_logger(),
            "Can't find all joints");
        continue;
      }

      size_t idx = name_to_index[joint_name];//关节在关节信息中对应的下标
      double q   = msg->position[idx];//关节位置
      double dq  = msg->velocity[idx];//关节速度

      double pos_err = q_desired_[i] - q;
      double vel_err = 0.0 - dq;  // 期望速度为0

      double tau = stiffness_ * pos_err + damping_ * vel_err;

      effort_cmd.data[i] = tau;
    }

    //发布力矩命令
    effort_pub_->publish(effort_cmd);
  }

  void desiredPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    //判断收到的期望位置数据是否完整
    if (msg->data.size() != 8)
    {
      RCLCPP_WARN(this->get_logger(),
          "Received desired positions with wrong size: %zu (expected 8), ignoring",
          msg->data.size());
      return;
    }

    //覆盖期望位置
    q_desired_ = msg->data;
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_pos_sub_;

  //阻抗参数
  double stiffness_ = 5.0;   //刚度 Nm/rad   
  double damping_ = 0.08;      //阻尼 Nms/rad  

  //关节初始角度
  double q_init_ = 0.5;
  std::vector<double> q_desired_ = {
    -q_init_,  //Joint_lff
    q_init_,   //Joint_lfb
    -q_init_,  //Joint_lbf
    q_init_,   //Joint_lbb
    q_init_,   //Joint_rff
    -q_init_,  //Joint_rfb
    q_init_,   //Joint_rbf
    -q_init_,  //Joint_rbb
  };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointImpedance>());
  rclcpp::shutdown();
  return 0;
}