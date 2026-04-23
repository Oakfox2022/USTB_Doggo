#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class TrotGaitGenerator : public rclcpp::Node
{
public:
  TrotGaitGenerator() : Node("trot_gait_generator")
  {
    desired_foot_positions_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/desired_foot_positions", 10);

    phase_type_pub_ = this->create_publisher<std_msgs::msg::UInt64MultiArray>(
      "/phase_type", 10);

    forward_speed_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/forward_speed", 10,
      std::bind(&TrotGaitGenerator::forwardSpeedCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      10ms, std::bind(&TrotGaitGenerator::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Trot gait generator started | cycle=%.2fs, speed=%.2f m/s, height=%.3f m",
                cycle_time_, forward_speed_, body_height_);
  }

private:
  void timer_callback()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(8, 0.0);

    std_msgs::msg::UInt64MultiArray phase_type_;//相位类型，1为支撑相，0为摆动相
    phase_type_.data.resize(4, 1);

    // 更新相位 (0~1 循环)
    phase_ = std::fmod(phase_ + phase_increment_ * 0.01, 1.0);

    // 每个步态周期的前进位移增量
    double stance_displacement = forward_speed_ * cycle_time_ * 0.5;  // 半周期位移

    for (int leg = 0; leg < 4; ++leg)
    {
      // 对角腿相位偏移：lf/rb offset 0, rf/lb offset 0.5
      double leg_phase;
      if(leg == 0 || leg == 3)
      {
        leg_phase = std::fmod(phase_, 1.0);
      }
      else
      {
        leg_phase = std::fmod(phase_ + 0.5, 1.0);
      }

      double x, z;

      if (leg_phase < 0.5)// 摆动相 (swing phase)
      {
        phase_type_.data[leg] = 0;
        double t = leg_phase / 0.5;               // 0~1
        // x: 前移（从 -stance/2 到 +stance/2）
        x = foot_rest_x_[leg] + stance_displacement * (2.0 * t - 1.0);
        // z: 正弦抬腿
        z = foot_rest_z_[leg] + step_height_ * std::sin(M_PI * t);
      }
      else// 支撑相 (stance phase)
      {
        phase_type_.data[leg] = 1;
        double t = (leg_phase - 0.5) / 0.5;       // 0~1
        // x: 线性后移（实现身体前进）
        x = foot_rest_x_[leg] + stance_displacement * (1.0 - 2.0 * t);
        z = foot_rest_z_[leg];
      }

      // 填入 msg (x,z 交替)
      msg.data[2 * leg]     = x;
      msg.data[2 * leg + 1] = z;
    }

    phase_type_pub_->publish(phase_type_);
    desired_foot_positions_pub_->publish(msg);
  }

  void forwardSpeedCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    forward_speed_ = -msg->data[0];
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_foot_positions_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr phase_type_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr forward_speed_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  //参数
  double cycle_time_ = 0.4;// 一个完整 trot 周期 (s)
  double step_height_ = 0.08;// 抬腿高度 (m)
  double body_height_ = 0.2;// 静止时躯干高度
  double forward_speed_ = 0.0;// 前进速度 (m/s)
  double phase_ = 0.0;
  double phase_increment_ = 1.0 / cycle_time_;// 每秒相位增量

  // 每条腿的 rest 位置
  // 顺序：lf(0), lb(1), rf(2), rb(3)
  std::array<double, 4> foot_rest_x_ = { 0.0,  0.0, 0.0, 0.0 };
  std::array<double, 4> foot_rest_z_ = { -body_height_, -body_height_, -body_height_, -body_height_ };
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrotGaitGenerator>());
  rclcpp::shutdown();
  return 0;
}