#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <unordered_map>

using namespace Eigen;

class ForwardKinematics : public rclcpp::Node
{
public:
    ForwardKinematics() : Node("forward_kinematics")
    {
        //接收的关节数据
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ForwardKinematics::jointCallback, this, std::placeholders::_1));

        //收到的期望关节位置
        des_joint_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/desired_joint_positions",
            rclcpp::QoS(10),
            std::bind(&ForwardKinematics::desiredJointPosCallback, this, std::placeholders::_1)
        );
    }

private:
    Vector2d forward_kinematics(double theta1, double theta2)
    {
        //计算膝盖关节位置 两电机中间为坐标原点
        double joint_f_x = -(BASE_HALF + L_ACTIVE * std::cos(theta1));
        double joint_f_z = L_ACTIVE * std::sin(theta1);

        double joint_b_x = BASE_HALF + L_ACTIVE * std::cos(theta2);
        double joint_b_z = -(L_ACTIVE * std::sin(theta2));

        //计算两关节距离
        double dx = joint_b_x - joint_f_x;
        double dz = joint_b_z - joint_f_z;
        double d_joint = std::hypot(dx, dz);  // sqrt(dx² + dz²)

        if (d_joint > 2.0 * L_PASSIVE || d_joint < 0.09) {
            RCLCPP_WARN(this->get_logger(),
                "Error joint positions: d_joint = %f",
                d_joint);
            return Vector2d::Zero();  // 返回无效值
        }

        double d_joint_half = d_joint / 2;
        double h_squared = L_PASSIVE * L_PASSIVE - d_joint_half * d_joint_half;
        double h = std::sqrt(h_squared);

        //关节连线中点P位置
        double P_x = (joint_f_x + joint_b_x) / 2;
        double P_z = (joint_f_z + joint_b_z) / 2;

        //足端位置
        double foot_x = (P_z - joint_f_z) / d_joint_half * h + P_x;
        double foot_z = (joint_f_x - P_x) / d_joint_half * h + P_z;

        return {foot_x, foot_z};
    }

    void desiredJointPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        //判断收到的期望位置数据是否完整
        if (msg->data.size() != 8)
        {
            RCLCPP_WARN(this->get_logger(),
                "Received desired joint positions with wrong size: %zu (expected 8), ignoring",
                msg->data.size());
            return;
        }

        std::vector<double> des_joint_pos_ = msg->data;

        //计算四个足端位置
        Vector2d foot_lf_pos_ = forward_kinematics(des_joint_pos_[0], des_joint_pos_[1]);
        Vector2d foot_lb_pos_ = forward_kinematics(des_joint_pos_[2], des_joint_pos_[3]);
        Vector2d foot_rf_pos_ = forward_kinematics(-des_joint_pos_[4], -des_joint_pos_[5]);
        Vector2d foot_rb_pos_ = forward_kinematics(-des_joint_pos_[6], -des_joint_pos_[7]);

        //打印期望足端位置
        RCLCPP_INFO(this->get_logger(),
            "\n Joint_Wheel_lf desired position: [%.6f %.6f]"
            "\n Joint_Wheel_lb desired position: [%.6f %.6f]"
            "\n Joint_Wheel_rf desired position: [%.6f %.6f]"
            "\n Joint_Wheel_rb desired position: [%.6f %.6f]",
            foot_lf_pos_(0),
            foot_lf_pos_(1),
            foot_lb_pos_(0),
            foot_lb_pos_(1),
            foot_rf_pos_(0),
            foot_rf_pos_(1),
            foot_rb_pos_(0),
            foot_rb_pos_(1));
    }

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.empty()) return;

        std::vector<std::string> leg_joint_names =
        {
            "Joint_lff", "Joint_lfb",
            "Joint_lbf", "Joint_lbb",
            "Joint_rff", "Joint_rfb",
            "Joint_rbf", "Joint_rbb"
        };
        
        //创建关节信息名字和下标对应表格
        std::unordered_map<std::string, size_t> name_to_index;
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            name_to_index[msg->name[i]] = i;
        }

        //查找关节，记录关节位置和速度
        std::vector<double> joint_positions_;
        joint_positions_.resize(8, 0.0);
        std::vector<double> joint_velocity_;
        joint_velocity_.resize(8, 0.0);
        for (size_t i = 0; i < leg_joint_names.size(); ++i)
        {
            const auto& joint_name = leg_joint_names[i];

            if (name_to_index.find(joint_name) == name_to_index.end())
            {
                RCLCPP_WARN(this->get_logger(),
                    "Can't find all joints");
                continue;
            }
            
            size_t idx = name_to_index[joint_name];//关节在关节信息中对应的下标
            joint_positions_[i] = msg->position[idx];//关节位置
            joint_velocity_[i] = msg->velocity[idx];//关节速度
        }

        //计算当前足端位置
        std::vector<Vector2d> foot_pos_ = std::vector<Vector2d>(4, Vector2d::Zero());
        foot_pos_[0] = forward_kinematics(joint_positions_[0], joint_positions_[1]);
        foot_pos_[1] = forward_kinematics(joint_positions_[2], joint_positions_[3]);
        foot_pos_[2] = forward_kinematics(-joint_positions_[4], -joint_positions_[5]);
        foot_pos_[3] = forward_kinematics(-joint_positions_[6], -joint_positions_[7]);

        //打印当前足端位置
        RCLCPP_INFO(this->get_logger(),
            "\n Joint_Wheel_lf current position: [%.6f %.6f]"
            "\n Joint_Wheel_lb current position: [%.6f %.6f]"
            "\n Joint_Wheel_rf current position: [%.6f %.6f]"
            "\n Joint_Wheel_rb current position: [%.6f %.6f]",
            foot_pos_[0](0),
            foot_pos_[0](1),
            foot_pos_[1](0),
            foot_pos_[1](1),
            foot_pos_[2](0),
            foot_pos_[2](1),
            foot_pos_[3](0),
            foot_pos_[3](1));
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr des_joint_pos_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    double BASE_HALF = 0.04;      // 基座一半宽度 = 80mm/2
    double L_ACTIVE  = 0.09;      // 主动杆长度 (单位m)
    double L_PASSIVE = 0.16;     // 被动杆长度（两根相同）
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematics>());
    rclcpp::shutdown();
    return 0;
}