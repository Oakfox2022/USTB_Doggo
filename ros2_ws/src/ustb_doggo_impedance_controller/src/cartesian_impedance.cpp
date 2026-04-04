#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <unordered_map>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace Eigen;

class CartesianImpedance : public rclcpp::Node
{
public:
    CartesianImpedance() : Node("cartesian_impedance")
    {
        // 获取 URDF 路径
        std::string pkg_share = ament_index_cpp::get_package_share_directory("ustb_doggo_urdf");
        std::string urdf_path = pkg_share + "/urdf/ustb_doggo_urdf.urdf";

        // 加载 URDF 到 Pinocchio 模型
        pinocchio::urdf::buildModel(
            urdf_path,
            pinocchio::JointModelFreeFlyer(),
            pin_model_
        );
        pin_data_ = pinocchio::Data(pin_model_);

        RCLCPP_INFO(this->get_logger(), "Pinocchio model loaded, nq=%d, nv=%d",
                pin_model_.nq, pin_model_.nv);

        RCLCPP_INFO(this->get_logger(), "Pinocchio joints:");
        for (size_t i = 0; i < pin_model_.names.size(); ++i) 
        {
            RCLCPP_INFO(this->get_logger(), "[%zu] %s", i, pin_model_.names[i].c_str());
            joint_name_to_pin_idx_[pin_model_.names[i]] = i;
        }

        q = VectorXd::Zero(pin_model_.nq);

        //接收的baselink位姿
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&CartesianImpedance::odomCallback, this, std::placeholders::_1));

        //接收的阻抗参数
        para_k_d_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/para_k_d_", 10,
            std::bind(&CartesianImpedance::paraKDCallback, this, std::placeholders::_1));

        //发布的力矩
        effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_effort_controller/commands", 10);

        //发布的当前足端位置
        current_foot_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/current_foot_position", 10);

        //接收的关节数据
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&CartesianImpedance::jointCallback, this, std::placeholders::_1));
        
        //接收的期望足端位置
        desired_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/desired_foot_positions",
            rclcpp::QoS(10),
            std::bind(&CartesianImpedance::desiredPosCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Cartesian Impedance started! Parameters: K = [%f %f], D = [%f %f]", K_(0,0), K_(1,1), D_(0,0), D_(1,1));
    }

private:
    //正运动学
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

        if (d_joint > 2.0 * L_PASSIVE || d_joint < 0.01)
        {
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
    
    //解析雅可比 关节速度到足端速度的映射
    Matrix2d analytical_jacobian(double theta1, double theta2, double x, double z)
    {
        //计算膝盖关节位置 两电机中间为坐标原点
        double joint_f_x = -(BASE_HALF + L_ACTIVE * std::cos(theta1));
        double joint_f_z = L_ACTIVE * std::sin(theta1);

        double joint_b_x = BASE_HALF + L_ACTIVE * std::cos(theta2);
        double joint_b_z = -(L_ACTIVE * std::sin(theta2));

        // 计算 A 矩阵（2×2）
        Matrix2d A;
        A(0,0) = 2 * (x - joint_f_x);
        A(0,1) = 2 * (z - joint_f_z);
        A(1,0) = 2 * (x - joint_b_x);
        A(1,1) = 2 * (z - joint_b_z);

        // 计算 B 矩阵（2×2）
        Matrix2d B;
        B(0,0) = -2 * L_ACTIVE * (std::sin(theta1) * (x - joint_f_x) + std::cos(theta1) * (z - joint_f_z));
        B(0,1) = 0;
        B(1,0) = 0;
        B(1,1) = 2 * L_ACTIVE * (std::sin(theta2) * (x - joint_b_x) + std::cos(theta2) * (z - joint_b_z));

        // -A [dx dz]^T = B [dtheta1 dtheta2]^T
        // J = - A^{-1} B
        Matrix2d J = -A.inverse() * B;

        return J;
    }

    //odem回调
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(pin_model_.nq == 31)
        {
            // 填 q 前 7 个
            q(0) = msg->pose.pose.position.x;
            q(1) = msg->pose.pose.position.y;
            q(2) = msg->pose.pose.position.z;
            q(3) = msg->pose.pose.orientation.w;
            q(4) = msg->pose.pose.orientation.x;
            q(5) = msg->pose.pose.orientation.y;
            q(6) = msg->pose.pose.orientation.z;
        }
    }

    //pinocchio重力补偿计算函数
    VectorXd compute_gravity_compensation(const std::vector<double>& joint_positions)
    {
        for(size_t i = 0; i < leg_joint_names.size(); ++i)
        {
            //传入关节位置，q中索引对应位置向量q
            q(pin_model_.idx_qs[joint_name_to_pin_idx_[leg_joint_names[i]]]) = joint_positions[i];
            //RCLCPP_INFO(this->get_logger(), "%s %d", leg_joint_names[i].c_str(), pin_model_.idx_qs[joint_name_to_pin_idx_[leg_joint_names[i]]]);
        }
        
        // 计算重力项 G(q)（q̇=0, q̈=0）
        VectorXd tau_gravity = pinocchio::rnea(
            pin_model_, pin_data_,
            q,
            VectorXd::Zero(pin_model_.nv),  // q̇ = 0
            VectorXd::Zero(pin_model_.nv)   // q̈ = 0
        );

        return tau_gravity;
    }
    
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.empty()) return;
        
        //创建关节信息名字和下标对应表格
        std::unordered_map<std::string, size_t> name_to_index;
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            name_to_index[msg->name[i]] = i;
        }

        //查找关节，记录关节位置和速度
        std::vector<double> joint_positions_;
        joint_positions_.resize(leg_joint_names.size(), 0.0);
        std::vector<double> joint_velocity_;
        joint_velocity_.resize(leg_joint_names.size(), 0.0);

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

        //发布足端位置（键盘控制节点接收）
        std_msgs::msg::Float64MultiArray foot_position;
        foot_position.data.resize(8, 0.0);
        foot_position.data[0] = foot_pos_[0](0);
        foot_position.data[1] = foot_pos_[0](1);
        foot_position.data[2] = foot_pos_[1](0);
        foot_position.data[3] = foot_pos_[1](1);
        foot_position.data[4] = foot_pos_[2](0);
        foot_position.data[5] = foot_pos_[2](1);
        foot_position.data[6] = foot_pos_[3](0);
        foot_position.data[7] = foot_pos_[3](1);
        current_foot_pos_pub_->publish(foot_position);

        //计算雅可比矩阵
        std::vector<Matrix2d> J = std::vector<Matrix2d>(4, Matrix2d::Zero());
        J[0] = analytical_jacobian(joint_positions_[0], joint_positions_[1], foot_pos_[0](0), foot_pos_[0](1));
        J[1] = analytical_jacobian(joint_positions_[2], joint_positions_[3], foot_pos_[1](0), foot_pos_[1](1));
        J[2] = analytical_jacobian(-joint_positions_[4], -joint_positions_[5], foot_pos_[2](0), foot_pos_[2](1));
        J[3] = analytical_jacobian(-joint_positions_[6], -joint_positions_[7], foot_pos_[3](0), foot_pos_[3](1));
        
        //计算当前足端速度
        std::vector<Vector2d> foot_vel_ = std::vector<Vector2d>(4, Vector2d::Zero());
        foot_vel_[0] = J[0] * Vector2d(joint_velocity_[0], joint_velocity_[1]);
        foot_vel_[1] = J[1] * Vector2d(joint_velocity_[2], joint_velocity_[3]);
        foot_vel_[2] = J[2] * Vector2d(-joint_velocity_[4], -joint_velocity_[5]);
        foot_vel_[3] = J[3] * Vector2d(-joint_velocity_[6], -joint_velocity_[7]);
        
        //计算重力补偿，tau_gravity中索引对应速度向量v
        VectorXd tau_gravity = compute_gravity_compensation(joint_positions_);
        gravity_comp_[0](0) = tau_gravity(pin_model_.idx_vs[joint_name_to_pin_idx_[leg_joint_names[0]]]);
        gravity_comp_[0](1) = tau_gravity(pin_model_.idx_vs[joint_name_to_pin_idx_[leg_joint_names[1]]]);
        gravity_comp_[1](0) = tau_gravity(pin_model_.idx_vs[joint_name_to_pin_idx_[leg_joint_names[2]]]);
        gravity_comp_[1](1) = tau_gravity(pin_model_.idx_vs[joint_name_to_pin_idx_[leg_joint_names[3]]]);
        gravity_comp_[2](0) = tau_gravity(pin_model_.idx_vs[joint_name_to_pin_idx_[leg_joint_names[4]]]);
        gravity_comp_[2](1) = tau_gravity(pin_model_.idx_vs[joint_name_to_pin_idx_[leg_joint_names[5]]]);
        gravity_comp_[3](0) = tau_gravity(pin_model_.idx_vs[joint_name_to_pin_idx_[leg_joint_names[6]]]);
        gravity_comp_[3](1) = tau_gravity(pin_model_.idx_vs[joint_name_to_pin_idx_[leg_joint_names[7]]]);
        
        //力矩控制命令数据初始化
        std_msgs::msg::Float64MultiArray effort_cmd;
        effort_cmd.data.resize(8, 0.0);
        size_t k = 0;
        Vector2d delta_pos_;
        Vector2d delta_vel_;
        Vector2d F_virt;
        Vector2d tau;
        Vector2d foot_force_world(0, -Fz_per_leg);
        for(size_t i = 0; i < 4; ++i)
        {
            //计算位置误差和速度误差
            delta_pos_ = foot_pos_desired_[i] - foot_pos_[i];
            delta_vel_ = foot_vel_desired_[i] - foot_vel_[i];

            //计算笛卡尔空间虚拟力
            F_virt = K_ * delta_pos_ + D_ * delta_vel_;

            //加足端接触力补偿（与pinocchio重力补偿二选一）
            F_virt += foot_force_world;

            //通过雅可比转置映射到关节力矩
            if(i < 2)
            {
                tau = J[i].transpose() * F_virt;    //左腿
            }
            else
            {
                tau = -J[i].transpose() * F_virt;   //右腿
            }

            //加重力补偿（与足端接触力补偿二选一）
            //tau += gravity_comp_[i];

            for(size_t j = 0; j < 2; ++j)
            {
                effort_cmd.data[k] = tau(j);
                k++;
            }
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
        size_t k = 0;
        for(size_t i = 0; i < 4; ++i)
        {
            for(size_t j = 0; j < 2; ++j)
            {
                foot_pos_desired_[i](j) = msg->data[k];
                k++;
            }
        }
    }

    void paraKDCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        //判断收到的期望位置数据是否完整
        if (msg->data.size() != 4)
        {
            RCLCPP_WARN(this->get_logger(),
                "Received parameters with wrong size: %zu (expected 4), ignoring",
                msg->data.size());
            return;
        }

        //覆盖阻抗参数
        K_ = Vector2d(msg->data[0], msg->data[1]).asDiagonal(); 
        D_ = Vector2d(msg->data[2], msg->data[3]).asDiagonal();
        RCLCPP_INFO(this->get_logger(), "Parameters changed!: K = [%f %f], D = [%f %f]", K_(0,0), K_(1,1), D_(0,0), D_(1,1));
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr current_foot_pos_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_pos_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr para_k_d_sub_;

    std::vector<std::string> leg_joint_names =
    {
        //主动关节
        "Joint_lff", "Joint_lfb",
        "Joint_lbf", "Joint_lbb",
        "Joint_rff", "Joint_rfb",
        "Joint_rbf", "Joint_rbb",
        //被动关节
        "Joint_lff_1", "Joint_lfb_1",
        "Joint_lbf_1", "Joint_lbb_1",
        "Joint_rff_1", "Joint_rfb_1",
        "Joint_rbf_1", "Joint_rbb_1"
    };

    std::vector<Vector2d> foot_pos_desired_ = 
    {
        {0.0, -0.2},
        {0.0, -0.2},
        {0.0, -0.2},
        {0.0, -0.2}
    };
    std::vector<Vector2d> foot_vel_desired_ = std::vector<Vector2d>(4, Vector2d::Zero());

    //重力补偿 固定偏置
    std::vector<Vector2d> gravity_comp_ = 
    {
        {-0.43, 0.43},
        {-0.43, 0.43},
        {-0.43, 0.43},
        {-0.43, 0.43}
    };

    //阻抗参数
    Matrix2d K_ = Vector2d(500.0, 1000.0).asDiagonal(); 
    Matrix2d D_ = Vector2d(5.0, 30.0).asDiagonal();

    double BASE_HALF = 0.04;    // 基座一半宽度 = 80mm/2
    double L_ACTIVE  = 0.09;    // 主动杆长度 (单位m)
    double L_PASSIVE = 0.16;   // 被动杆长度（两根相同）

    double total_mass = 3.77;  //整机重量（kg）
    double g = 9.81;    //重力加速度
    size_t n_support = 4;
    double Fz_per_leg = total_mass * g / n_support; //地面对每条腿的支撑力

    pinocchio::Model pin_model_;
    pinocchio::Data pin_data_;
    std::unordered_map<std::string, size_t> joint_name_to_pin_idx_;

    // pinocchio 计算所需的位姿数据
    VectorXd q;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianImpedance>());
    rclcpp::shutdown();
    return 0;
}