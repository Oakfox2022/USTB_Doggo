#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl() : Node("keyboard_control")
    {
        wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_velocity_controller/commands", 10);
        foot_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/desired_foot_positions", 10);

        //接收的当前足端位置
        current_foot_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/current_foot_position", 10,
            std::bind(&KeyboardControl::currentFootPosCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(8),   //125Hz（由键盘回报率决定）
            std::bind(&KeyboardControl::timerCallback, this)
        );

        setNonBlockingTerminal();
    }

    ~KeyboardControl()
    {
        resetTerminal();
    }

private:
    void timerCallback()
    {
        char c = getcharNonBlocking();
        if (c != 0)
        {
            processKey(c);
            std_msgs::msg::Float64MultiArray msg;
            msg.data = wheel_vel_;
            wheel_vel_pub_->publish(msg);
            msg.data = foot_pos_;
            foot_pos_pub_->publish(msg);
        }
    }

    char getcharNonBlocking()
    {
        char c = 0;
        int r = read(STDIN_FILENO, &c, 1);
        if (r > 0)
            return c;
        return 0;
    }

    void processKey(char key)
    {
        switch (key)
        {
        //前进后退左转右转
        case 'w':
            L_VEL_ += DELTA_VEL_;
            R_VEL_ += DELTA_VEL_;
            wheel_vel_ = {L_VEL_, L_VEL_, -R_VEL_, -R_VEL_};
            RCLCPP_INFO(this->get_logger(), "Current wheel velocity: L_VEL_ = %f, R_VEL_ = %f", L_VEL_, R_VEL_);
            break;
        case 's':
            L_VEL_ -= DELTA_VEL_;
            R_VEL_ -= DELTA_VEL_;
            wheel_vel_ = {L_VEL_, L_VEL_, -R_VEL_, -R_VEL_};
            RCLCPP_INFO(this->get_logger(), "Current wheel velocity: L_VEL_ = %f, R_VEL_ = %f", L_VEL_, R_VEL_);
            break;
        case 'a':
            L_VEL_ -= DELTA_VEL_;
            R_VEL_ += DELTA_VEL_;
            wheel_vel_ = {L_VEL_, L_VEL_, -R_VEL_, -R_VEL_};
            RCLCPP_INFO(this->get_logger(), "Current wheel velocity: L_VEL_ = %f, R_VEL_ = %f", L_VEL_, R_VEL_);
            break;
        case 'd':
            L_VEL_ += DELTA_VEL_;
            R_VEL_ -= DELTA_VEL_;
            wheel_vel_ = {L_VEL_, L_VEL_, -R_VEL_, -R_VEL_};
            RCLCPP_INFO(this->get_logger(), "Current wheel velocity: L_VEL_ = %f, R_VEL_ = %f", L_VEL_, R_VEL_);
            break;
        case 'p':
            L_VEL_ = 0.0;
            R_VEL_ = 0.0;        
            wheel_vel_ = {L_VEL_, L_VEL_, -R_VEL_, -R_VEL_};
            RCLCPP_INFO(this->get_logger(), "Current wheel velocity: L_VEL_ = %f, R_VEL_ = %f", L_VEL_, R_VEL_);
            break;

        //整机高度控制
        case 'q':
            if(Z_POS_ > Z_MIN_)
            {
                Z_POS_ -= 0.01;
            }
            foot_pos_[1] = Z_POS_;
            foot_pos_[3] = Z_POS_;
            foot_pos_[5] = Z_POS_;
            foot_pos_[7] = Z_POS_;
            break;
        case 'e':
            if(Z_POS_ < Z_MAX_)
            {
                Z_POS_ += 0.01;
            }
            foot_pos_[1] = Z_POS_;
            foot_pos_[3] = Z_POS_;
            foot_pos_[5] = Z_POS_;
            foot_pos_[7] = Z_POS_;
            break;
        
        //单腿控制
        case '1':
            foot_num_ = 0;//控制左前腿
            break;
        case '2':
            foot_num_ = 2;//控制左后腿
            break;
        case '3':
            foot_num_ = 4;//控制右前腿
            break;
        case '4':
            foot_num_ = 6;//控制右后腿
            break;
        case 'j':
            foot_pos_[foot_num_] -= 0.01;//前伸
            break;
        case 'l':
            foot_pos_[foot_num_] += 0.01;//后伸
            break;
        case 'k':
            foot_pos_[foot_num_ + 1] -= 0.01;//下压
            break;
        case 'i':
            foot_pos_[foot_num_ + 1] += 0.01;//上抬
            break;

        default:
            break;
        }
    }

    void currentFootPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        for(size_t i = 0; i < msg->data.size(); ++i)
        {
            current_foot_position_[i] = msg->data[i];
        }
    }

    void setNonBlockingTerminal()
    {
        tcgetattr(STDIN_FILENO, &orig_termios_);
        termios new_termios = orig_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    void resetTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr foot_pos_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_foot_pos_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    size_t foot_num_ = 0;
    double L_VEL_ = 0.0;
    double R_VEL_ = 0.0;
    double DELTA_VEL_ = 0.5;
    double Z_POS_ = -0.2;
    double Z_MAX_ = -0.06;
    double Z_MIN_ = -0.23;
    std::vector<double> wheel_vel_ = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> foot_pos_ = {0.0, Z_POS_, 0.0, Z_POS_, 0.0, Z_POS_, 0.0, Z_POS_};
    std::vector<double> current_foot_position_ = foot_pos_;
    termios orig_termios_;
};
        

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
