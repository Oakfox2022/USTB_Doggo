#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/bool.hpp>

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl() : Node("keyboard_control")
    {
        forward_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/forward_speed", 10);
        wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_velocity_controller/commands", 10);
        if_trot_pub_ = this->create_publisher<std_msgs::msg::Bool>("/if_trot", 10);

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

            std_msgs::msg::Float64 forward_speed_msg;
            forward_speed_msg.data = forward_speed_;
            forward_speed_pub_->publish(forward_speed_msg);

            std_msgs::msg::Float64MultiArray wheel_vel_msg;
            wheel_vel_msg.data = wheel_vel_;
            wheel_vel_pub_->publish(wheel_vel_msg);

            std_msgs::msg::Bool if_trot_msg;
            if_trot_msg.data = IF_trot;
            if_trot_pub_->publish(if_trot_msg);
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
        case 'w':
            if(IF_foot == true && IF_trot == true)
            {
                forward_speed_ += 0.1;
                RCLCPP_INFO(this->get_logger(), "Current foot forward speed: %f", forward_speed_);
            }
            else if(IF_foot == true && IF_trot == false)
            {
                RCLCPP_WARN(this->get_logger(), "Press 't' to start trot!");
            }
            else if(IF_foot == false)
            {
                L_VEL_ += DELTA_VEL_;
                R_VEL_ += DELTA_VEL_;
                wheel_vel_ = {L_VEL_, L_VEL_, -R_VEL_, -R_VEL_};
                RCLCPP_INFO(this->get_logger(), "Current wheel velocity: L_VEL_ = %f, R_VEL_ = %f", L_VEL_, R_VEL_);
            }
            break;
        case 's':
            if(IF_foot == true && IF_trot == true)
            {
                forward_speed_ -= 0.1;
                RCLCPP_INFO(this->get_logger(), "Current foot forward speed: %f", forward_speed_);
            }
            else if(IF_foot == true && IF_trot == false)
            {
                RCLCPP_WARN(this->get_logger(), "Press 't' to start trot!");
            }
            else if(IF_foot == false)
            {
                L_VEL_ -= DELTA_VEL_;
                R_VEL_ -= DELTA_VEL_;
                wheel_vel_ = {L_VEL_, L_VEL_, -R_VEL_, -R_VEL_};
                RCLCPP_INFO(this->get_logger(), "Current wheel velocity: L_VEL_ = %f, R_VEL_ = %f", L_VEL_, R_VEL_);
            }
            break;

        case 'a':
            if(IF_foot == false)
            {
                L_VEL_ -= DELTA_VEL_;
                R_VEL_ += DELTA_VEL_;
                wheel_vel_ = {L_VEL_, L_VEL_, -R_VEL_, -R_VEL_};
                RCLCPP_INFO(this->get_logger(), "Current wheel velocity: L_VEL_ = %f, R_VEL_ = %f", L_VEL_, R_VEL_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Press 'f' to control wheels!");
            }
            break;
        case 'd':
            if(IF_foot == false)
            {
                L_VEL_ += DELTA_VEL_;
                R_VEL_ -= DELTA_VEL_;
                wheel_vel_ = {L_VEL_, L_VEL_, -R_VEL_, -R_VEL_};
                RCLCPP_INFO(this->get_logger(), "Current wheel velocity: L_VEL_ = %f, R_VEL_ = %f", L_VEL_, R_VEL_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Press 'f' to control wheels!");
            }
            break;

        case 'f':
            if(IF_foot == false)
            {
                IF_foot = true;
                RCLCPP_INFO(this->get_logger(), "Foot control!");
            }
            else
            {
                IF_foot = false;
                RCLCPP_INFO(this->get_logger(), "Wheel control!");
            }
            break;

        case 't':
            if(IF_trot == false)
            {
                IF_trot = true;
                RCLCPP_INFO(this->get_logger(), "Trot started!");
            }
            else
            {
                IF_trot = false;
                RCLCPP_INFO(this->get_logger(), "Trot closed!");
                forward_speed_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "Current foot forward speed: %f", forward_speed_);
            }
            break;

        case 'p':
            if(IF_foot == true)
            {
                forward_speed_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "Current foot forward speed: %f", forward_speed_);
            }
            else if(IF_foot == false)
            {
                L_VEL_ = 0.0;
                R_VEL_ = 0.0;        
                wheel_vel_ = {L_VEL_, L_VEL_, -R_VEL_, -R_VEL_};
                RCLCPP_INFO(this->get_logger(), "Current wheel velocity: L_VEL_ = %f, R_VEL_ = %f", L_VEL_, R_VEL_);
            }
            break;
        
        default:
            break;
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
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr forward_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr if_trot_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double forward_speed_ = 0.0;
    std::vector<double> wheel_vel_ = {0.0, 0.0, 0.0, 0.0};

    double L_VEL_ = 0.0;
    double R_VEL_ = 0.0;
    double DELTA_VEL_ = 0.5;

    bool IF_foot = false;
    bool IF_trot = false;

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