#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class KeyboardControlTrot : public rclcpp::Node
{
public:
    KeyboardControlTrot() : Node("keyboard_control_trot")
    {
        forward_speed_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_speed", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(8),   //125Hz（由键盘回报率决定）
            std::bind(&KeyboardControlTrot::timerCallback, this)
        );

        setNonBlockingTerminal();
    }

    ~KeyboardControlTrot()
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
            msg.data = forward_speed_;
            forward_speed_pub_->publish(msg);
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
            forward_speed_[0] += 0.1;
            RCLCPP_INFO(this->get_logger(), "Current forward speed: %f", forward_speed_[0]);
            break;
        case 's':
            forward_speed_[0] -= 0.1;
            RCLCPP_INFO(this->get_logger(), "Current forward speed: %f", forward_speed_[0]);
            break;
        case 'p':
            forward_speed_[0] = 0.0;
            RCLCPP_INFO(this->get_logger(), "Current forward speed: %f", forward_speed_[0]);
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
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_speed_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> forward_speed_ = {0.0};

    termios orig_termios_;
};
        

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControlTrot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}