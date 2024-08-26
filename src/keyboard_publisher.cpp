#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <termios.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

class KeyboardPublisher {
public:
    KeyboardPublisher() {
        // pub_ = nh_.advertise<std_msgs::Char>("keyboard", 30);
        pub_ = nh_.advertise<std_msgs::Char>("keyboard", 5);
        // Set terminal to non-blocking mode
        fd_ = open("/dev/tty", O_RDONLY | O_NONBLOCK);

        // Initialize last_char with a default value
        last_char_ = '\0'; // or any other default character
    }

    void spin() {
        // ros::Rate rate(30); // 30 Hz
        ros::Rate rate(5); // 5 Hz
        while (ros::ok()) {
            char ch;
            if (read(fd_, &ch, 1) > 0) {
                // Check if the character is one of the valid ones
                if (ch == 'a' || ch == 'n' || ch == 'm' || ch == 's' || ch == 'q' || ch == 'h') {
                    last_char_ = ch;
                }

                // Publish the last valid character
                // std_msgs::Char msg;
                // msg.data = last_char_;
                // pub_.publish(msg);
            }
            std_msgs::Char msg;
            msg.data = last_char_;
            pub_.publish(msg);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    int fd_;
    char last_char_; // Store the last valid character
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_publisher");
    KeyboardPublisher kp;
    kp.spin();
    return 0;
}
