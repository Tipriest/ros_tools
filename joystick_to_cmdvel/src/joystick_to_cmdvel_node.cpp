#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class JoystickToCmdVel {
public:
  JoystickToCmdVel() {
    // Initialize subscriber and publisher
    joy_sub_ = nh_.subscribe("joy", 10, &JoystickToCmdVel::joyCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher cmd_vel_pub_;

  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
    geometry_msgs::Twist cmd_vel_msg;

    // Map joystick axes to linear and angular velocities
    cmd_vel_msg.linear.x = msg->axes[1] * 0.00005; // Left stick vertical axis
    cmd_vel_msg.angular.z =
        msg->axes[0] * 0.00005; // Left stick horizontal axis
    std::cout << "linear.x = " << cmd_vel_msg.linear.x << "angular.z"
              << cmd_vel_msg.angular.z << std::endl;
    // Publish the Twist message
    cmd_vel_pub_.publish(cmd_vel_msg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick_to_cmdvel_node");
  JoystickToCmdVel converter;
  ros::spin();
  return 0;
}
