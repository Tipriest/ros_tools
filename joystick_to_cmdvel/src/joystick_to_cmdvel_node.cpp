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
  double clamp(double min_input, double max_input, double input,
               double min_output, double max_output) {
    return (input - min_input) / (max_input - min_input) *
               (max_output - min_output) +
           min_output;
  }
  void deadzone(double deadzone, double &input) {
    if (abs(input) < abs(deadzone)) {
      input = 0;
    }
  }
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
    geometry_msgs::Twist cmd_vel_msg;

    // Map joystick axes to linear and angular velocities
    cmd_vel_msg.linear.x = clamp(-32767, 32767, msg->axes[1], -1.0,
                                 1.0); // Left stick vertical axis
    cmd_vel_msg.angular.z = -clamp(-32767, 32767, msg->axes[2], -1.0,
                                   1.0); // Left stick horizontal axis
    deadzone(0.04, cmd_vel_msg.linear.x);
    deadzone(0.04, cmd_vel_msg.angular.z);
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
