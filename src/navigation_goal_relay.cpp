#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

// Publish the initial request once and relay replacements without modification.
// The coordinator validates requests and resolves their frames; invalid requests
// must also be forwarded so that they stop the previous navigation task.
class NavigationGoalRelay {
 public:
  NavigationGoalRelay() : private_nh_("~") {
    publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/wamv/navigation/goal_request", 1, true);
    subscriber_ = nh_.subscribe("/wamv/navigation/goal", 1,
                                &NavigationGoalRelay::update, this);
    bool use_initial_goal;
    private_nh_.param("use_initial_goal", use_initial_goal, true);
    if (use_initial_goal) {
      geometry_msgs::PoseStamped goal;
      private_nh_.param<std::string>("goal_frame", goal.header.frame_id, "map");
      private_nh_.param("goal_x", goal.pose.position.x, -500.0);
      private_nh_.param("goal_y", goal.pose.position.y, 232.0);
      double yaw;
      private_nh_.param("goal_yaw", yaw, 0.0);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      goal.pose.orientation.x = q.x();
      goal.pose.orientation.y = q.y();
      goal.pose.orientation.z = q.z();
      goal.pose.orientation.w = q.w();
      publisher_.publish(goal);
    }
  }
 private:
  void update(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    publisher_.publish(*goal);
  }
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_relay");
  NavigationGoalRelay goals;
  ros::spin();
  return 0;
}
