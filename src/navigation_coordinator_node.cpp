#include "path_planning/avoidance_dwa.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "coordinator");
  try {
    NavigationCoordinator coordinator;
    coordinator.process();
  } catch (const std::exception& error) {
    ROS_FATAL("Navigation initialization failed: %s", error.what());
    return 1;
  }
  return 0;
}
