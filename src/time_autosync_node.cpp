#include "time_autosync/time_autosync.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "time_autosync_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  TimeAutosync time_autosync(nh, nh_private);

  ros::spin();

  return 0;
}
