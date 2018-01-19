#include <ros/ros.h>
#include "loam_velodyne/LaserLocalization.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "LaserLocalization");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::LaserLocalization LaserLocalization(0.1);

  if (LaserLocalization.setup(node, privateNode)) {
    // initialization successful
    LaserLocalization.spin();
  }

  return 0;
}
