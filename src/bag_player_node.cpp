/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "annotator/bag_player.h"

#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "decode_bag");
  ros::NodeHandle pnh("~");
  try {
    annotator::BagPlayer node(pnh);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
