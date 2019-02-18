/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "annotator/PlayerCmd.h"

#include <ffmpeg_image_transport/ffmpeg_decoder.h>

#include <ffmpeg_image_transport_msgs/FFMPEGPacket.h>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

#include <vector>
#include <unordered_map>
#include <string>
#include <mutex>
#include <condition_variable>
#include <memory>

namespace annotator {
  class BagPlayer {
    using ImageConstPtr = sensor_msgs::ImageConstPtr;
    using FFMPEGPacket  = ffmpeg_image_transport_msgs::FFMPEGPacket;
    using FFMPEGPacketConstPtr = FFMPEGPacket::ConstPtr;
    using string = std::string;
  public:
    class Session {
    public:
      Session(ros::NodeHandle &nh,
              const string &topic,
              const string &outTopic);
      void callback(const ImageConstPtr &img);
      void processMessage(const FFMPEGPacketConstPtr &msg);
    private:
      // --------- variables
      string             topic_;
      ros::Publisher     pub_;
      ffmpeg_image_transport::FFMPEGDecoder decoder_;
    };
    
    BagPlayer(const ros::NodeHandle& pnh);
    bool initialize();
  private:
    void openBag(const string &fname);
    void mainThread();
    bool command(PlayerCmd::Request& req,  PlayerCmd::Response &res);
    // ------------------------ variables --------
    typedef std::shared_ptr<Session> SessionPtr;
    typedef std::unordered_map<string, SessionPtr> SessionMap;
    ros::NodeHandle     nh_;
    ros::Subscriber     sub_;
    SessionMap          sessions_;
    rosbag::Bag         bag_;
    std::vector<string> imageTopics_;
    std::shared_ptr<boost::thread> thread_;
    std::mutex              mutex_;
    std::condition_variable cv_;
    std::shared_ptr<PlayerCmd::Request> command_;
    ros::ServiceServer  cmdService_;
  };

}

