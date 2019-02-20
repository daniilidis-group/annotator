/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "annotator/PlayerCmd.h"

#include <audio_common_msgs/AudioDataStamped.h>
#include <ffmpeg_image_transport/ffmpeg_decoder.h>

#include <ffmpeg_image_transport_msgs/FFMPEGPacket.h>
#include <flex_sync/sync.h>

#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <ros/ros.h>

#include <thread>

#include <vector>
#include <unordered_map>
#include <deque>
#include <string>
#include <mutex>
#include <condition_variable>
#include <memory>

namespace annotator {
  class BagPlayer {
    using Image = sensor_msgs::Image;
    using ImageConstPtr = sensor_msgs::ImageConstPtr;
    using FFMPEGPacket  = ffmpeg_image_transport_msgs::FFMPEGPacket;
    using FFMPEGPacketConstPtr = FFMPEGPacket::ConstPtr;
    using AudioDataStamped = audio_common_msgs::AudioDataStamped;
    using AudioDataStampedConstPtr = AudioDataStamped::ConstPtr;
    using string = std::string;
    using ThreadPtr = std::shared_ptr<std::thread>;
    using Sync = flex_sync::Sync<Image>;
  public:
    class Session {
    public:
      Session(ros::NodeHandle &nh,
              std::shared_ptr<Sync> sync,
              const string &topic,
              const string &outTopic);
      ~Session();
      void callback(const ImageConstPtr &img, bool isKeyFrame);
      void enqueueMessage(const FFMPEGPacketConstPtr &msg);
      void publish(const ImageConstPtr &img);
      void reset();
    private:
      void run();
      void processMessage(const FFMPEGPacketConstPtr &msg);
      // --------- variables
      string                  topic_;
      ros::Publisher          pub_;
      bool                    keepRunning_{true};
      std::deque<FFMPEGPacketConstPtr> packets_;
      ffmpeg_image_transport::FFMPEGDecoder decoder_;
      ThreadPtr               thread_;
      mutable std::mutex              mutex_;
      mutable std::condition_variable cv_;
      std::shared_ptr<Sync>   sync_;
    };
    
    BagPlayer(const ros::NodeHandle& pnh);
    bool initialize();
  private:
    void openBag(const string &fname);
    void mainThread();
    bool command(PlayerCmd::Request& req,  PlayerCmd::Response &res);
    void syncCallback(const std::vector<ImageConstPtr> &msgs);

    void play(const ros::Time &t, const ros::Duration &d, bool monitorOnly);
    // ------------------------ variables --------
    typedef std::shared_ptr<Session> SessionPtr;
    typedef std::unordered_map<string, SessionPtr> SessionMap;
    std::shared_ptr<Sync>     sync_;
    ros::NodeHandle     nh_;
    ros::Subscriber     sub_;
    SessionMap          sessions_;
    rosbag::Bag         bag_;
    ros::Publisher      audioPub_;
    ros::Publisher      clockPub_;
    ros::Time           currentTime_{ros::Time(0)};
    string              audioTopic_;
    std::vector<string> imageTopics_;
    string              monitorTopic_;
    ThreadPtr           thread_;
    std::mutex              mutex_;
    std::condition_variable cv_;
    std::shared_ptr<PlayerCmd::Request> command_;
    ros::ServiceServer  cmdService_;
    int                 minQueueSize_;
    ros::Duration       audioBufferTime_;
  };

}

