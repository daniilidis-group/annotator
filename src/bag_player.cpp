/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "annotator/bag_player.h"
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <boost/range/irange.hpp>
#include <iomanip>
#include <climits>

namespace annotator {
  using boost::irange;

  BagPlayer::Session::Session(ros::NodeHandle &nh, const string &topic,
                              const string &outTopic) :
    topic_(topic) {
    pub_ = nh.advertise<sensor_msgs::Image>(outTopic, 1);
  }
  void BagPlayer::Session::callback(const ImageConstPtr &img) {
    if (pub_.getNumSubscribers() > 0) {
      pub_.publish(img);
    }
  }

  void BagPlayer::Session::processMessage(const FFMPEGPacketConstPtr &msg) {
    if (!decoder_.isInitialized()) {
      decoder_.initialize(msg, boost::bind(&BagPlayer::Session::callback,
                                           this, _1));
      if (!decoder_.isInitialized()) {
        return;
      }
    }
    decoder_.decodePacket(msg);
  }

  BagPlayer::BagPlayer(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  bool BagPlayer::initialize() {
    if (!nh_.getParam("image_topics", imageTopics_)) {
      ROS_ERROR("no image topics found!");
      return (false);
    }
    string bagFile;
    nh_.param<string>("bag_file",  bagFile,  "");
    ROS_INFO_STREAM("playing from bag: " << bagFile);
    if (!bagFile.empty()) {
      openBag(bagFile);
    } else {
      ROS_ERROR_STREAM("must specify bag_file!");
      return (false);
    }
    cmdService_ = nh_.advertiseService("command", &BagPlayer::command, this);

    thread_ = std::make_shared<boost::thread>(
      &BagPlayer::mainThread, this);
    return (true);
  }

  bool BagPlayer::command(PlayerCmd::Request& req,
                          PlayerCmd::Response &res) {
    ROS_INFO_STREAM("got command: " << req.command);
    std::unique_lock<std::mutex> lock(mutex_);
    command_.reset(new PlayerCmd::Request(req));
    cv_.notify_all();
    ROS_INFO_STREAM("waiting for command completion: " << req.command);
    while (command_) {
      cv_.wait(lock);
    }
    ROS_INFO_STREAM("command complete: " << req.command);
    res.success = true;
    return (true);
  }

  void BagPlayer::mainThread() {
    while (true) {
      PlayerCmd::Request req;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!command_) { // wait for command to arrive
          ROS_INFO_STREAM("waiting for command to arrive!");
          cv_.wait(lock);
          ROS_INFO_STREAM("got something!");
        }
        req = *command_; // make deep copy
        command_.reset();
        cv_.notify_all();
      }
      //
      if (req.command == "start") {
        ROS_INFO_STREAM("starting playback!");
        rosbag::View view(bag_, rosbag::TopicQuery(imageTopics_));
        for (const rosbag::MessageInstance &m: view) {
          FFMPEGPacketConstPtr msg = m.instantiate<FFMPEGPacket>();
          if (msg) {
            SessionPtr sess = sessions_[m.getTopic()];
            sess->processMessage(msg);
          }
          if (!ros::ok()) {
            break;
          }
          std::unique_lock<std::mutex> lock(mutex_);
          if (command_) {
            if (command_->command == "stop");
            ROS_INFO_STREAM("interrupted playback!");
            break;
          }
        }
      } else if (req.command == "stop") {
        ROS_INFO_STREAM("stopping playback!");
      } else {
        ROS_ERROR_STREAM("unknown command!");
      }
    }
  }
 
  void BagPlayer::openBag(const string &fname) {
    bag_.open(fname, rosbag::bagmode::Read);
    std::vector<string> topics = imageTopics_;
    int sessionIdx(0);
    for (const auto &topic: topics) {
      rosbag::View cv(bag_, rosbag::TopicQuery({topic}));
      if (cv.begin() == cv.end()) {
        ROS_WARN_STREAM("cannot find topic: " << topic);
      }
      if (sessions_.count(topic) == 0) {
        string pub_topic = "image_" + std::to_string(sessionIdx);
        sessions_[topic].reset(new Session(nh_, topic, pub_topic));
        sessionIdx++;
      } else {
        ROS_WARN_STREAM("duplicate topic: " << topic);
      }
    }
  }
  
}  // namespace
