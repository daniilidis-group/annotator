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
#include <rosgraph_msgs/Clock.h>


namespace annotator {
  using boost::irange;

  BagPlayer::Session::Session(ros::NodeHandle &nh,
                              std::shared_ptr<Sync> sync,
                              const string &topic,
                              const string &outTopic) :
    topic_(topic), sync_(sync) {
    pub_ = nh.advertise<sensor_msgs::Image>(outTopic, 5);
    thread_ = std::make_shared<std::thread>(
      &BagPlayer::Session::run, this);
  }
  BagPlayer::Session::~Session() {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      keepRunning_ = false;
      cv_.notify_all();
    }
    thread_->join();
  }

  void BagPlayer::Session::run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (keepRunning_) {
      // wait until enough packets have arrived
      while (packets_.empty()) {
        cv_.wait(lock);
      }
      // now drain queue completely
      while (!packets_.empty()) {
        FFMPEGPacketConstPtr p = packets_.front();
        packets_.pop_front();
        processMessage(p);
      }
      cv_.notify_all(); 
    }
  }

  void BagPlayer::Session::callback(const ImageConstPtr &img, bool isKeyFrame) {
    publish(img);
    //sync_->process(topic_, img);
  }

  void BagPlayer::Session::publish(const ImageConstPtr &img) {
    if (pub_.getNumSubscribers() > 0) {
      pub_.publish(img);
    }
  }

  void BagPlayer::Session::reset() {
    std::unique_lock<std::mutex> lock(mutex_);
    packets_.clear();
    decoder_.reset();
  }

  void BagPlayer::Session::processMessage(const FFMPEGPacketConstPtr &msg) {
    if (!decoder_.isInitialized()) {
      decoder_.initialize(msg, boost::bind(&BagPlayer::Session::callback,
                                           this, _1, _2));
      if (!decoder_.isInitialized()) {
        return;
      }
    }
    decoder_.decodePacket(msg);
  }

  void BagPlayer::Session::enqueueMessage(const FFMPEGPacketConstPtr &msg) {
    std::unique_lock<std::mutex> lock(mutex_);
    packets_.push_back(msg);
    cv_.notify_all();
  }

  BagPlayer::BagPlayer(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  void
  BagPlayer::syncCallback(const std::vector<ImageConstPtr> &msgs) {
    ROS_INFO_STREAM("got callback: " << msgs[0]->header.stamp);
    for (const auto i : irange(0ul, msgs.size())) {
      SessionPtr sess = sessions_[imageTopics_[i]];
      sess->publish(msgs[i]);
    }
  }

  bool BagPlayer::initialize() {
    double abt;
    nh_.param<double>("audio_buffer_time", abt, 0.2);
    audioBufferTime_ = ros::Duration(abt);
    nh_.param<string>("audio_topic", audioTopic_, "audio_stamped");
    nh_.param<string>("monitor_topic", monitorTopic_, "monitor_img");
    if (!nh_.getParam("image_topics", imageTopics_)) {
      ROS_ERROR("no image topics found!");
      return (false);
    }

    std::vector<std::vector<string>> tp(1, imageTopics_);
    sync_.reset(
      new Sync(tp, boost::bind(&BagPlayer::syncCallback, this, _1)));
    string bagFile;
    nh_.param<string>("bag_file",  bagFile,  "");
    ROS_INFO_STREAM("playing from bag: " << bagFile);
    if (!bagFile.empty()) {
      openBag(bagFile);
    } else {
      ROS_ERROR_STREAM("must specify bag_file!");
      return (false);
    }
    audioPub_ = nh_.advertise<AudioDataStamped>("audio_stamped", 100);
    clockPub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
    cmdService_ = nh_.advertiseService("command", &BagPlayer::command, this);

    thread_ = std::make_shared<std::thread>(&BagPlayer::mainThread, this);
    return (true);
  }

  bool BagPlayer::command(PlayerCmd::Request& req,
                          PlayerCmd::Response &res) {
    ROS_INFO_STREAM("got command: " << req.command << " v: " << req.value);
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

  void BagPlayer::play(const ros::Time &t, const ros::Duration &d,
                       bool monitorOnly) {
    // reset the decoders for all streams
    for (auto &s : sessions_) {
      s.second->reset();
    }
    std::vector<string> topics = imageTopics_;
    topics.push_back(audioTopic_);
    const ros::Time tEnd = t + d;
    ROS_INFO_STREAM("playing: " << t << " " << tEnd);
    rosbag::View view(bag_, rosbag::TopicQuery(topics), t, tEnd);
    ros::WallTime tw0(0);
    ros::Time t0(0);
    for (const rosbag::MessageInstance &m: view) {
      FFMPEGPacketConstPtr msg = m.instantiate<FFMPEGPacket>();
      ros::Time      t = m.getTime();
      ros::WallTime tw = ros::WallTime::now();
      if (tw0 == ros::WallTime(0)) {
        tw0 = ros::WallTime::now();
        t0 = t;
      } else {
        ros::WallDuration dtw = tw - tw0;
        ros::Duration     dt  = t  -  t0;
        double aheadTime = dt.toSec() - dtw.toSec();
        const ros::Duration sleepTime(0.1);
        if (aheadTime < 0) {
          ROS_WARN_STREAM("falling behind ros time: " << aheadTime);
        }
        if (ros::Duration(aheadTime) - sleepTime > audioBufferTime_) {
          //ROS_INFO_STREAM("sleeping for " << sleepTime);
          // sleep until wall time has caught up with ros time
          ros::WallTime::sleepUntil(tw + ros::WallDuration(sleepTime.toSec()));
        }
      }
      if (msg) {
        if (!monitorOnly || m.getTopic() == monitorTopic_) {
          SessionPtr sess = sessions_[m.getTopic()];
          sess->enqueueMessage(msg);
        }
      }
      AudioDataStampedConstPtr audio = m.instantiate<AudioDataStamped>();
      if (audio) {
        audioPub_.publish(audio);
      }
      currentTime_ = m.getTime();
      rosgraph_msgs::Clock clockMsg;
      clockMsg.clock = currentTime_;
      clockPub_.publish(clockMsg);

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
    ROS_INFO_STREAM("playback finished!");
  }

  void BagPlayer::mainThread() {
    while (true) {
      PlayerCmd::Request req;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!command_) { // wait for command to arrive
          ROS_INFO_STREAM("waiting for command to arrive!");
          cv_.wait(lock);
        }
        req = *command_; // make deep copy
        command_.reset();
        cv_.notify_all();
      }
      //
      if (req.command == "play") {
        ROS_INFO_STREAM("starting playback!");
        // this will hog the thread until stop is encountered
        play(currentTime_, ros::Duration(INT_MAX), true);
      } else if (req.command == "stop") {
        ROS_INFO_STREAM("stopping playback!");
      } else if (req.command == "position") {
        ROS_INFO_STREAM("setting position: " << req.time);
        currentTime_ = req.time;
      } else if (req.command == "replay") {
        ROS_INFO_STREAM("replaying: " << req.time << " duration: " << req.value);
        play(currentTime_, ros::Duration(req.value), false);
      } else {
        ROS_ERROR_STREAM("unknown command!");
      }
    }
  }
 
  void BagPlayer::openBag(const string &fname) {
    std::vector<string> topics = imageTopics_;
    nh_.param<int>("queue_size", minQueueSize_, 20 * (int)imageTopics_.size());
    bag_.open(fname, rosbag::bagmode::Read);
    int sessionIdx(0);

    for (const auto &topic: topics) {
      rosbag::View cv(bag_, rosbag::TopicQuery({topic}));
      if (cv.begin() == cv.end()) {
        ROS_WARN_STREAM("cannot find topic: " << topic);
      }
      if (sessions_.count(topic) == 0) {
        string pub_topic = "image_" + std::to_string(sessionIdx);
        sessions_[topic].reset(new Session(nh_, sync_,
                                           topic, pub_topic));
        sessionIdx++;
      } else {
        ROS_WARN_STREAM("duplicate topic: " << topic);
      }
    }
  }
  
}  // namespace
