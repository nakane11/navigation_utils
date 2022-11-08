#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

class TfTransposer{
public:

  void pose_cb(const geometry_msgs::PoseStampedPtr& msg){
    tf::StampedTransform base_to_laser, laser_to_target;
    tf::poseMsgToTF(msg->pose, laser_to_target);
    try {
      tfl_.waitForTransform(base_frame_id, msg->header.frame_id,
                            msg->header.stamp, ros::Duration(3.0));
      tfl_.lookupTransform(base_frame_id, msg->header.frame_id,
                           msg->header.stamp, base_to_laser);
    }
    catch (tf2::TransformException &e) {
      return;
    }
    tf::Transform base_to_target;
    base_to_target = base_to_laser * laser_to_target;
    base_people_tf.setRotation(tf::Quaternion(0,0,0,1));
    base_people_tf.setOrigin(tf::Vector3(
                                         base_to_target.getOrigin().getX()/2.0,
                                         base_to_target.getOrigin().getY()/2.0,
                                         0.0
                                         ));
    last_received_time_ = ros::Time::now();
  }

  void timer_cb(void)
  {
    if ((ros::Time::now() - last_received_time_) < valid_duration){
      tfb_.sendTransform(
                         tf::StampedTransform(base_people_tf,
                                              ros::Time::now(),
                                              base_frame_id,
                                              child_frame_id
                                              )
                         );
    }
    else{
      tf::Transform base_tf;
      base_tf.setIdentity();
      tfb_.sendTransform(
                         tf::StampedTransform(
                                              base_tf,
                                              ros::Time::now(),
                                              base_frame_id,
                                              child_frame_id
                                              )
                         );
    }    
  }

  TfTransposer(): nh_(), pnh_("~"), tfl_(nh_) {
    pnh_.param("base_frame_id", base_frame_id, std::string("base_footprint"));
    pnh_.param("child_frame_id", child_frame_id, std::string("base_converted"));
    sub_ = pnh_.subscribe("input/pose", 1, &TfTransposer::pose_cb, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent& e) {
        timer_cb();
      });
    valid_duration = ros::Duration(5);
    last_received_time_ = ros::Time::now() - valid_duration;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf::TransformBroadcaster tfb_;
  tf::TransformListener tfl_;
  ros::Subscriber sub_;
  ros::Timer timer_;
  std::string base_frame_id;
  std::string child_frame_id;
  tf::Transform base_people_tf;
  tf::Transform base_tf;
  ros::Duration valid_duration;
  ros::Time last_received_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_transposer");
  TfTransposer tft;
  ros::spin();

  return 0;
};
