#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

class TfTransposer{
public:

  void cb(const geometry_msgs::PoseStampedPtr& msg){
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
    tf::Transform transform(
                            tf::Quaternion(0,0,0,1),
                            tf::Vector3(
                                        base_to_target.getOrigin().getX()/2.0,
                                        base_to_target.getOrigin().getY()/2.0,
                                        0.0
                                        )
                            );
    tfb_.sendTransform(
                     tf::StampedTransform(
                                          transform,
                                          msg->header.stamp,
                                          base_frame_id,
                                          child_frame_id
                                          )
                     );
  }

  TfTransposer(): nh_(), pnh_("~"), tfl_(nh_) {
    pnh_.param("base_frame_id", base_frame_id, std::string("base_footprint"));
    pnh_.param("child_frame_id", child_frame_id, std::string("base_converted"));
    sub_ = pnh_.subscribe("input/pose", 1, &TfTransposer::cb, this);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf::TransformBroadcaster tfb_;
  tf::TransformListener tfl_;
  ros::Subscriber sub_;

  std::string base_frame_id;
  std::string child_frame_id;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_transposer");
  TfTransposer tft;
  ros::spin();

  return 0;
};
