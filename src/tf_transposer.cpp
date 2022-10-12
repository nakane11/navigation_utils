#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

std::string child_frame_id;

void cb(const geometry_msgs::PoseStamped& msg){
  static tf::TransformBroadcaster br;
  // transpose msg to base_link
  //divide pos by 2
  tf::Transform transform(
                          tf::Quaternion(
                                         msg.pose.orientation.x,
                                         msg.pose.orientation.y,
                                         msg.pose.orientation.z,
                                         msg.pose.orientation.w
                                         ),
                          tf::Vector3(msg.pose.position.x,
                                      msg.pose.position.y,
                                      msg.pose.position.z)
                          );
  br.sendTransform(
                   tf::StampedTransform(
                                        transform,
                                        msg.header.stamp,
                                        msg.header.frame_id,
                                        child_frame_id
                                        )
                   );
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_transposer");
  ros::NodeHandle nh("~");
  nh.param("child_frame_id", child_frame_id, std::string("base_link_offset"));
  ros::Subscriber sub = nh.subscribe("input/pose", 1, &cb);

  ros::spin();
  return 0;
};
