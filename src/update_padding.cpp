#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <math.h>
#include <std_msgs/Float32.h>

class UpdatePadding
{
public:
  UpdatePadding(): pnh_("~")
  {
    pub_ = pnh_.advertise<std_msgs::Float32>("output", 5);
    sub_ = pnh_.subscribe("input", 1,
        &UpdatePadding::callback, this);
  }

  float computeNorm(pcl::PointXYZ point)
  {
    double norm = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));
    return norm;
  }
  
  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    double sum = 0;
    for (int i=0; i<cloud->size(); i++)
      {
        sum += computeNorm(cloud->points[i]);
      }
    std_msgs::Float32Ptr mean(new std_msgs::Float32);
    mean->data = cloud->size();
    pub_.publish(mean);
    // std::cout << sum << std::endl;
  }
  
private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::NodeHandle pnh_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "update_padding");
  UpdatePadding update_padding;
  ros::spin();

  return 0;
}
