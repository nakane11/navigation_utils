#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <math.h>
#include <std_msgs/Float32.h>

class ComputeYDistance
{
public:
  ComputeYDistance(): pnh_("~")
  {
    pub_ = pnh_.advertise<std_msgs::Float32>("output", 5);
    sub_ = pnh_.subscribe("input", 1,
        &ComputeYDistance::callback, this);
  }
  
  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    double sum = 0;
    double size = 0;
    for (int i=0; i<cloud->size(); i++)
      {
        pcl::PointXYZ point = cloud->points[i];
        if (point.x < 1.0 && point.x > -0.5)
          {
            sum += abs(point.y);
            size += 1;
          }
      }
    std_msgs::Float32Ptr mean(new std_msgs::Float32);
    if (size > 0){
      mean->data = sum / size;
      pub_.publish(mean);
    }
  }
  
private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::NodeHandle pnh_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "compute_y_distance");
  ComputeYDistance cyd;
  ros::spin();

  return 0;
}
