#include "ros/ros.h"

#include "performance_test_ros1_msgs/PointCloud4m.h"

#include <chrono>

int main(int argc, char**argv)
{

  ros::init(argc, argv, "point_cloud_publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<performance_test_ros1_msgs::PointCloud4m>(
    "PointCloud4m",
    100
  );

  ros::Rate loop_rate(100);

  uint64_t id = 0;

  while (ros::ok())
  { 
    performance_test_ros1_msgs::PointCloud4m msg_p;
    msg_p.id = ++id;
    msg_p.time = std::chrono::steady_clock::now().time_since_epoch().count();

    pub.publish(msg_p);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
