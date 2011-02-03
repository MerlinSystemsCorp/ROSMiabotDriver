#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0 /*pitch*/, 0/*yaw*/, 0/*roll*/, 1), tf::Vector3(0.1/*x*/, 0.0/*y*/, 0.2/*z*/)),
        ros::Time::now(),"base_link", "base_laser"));
    r.sleep();
  }
}

