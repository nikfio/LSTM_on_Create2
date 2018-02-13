#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
 
int main(int argc, char** argv){
   ros::init(argc, argv, "tf_basetolaser_broadcaster");
   ros::NodeHandle n;
 
   ros::Rate r(100);
 
   tf::TransformBroadcaster base_to_laser_broadcaster;
	
   tf::TransformBroadcaster footprint_to_baselink_broadcaster;

   
  
   while(n.ok()){
     base_to_laser_broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.036, 0.015)),
          ros::Time::now(),"base_link", "left_wheel_link"));

	 base_to_laser_broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, -0.036, 0.015)),
          ros::Time::now(),"base_link", "right_wheel_link"));

      r.sleep();
    }

}
