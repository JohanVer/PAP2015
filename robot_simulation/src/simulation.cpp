#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_sender");
  ros::NodeHandle n;
  static tf::TransformBroadcaster br;
  ros::Rate loop_rate(30);

  float x = 0.0;
  float y =0.0;
  while(ros::ok())
  {
	  tf::Transform transform;
	  tf::Transform transformReference;
	  tf::Transform transformX;
	  tf::Transform transformY;

	  // Reference frame
	  transformReference.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	  tf::Quaternion qRef;
	  qRef.setRPY(0, 0, 0);
	  transformReference.setRotation(qRef);

	  // Base_link frame
	  transform.setOrigin( tf::Vector3(0, 0, 0) );
	  tf::Quaternion q;
	  q.setRPY(0, 0, 0);
	  transform.setRotation(q);

	  // X-Link
	  x = x+0.001;
	  if(x>=0.3) x =0.0;

	  y = y+0.001;
	  	  if(y>=0.09) y =0.0;

	  transformX.setOrigin( tf::Vector3(0.0059907+x, 0.052959, 0.11591) );
	  tf::Quaternion qX;
	  qX.setX(0.707108);
	  qX.setY(0);
	  qX.setZ(0);
	  qX.setW(0.707105);
	  transformX.setRotation(qX);

	  // Y-Link
	  transformY.setOrigin( tf::Vector3(0.059991+x, 0.22188+y, 0.15122) );
	  tf::Quaternion qY;
	  qY.setX(0.707107);
	  qY.setY(0.707107);
	  qY.setZ(1.29867e-06);
	  qY.setW(-1.29867e-06);
	  transformY.setRotation(qY);


	  br.sendTransform(tf::StampedTransform(transformReference,ros::Time::now(),"/base_link","/world"));
	  //br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/world","/base_link"));

	  // Axes
	  br.sendTransform(tf::StampedTransform(transformX,ros::Time::now(),"/base_link","/XLink"));
	  br.sendTransform(tf::StampedTransform(transformY,ros::Time::now(),"/base_link","/YLink"));


	  ros::spinOnce();
	  loop_rate.sleep();

  }
  return 0;
}
