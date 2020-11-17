/*
   Publish the door number asked by the user to the topic /door_number
*/

#include <ros.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

std_msgs::Int8 msg;
ros::Publisher pub("/door_number", &msg);

void setup()
{
  nh.initNode();
  nh.advertise(pub);
}


void loop()
{
  msg.data = 2;
  pub.publish(&msg);
  delay(5000);
  msg.data = 1;
  pub.publish(&msg);
  delay(5000);

  nh.spinOnce();
  delay(100);
}
