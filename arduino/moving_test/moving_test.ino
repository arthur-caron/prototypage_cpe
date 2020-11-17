/*
 * Switch on the LED only if the motor is not moving
 */
#include <ros.h>
#include <std_msgs/Bool.h>

const int ledPin = 12;

ros::NodeHandle  nh;

void messageCallback(const std_msgs::Bool& msg)
{
  if (msg.data == false) 
  {
    digitalWrite(ledPin, HIGH);   // TODO: Remplacer par verifydistance()
  }
  else 
  {
    digitalWrite(ledPin, LOW)  ;
  }
}

ros::Subscriber<std_msgs::Bool> sub("/is_moving", &messageCallback);

void setup()
{
  pinMode(ledPin, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

}

void loop()
{
  nh.spinOnce();
  delay(1);
}
