/*
 * Publish mesure to the /distance topic
 */
#include <ros.h>
#include <std_msgs/Int8.h>
#include <Adafruit_VL53L0X.h>

ros::NodeHandle nh;
std_msgs::Int8 msg;
ros::Publisher pub("/distance", &msg);

Adafruit_VL53L0X mesureDistance = Adafruit_VL53L0X(); 


void setup()
{  
  nh.initNode();
  nh.advertise(pub);

  if (!mesureDistance.begin())
  {
    while(true);
  }
}

void loop()
{
  verifyDistance();
  nh.spinOnce();
  delay(100);
}

void verifyDistance(){
  VL53L0X_RangingMeasurementData_t mesure;  // déclaration du pointeur de mesure
  mesureDistance.rangingTest(&mesure, false);  // effectue la mesure; true pour passer en mode DEBUG
  
  if (mesure.RangeStatus != 4)  // mesure correcte
  {
    int distanceCm = mesure.RangeMilliMeter / 10; // convert to cm

    msg.data = distanceCm;
    pub.publish(&msg);
  }
}
