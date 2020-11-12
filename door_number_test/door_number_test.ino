/*
 * Publish the door number asked by the user to the topic /door_number
 */

#include <ros.h>
#include <std_msgs/Int8.h>

const int buttonPin = 2;
int buttonState = 0; 

ros::NodeHandle nh;

std_msgs::Int8 msg;
ros::Publisher pub("/door_number", &msg);

void setup() {
  nh.initNode();
  nh.advertise(pub);

  pinMode(buttonPin, INPUT); 
}

void loop() {
  // Remplacer bouton poussoir par le choix de l'utilisateur
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) { 
    msg.data = 2;
    pub.publish(&msg);
  }

  nh.spinOnce();
  
  delay(100);
}
