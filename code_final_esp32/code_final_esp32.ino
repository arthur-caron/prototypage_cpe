/*
 * Take a measure using TOF sensor and print an alert if the measure has changed
 */
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <Adafruit_VL53L0X.h>

#define distance_min 31
#define distance_max 33

const int ledPin = 2;
const int buttonPin = 3;
int buzzerPin = 4 ;
int buttonState = 0; 
int doorNumber = 1;
int distances[2] = {32, 34} 

ros::NodeHandle nh;

void messageCb(const std_msgs::Bool& msg){
  if (msg.data == false) {
    verify_distance(distance[doorNumber]);
  }
}

std_msgs::Int8 msg;
ros::Publisher pub("/door_number", &msg);
ros::Subscriber<std_msgs::Bool> sub("/is_moving", &messageCb);

Adafruit_VL53L0X mesure_distance = Adafruit_VL53L0X();  // création d’une instance

void setup()
{  
  pinMode(buttonPin, INPUT); 
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin,OUTPUT);
  
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);


  Serial.begin(9600);
  if (!mesure_distance.begin())
  {
    Serial.println("Erreur de démarrage du module VL53L0X");
    while(true);
  }
}

void loop()
{
  // TODO: Remplacer bouton poussoir par le choix de l'utilisateur
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) { 
    msg.data = 2;
    pub.publish(&msg);
  }

  delay(100);
}

void verify_distance(distanceToDoor){
  VL53L0X_RangingMeasurementData_t mesure;  // déclaration du pointeur de mesure
  mesure_distance.rangingTest(&mesure, false);  // effectue la mesure; true pour passer en mode DEBUG
  
  if (mesure.RangeStatus != 4){
    int distance_cm = mesure.RangeMilliMeter / 10; // convert to cm

    // If the distance is not between distance_min and ditance_max, send an alert to the app
    if(distance_cm < (distanceToDoor - 1) || distance_cm > (distanceToDoor + 1)){
      Serial.println("ALERT"); 
      //Serial.println(distance_cm); // Debug to see which distance
      
      // TODO: Send alert to the app
      playSound();
      
    }
    else {
      Serial.print("Distance (cm):");
      Serial.println(distance_cm);  // affichage distance en cm (un entier)
    }
    
  }
  else
  {
    Serial.println("Distance > 120 cm");
  }
}

void playSound(){
  while(1){
    for(i=0;i<50;i++){
      digitalWrite(buzzerPin,HIGH);
      delay(1);
      digitalWrite(buzzerPin,LOW);
      delay(1);//wait for 1ms
    }
  }
}
