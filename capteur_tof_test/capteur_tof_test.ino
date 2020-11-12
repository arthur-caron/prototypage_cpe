/*
 * Take a measure using TOF sensor and print an alert if the measure has changed
 */

#include <Adafruit_VL53L0X.h>

#define distance_min 31
#define distance_max 33

Adafruit_VL53L0X mesure_distance = Adafruit_VL53L0X();  // création d’une instance

void setup()
{  
  Serial.begin(9600);
  if (!mesure_distance.begin())
  {
    Serial.println("Erreur de démarrage du module VL53L0X");
    while(true);
  }
}

void loop()
{

  // Faire subscriber qui écoute la variable moving du topic /motor_states/pan_tilt_port/
  // Si True, ne pas faire la mesure de distance
  verify_distance();

  delay(100);
}

void verify_distance(){
  VL53L0X_RangingMeasurementData_t mesure;  // déclaration du pointeur de mesure
  mesure_distance.rangingTest(&mesure, false);  // effectue la mesure; true pour passer en mode DEBUG
  
  if (mesure.RangeStatus != 4)  // mesure correcte
  {
    int distance_cm = mesure.RangeMilliMeter / 10; // convert to cm

    // If the distance is not between distance_min and ditance_max, send an alert to the app
    if(distance_cm < distance_min || distance_cm > distance_max){
      Serial.println("ALERT"); 
      //Serial.println(distance_cm); // Debug to see which distance
      
      // TODO: Send alert to the app
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
