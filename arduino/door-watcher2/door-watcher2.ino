/**
 * Prototype project Gr1
 * Name : door-watcher
 * Purpose : Prototype robot which can watch many doors and play sound if a door is opening 
 * 
 * @author Edouard Petitpierre, Arthur Caron, Eléonore François
 * @version 1.0 14/11/2020
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <Adafruit_VL53L0X.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


/*****************************************/
/*      Variable declaration             */
/*****************************************/

#define BUZZER_PIN 2
#define LED_PIN 12

// Default door number
String doorNumberStr = "1";
int doorNumber = 1; 

int distances[3] = {38, 28, 39}; // Distances to doors


Adafruit_VL53L0X measureDistance = Adafruit_VL53L0X(); // Using TOF sensor to detect motion

unsigned long previousMillis = 0;
const long interval = 1000;
unsigned long previousMillis_alert = 0;
const long interval_alert = 2000;
int buzzerState = 0;

// BLE variables
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
int txValue = 0;
int a = 0;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


/*****************************************/
/*      Function declaration             */
/*****************************************/
int charToInt();
void publishDoorNumber();
void playSound();
void callApp();


/*****************************************/
/*      Node declaration                 */
/*****************************************/
ros::NodeHandle nh;

/**
 * Run the motion detection only if the motor is not moving
 */
void movingCallback(const std_msgs::Bool& msg) {
  if (msg.data == false) {
    verifyDistance(distances[doorNumber-1]);
  }
  else {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
  }
}

// Creation of Publishers/Subscribers
std_msgs::Int8 msg;
ros::Publisher pubDoorNumber("/door_number", &msg);
ros::Subscriber<std_msgs::Bool> sub("/is_moving", &movingCallback);
std_msgs::Int8 msg_dist;
ros::Publisher pub("/distance", &msg_dist);


/*****************************************/
/*      Class declaration                */
/*****************************************/
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

/**
 * Write strings
 */
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {

      std::string value = pCharacteristic->getValue();
      
      if (value.length() > 0) {
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        doorNumberStr = "";
        
        for (int i=0; i<value.length(); i++) {
          doorNumberStr = doorNumberStr + value[i];
        }

        publishDoorNumber();
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
      }
    }
};



void setup()
{  
  // Create the BLE Device
  BLEDevice::init("ESP32 gr1");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Write strings
  pCharacteristic->setCallbacks(new MyCallbacks());
  
  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

  // Set outputs
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);


  // Create node 
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pubDoorNumber);
  nh.advertise(pub);
  
  // Start the measure 
  if (!measureDistance.begin()) {
    while(true);
  }
}

void loop() {
  // Disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      oldDeviceConnected = deviceConnected;
  }
  
  // Connecting
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected; // do stuff here on connecting
  }
  
  nh.spinOnce();
  delay(100);
}

/**
 * Convert the door string to an integer 
 */
int charToInt() {
  if (doorNumberStr == "1") {
    doorNumber = 1;
    return 1;
  }
  if (doorNumberStr == "2") {
    doorNumber = 2;
    return 2;
  }
  if (doorNumberStr == "3") {
    doorNumber = 3;
    return 3;
  }
}

/**
 * Take a measure of distance and play sound if the distance change compared to the distance to the door
 * @param distanceToDoor distance to the current door 
 */
void verifyDistance(int distanceToDoor){
  VL53L0X_RangingMeasurementData_t measure;  // Measurement pointer declaration
  measureDistance.rangingTest(&measure, false);  // Take the measure
  
  if (measure.RangeStatus != 4) {  // Good measure 
    int distanceCm = measure.RangeMilliMeter / 10; // Convert to cm

    msg_dist.data = distanceCm;
    pub.publish(&msg_dist);
    
    // If the distance is not between distance_min and ditance_max, send an alert to the app
    if(distanceCm < (distanceToDoor - 2) || distanceCm > (distanceToDoor + 2)) {
      playSound();
      //callApp();
      digitalWrite(LED_PIN, HIGH);
    }
    else {
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(LED_PIN, LOW);  
    }
  }
}

/**
 * Publish the door number to the topic /door_number
 */
void publishDoorNumber() {
  msg.data = charToInt();
  pubDoorNumber.publish(&msg);
}

/**
 * Play sound using millis function to avoid blocking the program
 */
void playSound() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    if(buzzerState == LOW) {
      buzzerState= HIGH;
    }
    else {
      buzzerState = LOW;
    }
    
    digitalWrite(BUZZER_PIN, buzzerState);
  }
}

/**
 * Send alert to the app when a door is opened
 * TODO: If we call this function, we need to change in charToInt() the doors number +1
 */
void callApp() {
  unsigned long currentMillis_alert = millis();

  if (currentMillis_alert - previousMillis_alert >= interval_alert) {
    previousMillis_alert = currentMillis_alert;

    if (txValue = 48) {
      txValue = 49;
    }

    else if (txValue = 49) {
      txValue = 48;
    }
    pCharacteristic->setValue(txValue);
    pCharacteristic->notify();
  }

}
