int buzzerPin = 4 ;//the pin of the active buzzer

void setup(){
  pinMode(buzzerPin,OUTPUT);//initialize the buzzer pin as an output
}

void loop(){

  unsigned char i;

  while(1){
    for(i=0;i<50;i++){
      digitalWrite(buzzerPin,HIGH);
      delay(1);
      digitalWrite(buzzerPin,LOW);
      delay(1);//wait for 1ms
    }
  }
}
