
enum {BR, BL, FR, FL};
const int PIN_E[4] = {3,5,6,9};
const int PIN_M[4] = {2,4,7,8};

const int SPEED = 100;

void setup(){

  for (int i = 0; i < 4; i++){
    pinMode(PIN_E[i], OUTPUT);
    pinMode(PIN_M[i], OUTPUT);
  }

}

void loop(){

  Serial.println("Started");

  Forward();
  /*delay(2000);
  Backward();
  delay(2000);
  Left();
  delay(2000);
  Right();
  delay(2000);
  Stop();
  delay(2000); */
    
}

void Forward(){
  
    for (int i = 0; i < 4; i++){
      if (i%2 == 0){
        analogWrite(PIN_E[i], SPEED);
        digitalWrite(PIN_M[i], HIGH);
      }
      else {
        analogWrite(PIN_E[i], SPEED);
        digitalWrite(PIN_M[i], LOW);
      }
      
  }  
}

void Backward(){
  
  for (int i = 0; i < 4; i++){
      if (i%2 == 0){
        analogWrite(PIN_E[i], SPEED);
        digitalWrite(PIN_M[i], LOW);
      }
      else {
        analogWrite(PIN_E[i], SPEED);
        digitalWrite(PIN_M[i], HIGH);
      }
  }  
  
}

void Left(){

  for (int i = 0; i < 4; i++){
    analogWrite(PIN_E[i], SPEED);
    digitalWrite(PIN_M[i], HIGH);
  }  
  
}

void Right(){
  
  for (int i = 0; i < 4; i++){
    analogWrite(PIN_E[i], SPEED);
    digitalWrite(PIN_M[i], LOW);
  }  
  
}

void Stop(){
  
  for (int i = 0; i < 4; i++){
    analogWrite(PIN_E[i], 0);
  }  
  
}

