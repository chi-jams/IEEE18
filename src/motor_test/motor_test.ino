//L293D

//Motor A

const int motorPin1  = 9;  // Pin 14 of L293

const int motorPin2  = 10;  // Pin 10 of L293



//This will run only one time.

void setup(){

 

    //Set pins as outputs

    pinMode(motorPin1, OUTPUT);

    pinMode(motorPin2, OUTPUT);

    pinMode(motorPin3, OUTPUT);

    pinMode(motorPin4, OUTPUT);

}





void loop(){

  
    //This code  will turn Motor A clockwise for 2 sec.

    analogWrite(motorPin1, 180);

    analogWrite(motorPin2, 0);

    delay(2000); 

    //This code will turn Motor A counter-clockwise for 2 sec.

    analogWrite(motorPin1, 0);

    analogWrite(motorPin2, 180);


    delay(2000);

    

    //And this code will stop motors

    analogWrite(motorPin1, 0);

    analogWrite(motorPin2, 0);
    
    delay(2000);



}
