
#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 8, 9);

int pos[7] = {500,1000,1500,2000,2500,3000, 3500};

void setup()
{ 
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  
  stepper.setMaxSpeed(1000000);
  stepper.setAcceleration(1000000);
}

void loop()
{

  for (int i = 0; i < 7; i++){
      if (stepper.distanceToGo() == 0){
        delay(500);
        stepper.moveTo(pos[i]);
      }
      else {i--;}
      stepper.run();
    }
}
