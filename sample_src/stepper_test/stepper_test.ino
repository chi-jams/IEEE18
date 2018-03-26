
#include <AccelStepper.h>
#include <Wire.h>

const int addr = 0x43;

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 8, 9); //step, dir

enum COLOR{RED, GREEN, BLUE, GRAY, CYAN, MAGENTA, YELLOW};
int pos[7] = {0,450,900,1350,1800,2250,2700};
enum CMD{GET_COLOR};

void setup()
{ 
  Serial.begin(9600);
  Wire.begin(addr);
  Wire.onReceive(getColor);
  Wire.onRequest(nothing);
  
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  
  stepper.setMaxSpeed(50000);
  stepper.setAcceleration(3000000);
}

void loop()
{

  if (Serial.available() > 0){
    int goTo = Serial.read() - 48;
    stepper.moveTo(pos[goTo]);
  }
  
  stepper.run();
 }

void getColor(int num_bytes) {
    // The smbus implementation sends over a command and the number of bytes it has in its payload
    // before it actually, dir sends over the real data
    int cmd = Wire.read();
    int bytes = Wire.read();
    
    // Do a sanity check to make sure that the data received follows the correct format
    if (num_bytes == bytes + 2) {
        if (cmd == GET_COLOR) {
          int color = i2cGetInt();
          stepper.moveTo(color);
        }
        else {
            dumpData();
        }
    }
    else {
        // We have an unexpected message, throw it out.
        dumpData();   
    }
}

void nothing(){}

// Reads the next 2 bytes from the i2c bus and splices them together to make a signed 16-bit integer.
int i2cGetInt() {
    return ((Wire.read() << 8) | Wire.read());
}

void dumpData() {
    while (Wire.available()) Wire.read();
}


