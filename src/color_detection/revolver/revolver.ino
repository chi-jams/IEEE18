
#include <AccelStepper.h>
#include <Wire.h>

const int addr = 0x43;

//conveyor
int enA = 3;
int in1 = 4;
int in2 = 5;

//solenoid
int enB = 10;
int in3 = 11;
int in4 = 12;

volatile bool doOpenHatch;

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 8, 9); //dir, step

enum COLOR{RED, GREEN, BLUE, GRAY, CYAN, MAGENTA, YELLOW};
int pos[7] = {0,450,900,1350,1800,2250,2700};
enum CMD{GET_COLOR, DUMP, KILL_ALL};

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
  
    // set all the motor control pins to outputs
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
  
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite (enA, 0);
  
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite (enB, 0);

    setConveyor(true);
  
}

void loop()
{

    if (Serial.available() > 0){
        int goTo = Serial.read() - 48;
        stepper.moveTo(pos[goTo]);
    }
    
    stepper.run();
  
    if (doOpenHatch) {
        doOpenHatch = false;
        openHatch();
    }

    delay(5000);
    openHatch();
    delay(1000);
    setConveyor(false);
    while(true){}
  
 }


void setConveyor(bool on){
    if (on){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite (enA, 150);          
    } 
    else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite (enA, 0);
    } 
}

void openHatch(){
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite (enB, 255);
    delay(500);  
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite (enB, 0);
  
}

void kill_all(){
  
    setConveyor(false);
  
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
        else if (cmd == DUMP){
            int nil = i2cGetInt();
            doOpenHatch = true;
          }
        else if (cmd == KILL_ALL){
            int nil = i2cGetInt();
            kill_all();  
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


