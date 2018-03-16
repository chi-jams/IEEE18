/* 
    Navigation Arduino

    Module that contains the control loop for moving the robot chassis around
    the playing field. Uses I2C to communicate with the navigatin Pi in order
    to receive location instructions
*/

#include <Wire.h>
#include <stdio.h> // TODO: Remove when debug is done

const int I2C_ADDR = 0x42;

int x, y, r;
int dir;

// TODO: Remove debug code
int l_x, l_y, l_r;
int l_dir;

void setup() {
    Serial.begin(9600);

    Wire.begin(I2C_ADDR);
    Wire.onReceive(parseAction);

}

void loop() {
  // put your main code here, to run repeatedly:

    if (l_x != x || l_y != y || l_r != r) {
        Serial.println(sprintf("Pos: (%d, %d, %d)", x, y, r);
        l_x = x;
        l_y = y;
        l_r = r;
    }
}

void getPosition(int num_bytes) {
    Serial.println(sprintf("There are %d bytes", num_bytes));

    if (num_bytes != 6 || num_bytes != 8)
        Serial.println("Wtf");

    if (num_bytes == 10 || num_bytes == 9) {
        Serial.println("Oh it's the command");
        int cmd = 0;
        int d = Wire.read();
        if (num_buytes == 10) {
            cmd = (d << 8) | Wire.read();
        } 
        Serial.println(sprintf("cmd: %d", cmd));
    }

    // lol wtf am I doing
    x = (Wire.read() << 8) | Wire.read();
    y = (Wire.read() << 8) | Wire.read();
    r = (Wire.read() << 8) | Wire.read();
}

