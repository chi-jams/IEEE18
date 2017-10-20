/*
	MOTOR PID POSITION CONTROLLER

	Authors: Easton Bornemeier & Daichi Jameson

	Program to drive and control the motors using the I2C connection from the Raspberry Pi to track destination
	position and current position

*/

long Error[10]; //difference between actual values and intended values with back-buffer storage of 10
long I_Accum; //accumulation of error over time for use on the integral term
long PID; //actual PID value calculated

int P; //proportional term
int I; //integral term
int D; //derivative term

byte Divider; //shift term to bit shift the large PID value to a more feasible range

byte M_VALS[4]; //sign values for the motor controller
byte E_VALS[4]; //speed values for the motor controller

//Motor pin values in order FL, FR, BL, BR
int M_PINS[4] = {2, 4, 7, 8};
int E_PINS[4] = {3, 5, 6, 9};


void setup(){
	for (int i = 0; i < 4, i++){
		pinMode(M_PINS[i], OUTPUT);
		pinMode(E_PINS[i], OUTPUT);
	}

}

void loop(){


}

void getError(){


}

void calcPID(){


}

void writeToMotors(){

	for (int i = 0; i < 4; i++){
		digitalWrite(M_PINS[i], M_VALS[i]);
		analogWrite(E_PINS[i], E_VALS[i]);
	}

}