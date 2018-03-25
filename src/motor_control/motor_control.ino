//software interrupts library
#include <PinChangeInt.h>
//i2c library
#include <Wire.h>

const unsigned long COUNTS_PER_REV = 3200;
const unsigned int  MOVING_AVG_SIZE = 25;

const float DISTANCE_PER_REV = 3.14159265 * 3.295; //pi * wheel diameter
const float DISTANCE_PER_DEG = 3.14159265 * 30.0 / 360; //TODO Remove with IMU
const float ROBOT_LENGTH = 10.75;

//control info
const float K_P = 1/80.0;
const int ZERO_ERROR_MARGIN = 240;
int  MIN_MOTOR_SPEED = 110;
int  MAX_MOTOR_SPEED = 150;

//positional info
enum POS_INFO{X, Y, R, NUM_AXIS};
volatile int current_pos[NUM_AXIS];
volatile int target_pos[NUM_AXIS];

//encoder info
volatile long enc_counts[4];
unsigned int ENC_PINS[4][2] = { {A8, A9 },
                                {10, 11},
                                {A10,A11},
                                {12, 13} };


//motor pins
unsigned int PWM_PINS[4] = {2, 3, 4, 5};
unsigned int DIR_PINS[4] = {23, 22, 25, 24};

//motor distinction
enum ENC {FL, FR, BL, BR};
enum TYPE {A, B};

//communication info
enum COMMANDS {GET_TARG_POS, GET_CUR_POS};
bool completed_movement = true;
const int I2C_ADDR = 0x42;




void setup() {

    Serial.begin(9600);
    
    Wire.begin(I2C_ADDR);
    Wire.onReceive(getPosition);
    Wire.onRequest(checkDone);
  
    for (int i = 0; i < 4; i++){
        pinMode(ENC_PINS[i][A], INPUT_PULLUP);
        pinMode(ENC_PINS[i][B], INPUT_PULLUP);
  
        pinMode(DIR_PINS[i], OUTPUT);
        pinMode(PWM_PINS[i], OUTPUT);
    }
    
    PCintPort::attachInterrupt(ENC_PINS[FL][A], FLA_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[FL][B], FLB_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[FR][A], FRA_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[FR][B], FRB_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[BL][A], BLA_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[BL][B], BLB_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[BR][A], BRA_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[BR][B], BRB_changed,  CHANGE);

}

void loop() {

  
}

/*
 * void drive()
 * Takes in a distance in inches and drives forwards/backwards until value is reached
 */
void drive(float distance){

    resetEncoderCounts();
    
    long goals[4];
    for (int i = 0; i < 4; i++){
        goals[i] = (long) (distance * COUNTS_PER_REV / DISTANCE_PER_REV); 
    }
  
    long errors[4][MOVING_AVG_SIZE];
    int replaceIndex = 0;
    bool filled = false;
    do { 
      
        for (int i = 0; i < 4; i++){
            errors[i][replaceIndex] = enc_counts[i] - goals[i];
            if (replaceIndex == MOVING_AVG_SIZE-1) filled = true;
            if (!filled) continue;
            
            int motorVal = -errorToMotorOut(K_P, errors[i][replaceIndex]);
            Serial.print(average(errors[i]));
            Serial.print("  ");
            
            setMotor(i, motorVal);
        }
        Serial.println();
        replaceIndex = (replaceIndex + 1) % MOVING_AVG_SIZE;
    } while (!isZero(errors));
    Serial.println("DONE");
    stopMotors();
}

/*
 * void turn()
 * Takes in an angle in degrees and turns CW/CCW until value is reached
 */
 //TODO THIS NEEDS TO USE THE IMU WHEN IT GETS HERE, NOT ENCODERS
void turn(float angle){
    
    drive(-ROBOT_LENGTH * abs(angle)/180.0);
  
    resetEncoderCounts();

    float distance = DISTANCE_PER_DEG * abs(angle); 
    
    //convert to goal encoder values
    long goals[4];
    for (int i = 0; i < 4; i++){
      
        if (angle > 0)
            goals[i] = (long) (distance * COUNTS_PER_REV / DISTANCE_PER_REV) 
                              * ((i == FR || i == BR) ? 1: 0); 
        else 
            goals[i] = (long) (distance * COUNTS_PER_REV / DISTANCE_PER_REV) 
                              * ((i == FR || i == BR) ? 0: 1);
       
    }
  
    long errors[4][MOVING_AVG_SIZE];
    int replaceIndex;
    bool filled = false;
    do { 
        for (int i = 0; i < 4; i++){
          
            errors[i][replaceIndex] = enc_counts[i] - goals[i];
            if (replaceIndex == MOVING_AVG_SIZE-1) filled = true;
            if (!filled) continue;
            
            int motorVal = -errorToMotorOut(K_P, errors[i][replaceIndex]);
            Serial.print(average(errors[i]));
            Serial.print("  ");
            if (angle < 0){
                if (i != FR && i != BR) setMotor(i, motorVal);
                setMotor(FR, 50);
                setMotor(BR, 50);  
                errors[FR][replaceIndex] = 0;
                errors[BR][replaceIndex] = 0;
            }
            else {
              if (i != FL && i != BL) setMotor(i, motorVal);
                setMotor(FL, 50);
                setMotor(BL, 50);  
                errors[FL][replaceIndex] = 0;
                errors[BL][replaceIndex] = 0;
            }
            
            
        }
        Serial.println();
        replaceIndex = (replaceIndex + 1) % MOVING_AVG_SIZE;
    } while (!isZero(errors));

    drive(-ROBOT_LENGTH * abs(angle)/145.0);
    stopMotors();
  
}
/*
 * int errorToMotorOut()
 * Takes in an error and gain value and converts it to a motor output with an 
 * upper bound check of max speed
 */
int errorToMotorOut(float gain, long error){
    //no error
    if (abs(error) <= ZERO_ERROR_MARGIN) return 0;

    //motor level with base power + error * gain
    int motorOut = gain * -error + (error <= 0 ? MIN_MOTOR_SPEED : -MIN_MOTOR_SPEED);

    //bound check
    if (motorOut > MAX_MOTOR_SPEED) return MAX_MOTOR_SPEED;
    if (motorOut < -MAX_MOTOR_SPEED) return -MAX_MOTOR_SPEED;

    return motorOut;
}

/*
 * int setMotor()
 * Takes in a motor index and pwm value and corrects it to properly set the direction
 * pin and the abs(pwm) as the pwm
 */
void setMotor(int m, int pwm){
    //set direction
    digitalWrite(DIR_PINS[m], pwm > 0 ? LOW:HIGH);

    //set speed
    if (pwm < 0)   pwm = -pwm;
    analogWrite(PWM_PINS[m], pwm);
}

void stopMotors(){
  for(int i = 0; i < 4; i++){
      setMotor(i, 0);  
  } 
}

/*
 * bool isZero()
 * Takes in the list of errors values and determines if all of them are within the 
 * margin set to be "zero". This helps prevent the encoders from causing the robot
 * to oscillate back and forth due to it not being able to hit exactly zero. 
 */
bool isZero(long errors[][MOVING_AVG_SIZE]){

    bool zero = true;
    for (int i = 0; i < 4; i++){
        if (abs(average(errors[i])) > ZERO_ERROR_MARGIN) zero = false;
    }
    return zero;
}

long average(long* errors){
    float sum = 0;
    for (int i = 0; i < MOVING_AVG_SIZE; i++){
        sum += errors[i];
    }
    return (long)(sum /MOVING_AVG_SIZE);
  
}

/*
 * void resetEncoderCounts()
 * Resets each of the encoder counts to zero 
 */
void resetEncoderCounts(){
    for (int i = 0; i < 4; i++) enc_counts[i] = 0;  
}


/*
 * void encoderCount()
 * Method indirectly used by the interrupts to determine the direction the wheel
 * is going based on which value changed and the state of the one that did 
 * not change
 * 
 */
void encoderCount(int enc, int type){
    int inc = (enc == FR || enc == BR) ? 1 : -1;
  
    if (type == A){
    
      //low to high
      if (digitalRead(ENC_PINS[enc][A]) == HIGH)
          if (digitalRead( ENC_PINS[enc][B]) == LOW)  enc_counts[enc]+=inc;
          else                                        enc_counts[enc]-=inc;
      //high to low
      else 
          if (digitalRead( ENC_PINS[enc][B]) == HIGH) enc_counts[enc]+=inc;
          else                                        enc_counts[enc]-=inc;
    }
    
    else 
    if (type == B){
      //low to high
      if (digitalRead(ENC_PINS[enc][B]) == HIGH)
          if (digitalRead( ENC_PINS[enc][A]) == HIGH) enc_counts[enc]+=inc;
          else                                        enc_counts[enc]-=inc;
      //high to low
      else
          if (digitalRead( ENC_PINS[enc][A]) == LOW)  enc_counts[enc]+=inc;
          else                                        enc_counts[enc]-=inc;
    }
}

/*
 * A series of void arg void callback methods that pass arguments needed to 
 * the actual interrupt function since interrupts do not allow argument passing
 */
void FLA_changed(){encoderCount(FL, A);}
void FLB_changed(){encoderCount(FL, B);}
void FRA_changed(){encoderCount(FR, A);}
void FRB_changed(){encoderCount(FR, B);}
void BLA_changed(){encoderCount(BL, A);}
void BLB_changed(){encoderCount(BL, B);}
void BRA_changed(){encoderCount(BR, A);}
void BRB_changed(){encoderCount(BR, B);}


void getPosition(int num_bytes) {
    // The smbus implementation sends over a command and the number of bytes it has in its payload
    // before it actually, dir sends over the real data
    int cmd = Wire.read();
    int bytes = Wire.read();
    
    // Do a sanity check to make sure that the data received follows the correct format
    if (num_bytes == bytes + 2) {
        if (cmd == GET_TARG_POS) {
            for(int i = 0; i < 3; i++)
                target_pos[i] = i2cGetInt();
            //dir = i2cGetInt(); TODO: Don't think this is needed, handled on the pi side
        }
        else if (cmd == GET_CUR_POS) {
            for (int i = 0; i < 3; i++)
                current_pos[i] = i2cGetInt();
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


// Reads the next 2 bytes from the i2c bus and splices them together to make a signed 16-bit integer.
int i2cGetInt() {
    return ((Wire.read() << 8) | Wire.read());
}

void dumpData() {
    while (Wire.available()) Wire.read();
}


