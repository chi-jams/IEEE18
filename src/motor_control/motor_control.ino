//software interrupts library
#include <PinChangeInt.h>

//encoder constants
const unsigned long COUNTS_PER_REV = 3200;
const float DISTANCE_PER_REV = 3.14159265 * 3.295; //pi * wheel diameter
const float DISTANCE_PER_DEG = 3.14159265 * 5.5;

const float K_P = 1/320.0;
#define ZERO_ERROR_MARGIN 30
#define MIN_MOTOR_SPEED 80
#define MAX_MOTOR_SPEED 100

//encoder pins values
unsigned int ENC_PINS[4][2] = { {A8, A9 },
                                {A10,A11},
                                {10, 11},
                                {12, 13} };

//motor pins
unsigned int PWM_PINS[4] = {2, 3, 4, 5};
unsigned int DIR_PINS[4] = {25, 24, 23, 22};

//encoder distinction enums
enum ENC {FL, FR, BL, BR};
enum TYPE {A, B};

//encoder counts
volatile long enc_counts[4];

void setup() {

    Serial.begin(9600);
  
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
  
/* for (int i = 0; i < 4; i++){
    Serial.print(enc_counts[i]);
    Serial.print(" ");  
  }
  Serial.println(); */

  drive(60);
  while(true){}
  
}

/*
 * void drive()
 * Takes in a distance in inches and drives forwards/backwards until value is reached
 */
void drive(float distance){

    resetEncoderCounts();
    
    //convert to goal encoder values
    long goals[4];
    for (int i = 0; i < 4; i++){
        goals[i] = (long) (distance * COUNTS_PER_REV / DISTANCE_PER_REV); 
    }
  
    long errors[4];
    do { 
        for (int i = 0; i < 4; i++){
            errors[i] = enc_counts[i] - goals[i];
            
            int motorVal = -errorToMotorOut(K_P, errors[i]);
            Serial.print(errors[i]);
            Serial.print("  ");
            setMotor(i, motorVal);
        }
        Serial.println();
    } while (!isZero(errors));
}

/*
 * void turn()
 * Takes in an angle in degrees and turns CW/CCW until value is reached
 */
void turn(float angle){
  
    resetEncoderCounts();

    long distance = 
    
    //convert to goal encoder values
    long goals[4];
    for (int i = 0; i < 4; i++){
        goals[i] = (long) (distance * COUNTS_PER_REV / DISTANCE_PER_REV); 
    }
  
    long errors[4];
    do { 
        for (int i = 0; i < 4; i++){
            errors[i] = enc_counts[i] - goals[i];
            
            int motorVal = -errorToMotorOut(K_P, errors[i]);
            Serial.print(errors[i]);
            Serial.print("  ");
            setMotor(i, motorVal);
        }
        Serial.println();
    } while (!isZero(errors));
  
}
/*
 * int errorToMotorOut()
 * Takes in an error and gain value and converts it to a motor output with an 
 * upper bound check of max speed
 */
int errorToMotorOut(float gain, long error){
    //no error
    if (error == 0) return 0;

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

/*
 * bool isZero()
 * Takes in the list of errors values and determines if all of them are within the 
 * margin set to be "zero". This helps prevent the encoders from causing the robot
 * to oscillate back and forth due to it not being able to hit exactly zero. 
 */
bool isZero(long* errors){

    bool zero = true;
    for (int i = 0; i < 4; i++)
        if (errors[i] > ZERO_ERROR_MARGIN) zero = false;
    return zero;
}


/*
 * void resetEncoderCounts()
 * Resets each of the encoder counts to zero 
 */
void resetEncoderCounts(){
    for (int i = 0; i < 4; i++) enc_counts[i] = 0;  
}


/*
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

