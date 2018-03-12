//software interrupts library
#include <PinChangeInt.h>

//encoder pins values
unsigned int enc_pins[4][2] = { {2,3},
                                {4,5},
                                {6,7},
                                {8,9} };
//encoder distinction enums
enum ENC {FL, FR, BL, BR};
enum TYPE {A, B};

//encoder counts
volatile long enc_counts[4];

void setup() {

  for (int i = 0; i < 4; i++){
    pinMode(enc_pins[i][A], INPUT_PULLUP);
    pinMode(enc_pins[i][B], INPUT_PULLUP);
  }
  
  PCintPort::attachInterrupt(enc_pins[FL][A], FLA_changed,  CHANGE);
  PCintPort::attachInterrupt(enc_pins[FL][B], FLB_changed,  CHANGE);
  PCintPort::attachInterrupt(enc_pins[FR][A], FRA_changed,  CHANGE);
  PCintPort::attachInterrupt(enc_pins[FR][B], FRB_changed,  CHANGE);
  PCintPort::attachInterrupt(enc_pins[BL][A], BLA_changed,  CHANGE);
  PCintPort::attachInterrupt(enc_pins[BL][B], BLB_changed,  CHANGE);
  PCintPort::attachInterrupt(enc_pins[BR][A], BRA_changed,  CHANGE);
  PCintPort::attachInterrupt(enc_pins[BR][B], BRB_changed,  CHANGE);

}

void loop() {
  // put your main code here, to run repeatedly:
  
}

void encoderCount(int enc, int type){
    if (type == A){
    
      //low to high
      if (digitalRead(enc_pins[enc][A]) == HIGH)
        if (digitalRead( enc_pins[enc][B]) == LOW) enc_counts[enc]++;
        else                                       enc_counts[enc]--;
      //high to low
      else if (digitalRead( enc_pins[enc][B]) == HIGH) enc_counts[enc]++;
        else                                           enc_counts[enc]--;
    }
    
    else 
    if (type == B){
      //low to high
      if (digitalRead(enc_pins[enc][B]) == HIGH)
        if (digitalRead( enc_pins[enc][A]) == HIGH) enc_counts[enc]++;
        else                                        enc_counts[enc]--;
      //high to low
      else
        if (digitalRead( enc_pins[enc][A]) == LOW) enc_counts[enc]++;
        else                                       enc_counts[enc]--;
    }
}


//callbacks because ISR's suck
void FLA_changed(){encoderCount(FL, A);}
void FLB_changed(){encoderCount(FL, B);}
void FRA_changed(){encoderCount(FR, A);}
void FRB_changed(){encoderCount(FR, B);}
void BLA_changed(){encoderCount(BL, A);}
void BLB_changed(){encoderCount(BL, B);}
void BRA_changed(){encoderCount(BR, A);}
void BRB_changed(){encoderCount(BR, B);}

